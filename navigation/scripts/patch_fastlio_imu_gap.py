"""Install the bounded IMU gap recovery policy in a patched FAST-LIO tree."""
from pathlib import Path
import sys

source = Path(sys.argv[1]) / "src"
mapping_path = source / "laserMapping.cpp"
imu_path = source / "IMU_Processing.hpp"
mapping = mapping_path.read_text()
imu = imu_path.read_text()
marker = "// arx_imu_gap_recovery"
if marker in mapping:
    sys.exit(0)

def replace_once(content, before, after):
    if content.count(before) != 1:
        raise ValueError(f"Unexpected FAST-LIO source near: {before[:100]}")
    return content.replace(before, after)

mapping = replace_once(mapping, "double max_imu_gap_sec = 0.0;", """double max_imu_gap_sec = 0.0;
// arx_imu_gap_recovery
constexpr double kMaximumRecoverableImuGap = 0.35;
constexpr double kMaximumRecoveryTime = 1.0;
sensor_msgs::msg::Imu::ConstSharedPtr pending_imu_gap;
double imu_recovery_stamp = 0.0;
bool imu_recovery_anchor_pending = false;
bool imu_recovery_needs_reset = false;
bool imu_recovery_match = false;
""")
callback_start = mapping.index("    if (max_imu_gap_sec > 0.0 && last_timestamp_imu > 0.0 &&")
callback_end = mapping.index("    if (timestamp < last_timestamp_imu)", callback_start)
mapping = mapping[:callback_start] + """    mtx_buffer.lock();
    if (imu_recovery_needs_reset) {
        mtx_buffer.unlock();
        return;
    }
    if (max_imu_gap_sec > 0.0 && last_timestamp_imu > 0.0) {
        const double gap = timestamp - last_timestamp_imu;
        if (gap <= 0.0) {
            mtx_buffer.unlock();
            return;  // duplicate or reordered IMU sample
        }
        if (gap > max_imu_gap_sec) {
            const double pending_stamp = pending_imu_gap ?
                get_time_sec(pending_imu_gap->header.stamp) : 0.0;
            if (!pending_imu_gap || timestamp <= pending_stamp ||
                timestamp - pending_stamp > max_imu_gap_sec) {
                pending_imu_gap = msg;
                mtx_buffer.unlock();
                return;
            }
            const double confirmed_gap = pending_stamp - last_timestamp_imu;
            if (confirmed_gap > kMaximumRecoverableImuGap || imu_recovery_stamp > 0.0) {
                std::cerr << "IMU gap: holding pose output; stationary reinitialization required; dt="
                          << confirmed_gap << std::endl;
                imu_recovery_needs_reset = true;
            } else {
                std::cerr << "IMU gap: waiting for a new lidar match; dt="
                          << confirmed_gap << std::endl;
                imu_recovery_stamp = pending_stamp;
                imu_recovery_anchor_pending = true;
            }
            lidar_buffer.clear();
            time_buffer.clear();
            imu_buffer.clear();
            lidar_pushed = false;
            if (!imu_recovery_needs_reset) imu_buffer.push_back(pending_imu_gap);
        }
        pending_imu_gap.reset();
        if (imu_recovery_needs_reset) {
            mtx_buffer.unlock();
            return;
        }
    }

""" + mapping[callback_end:]
assert mapping.count("    double cur_time = get_time_sec(msg->header.stamp);\n    double preprocess_start_time = omp_get_wtime();") == 2
mapping = mapping.replace("""    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();""", """    double cur_time = get_time_sec(msg->header.stamp);
    if (imu_recovery_needs_reset || (imu_recovery_stamp > 0.0 && cur_time < imu_recovery_stamp)) {
        mtx_buffer.unlock();
        return;
    }
    double preprocess_start_time = omp_get_wtime();""")
mapping = replace_once(mapping, "    if (lidar_buffer.empty() || imu_buffer.empty()) {", """    if (pending_imu_gap || imu_recovery_needs_reset || lidar_buffer.empty() || imu_buffer.empty()) {""")
mapping = replace_once(mapping, """        if(sync_packages(Measures))
        {
            if (flg_first_scan)""", """        if(sync_packages(Measures))
        {
            if (Measures.imu.empty()) return;
            if (imu_recovery_stamp > 0.0 && Measures.lidar_beg_time < imu_recovery_stamp) return;
            if (imu_recovery_stamp > 0.0 && Measures.lidar_end_time - imu_recovery_stamp > kMaximumRecoveryTime) {
                RCLCPP_WARN(this->get_logger(), "IMU gap: lidar recovery timed out; holding pose output");
                imu_recovery_needs_reset = true;
                return;
            }
            if (flg_first_scan)""")
mapping = replace_once(mapping, "            p_imu->Process(Measures, kf, feats_undistort);", """            if (imu_recovery_anchor_pending) {
                p_imu->RebaseAfterGap(Measures.imu.front(), kf);
                imu_recovery_anchor_pending = false;
            }
            p_imu->Process(Measures, kf, feats_undistort);""")
mapping = replace_once(mapping, "            lasermap_fov_segment();", """            if (imu_recovery_stamp == 0.0) lasermap_fov_segment();""")
mapping = replace_once(mapping, """            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();
            euler_cur""", """            if (imu_recovery_stamp > 0.0) {
                auto predicted = kf.get_x();
                auto covariance = kf.get_P();
                imu_recovery_match = false;
                kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
                auto corrected = kf.get_x();
                const V3D delta = corrected.pos - predicted.pos;
                const double turn = Eigen::AngleAxisd((predicted.rot.conjugate() * corrected.rot).toRotationMatrix()).angle();
                if (!imu_recovery_match || !delta.allFinite() || delta.norm() > .4 ||
                    !std::isfinite(turn) || turn > .35) {
                    kf.change_x(predicted);
                    kf.change_P(covariance);
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "IMU gap recovered with a fresh lidar match");
                imu_recovery_stamp = 0.0;
            } else {
                kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            }
            state_point = kf.get_x();
            euler_cur""")
anchor = "    if (effct_feat_num < 1)"
mapping = replace_once(mapping, anchor, """    if (imu_recovery_stamp > 0.0) imu_recovery_match = false;
""" + anchor)
end = mapping.index("\n}", mapping.index("        if (extrinsic_est_en)\n        {", mapping.index("    if (effct_feat_num < 1)")))
mapping = mapping[:end] + """
    if (imu_recovery_stamp > 0.0) {
        // During recovery require the retained map to constrain X, Y and yaw.
        Eigen::MatrixXd planar(effct_feat_num, 3);
        planar.leftCols<2>() = ekfom_data.h_x.leftCols<2>();
        planar.col(2) = ekfom_data.h_x.middleCols<3>(3) * (s.rot.conjugate() * V3D::UnitZ());
        const Eigen::Matrix3d information = planar.transpose() * planar;
        const Eigen::LDLT<Eigen::Matrix3d> rank(information);
        imu_recovery_match = effct_feat_num >= 30 && res_mean_last < .06 &&
            rank.info() == Eigen::Success && rank.isPositive() &&
            rank.vectorD().minCoeff() > .02 * effct_feat_num;
        if (!imu_recovery_match) ekfom_data.valid = false;
    }
""" + mapping[end:]

imu = replace_once(imu, "  void Reset();", """  void Reset();
  void RebaseAfterGap(
    const sensor_msgs::msg::Imu::ConstSharedPtr &anchor,
    esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state);""")
imu = replace_once(imu, "void ImuProcess::UndistortPcl(", """void ImuProcess::RebaseAfterGap(
    const sensor_msgs::msg::Imu::ConstSharedPtr &anchor,
    esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state)
{
  if (imu_need_init_) return;
  // Preserve the mapped pose and biases. Do not integrate the unobserved gap.
  auto state = kf_state.get_x();
  state.vel.setZero();
  kf_state.change_x(state);
  auto covariance = kf_state.get_P();
  covariance.diagonal().head<6>().array() += .04;
  covariance.diagonal().segment<3>(12).array() += .25;
  kf_state.change_P(covariance);
  last_imu_ = anchor;
  last_lidar_end_time_ = rclcpp::Time(anchor->header.stamp).seconds();
  acc_s_last.setZero();
  angvel_last.setZero();
}

void ImuProcess::UndistortPcl(""")
mapping_path.write_text(mapping)
imu_path.write_text(imu)

