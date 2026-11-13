#!/usr/bin/env python3
"""Apply to the staged FAST-LIO only; original upstream remains untouched."""
from pathlib import Path
import sys

p = Path(sys.argv[1]) / 'src/laserMapping.cpp'
s = p.read_text()
if 'arx_dual_frames_and_twist' in s:
    print('Base patch already present')
    # The gravity extension below can also upgrade an earlier applied version.
    base_patched = True
else:
    base_patched = False
if base_patched:
    # Re-run only the additive gravity alignment extension.
    imu = p.parent/'IMU_Processing.hpp'
    text = imu.read_text()
    if 'align_gravity_world' not in text:
        text = text.replace('  ofstream fout_imu;', '  bool align_gravity_world = false;\n  ofstream fout_imu;')
        text = text.replace('  init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2);', '''  init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2);
  if (align_gravity_world) {
    init_state.rot = SO3(Eigen::Quaterniond::FromTwoVectors(mean_acc.normalized(), V3D(0,0,1)));
    init_state.grav = S2(V3D(0,0,-G_m_s2));
  }''')
        imu.write_text(text)
        anchor = '        this->declare_parameter<bool>("publish.sensor_only_tf", false);'
        s = s.replace(anchor, '''        this->declare_parameter<bool>("common.align_gravity_world", false);
        this->get_parameter("common.align_gravity_world", p_imu->align_gravity_world);
''' + anchor)
        p.write_text(s)
    s = p.read_text()
    if 'arx_pose_covariance_order' not in s:
        begin = s.index('    for (int i = 0; i < 6; i ++)', s.index('void publish_odometry('))
        end = s.index('    // State velocity', begin)
        s = s[:begin] + '''    // arx_pose_covariance_order: state order is pos, rot, extrinsics, vel, bg, ba, gravity.
    Eigen::Matrix<double,6,6> Jpose = Eigen::Matrix<double,6,6>::Identity();
    Jpose.block<3,3>(3,3) = state_point.rot.toRotationMatrix();
    Eigen::Matrix<double,6,6> Cpose = Jpose * P.block<6,6>(0,0) * Jpose.transpose();
    for(int i=0;i<6;++i) for(int j=0;j<6;++j)
        odomAftMapped.pose.covariance[i*6+j] = Cpose(i,j);

''' + s[end:]
        p.write_text(s)
    raise SystemExit(0)
s = s.replace('bool sensor_only_tf = false;', '''// arx_dual_frames_and_twist
bool sensor_only_tf = false;
string odom_frame_name = "lidar_odom", imu_frame_name = "lio_imu", lidar_frame_name = "livox_frame";
double max_imu_gap_sec = 0.0;''')
# Replace frame assignments, leaving parameter defaults as literal strings.
s = s.replace('= "lidar_odom";', '= odom_frame_name;').replace('="lidar_odom";', '= odom_frame_name;')
s = s.replace('sensor_only_tf ? "lio_imu" : "livox_frame"', 'sensor_only_tf ? imu_frame_name : lidar_frame_name')
s = s.replace('transform.child_frame_id = "lio_imu";', 'transform.child_frame_id = imu_frame_name;')
anchor = '        this->declare_parameter<bool>("publish.sensor_only_tf", false);'
assert anchor in s
s = s.replace(anchor, '''        this->declare_parameter<string>("frames.odom", "lidar_odom");
        this->declare_parameter<string>("frames.imu", "lio_imu");
        this->declare_parameter<string>("frames.lidar", "livox_frame");
        this->declare_parameter<double>("common.max_imu_gap_sec", 0.0);
        this->get_parameter("frames.odom", odom_frame_name);
        this->get_parameter("frames.imu", imu_frame_name);
        this->get_parameter("frames.lidar", lidar_frame_name);
        this->get_parameter("common.max_imu_gap_sec", max_imu_gap_sec);
''' + anchor)
anchor = '    double timestamp = get_time_sec(msg->header.stamp);'
assert s.count(anchor) == 1
s = s.replace(anchor, anchor + '''
    if (max_imu_gap_sec > 0.0 && last_timestamp_imu > 0.0 &&
        (timestamp <= last_timestamp_imu || timestamp - last_timestamp_imu > max_imu_gap_sec)) {
        std::cerr << "IMU discontinuity: stopping LIO; stationary reinitialization required" << std::endl;
        rclcpp::shutdown();
        return;
    }
''')
# Original code published before updating covariance, leaving covariance one scan late.
assert '    pubOdomAftMapped->publish(odomAftMapped);' in s
s = s.replace('    pubOdomAftMapped->publish(odomAftMapped);', '')
anchor = '  if (sensor_only_tf) {\n    geometry_msgs::msg::TransformStamped transform;'
assert anchor in s
s = s.replace(anchor, '''    // State velocity is in the gravity-aligned world; Odometry twist is in child frame.
    const Eigen::Matrix3d Rwi = state_point.rot.toRotationMatrix();
    const V3D vi = Rwi.transpose() * state_point.vel;
    const V3D wi = p_imu->angular_velocity();
    odomAftMapped.twist.twist.linear.x = vi.x();
    odomAftMapped.twist.twist.linear.y = vi.y();
    odomAftMapped.twist.twist.linear.z = vi.z();
    odomAftMapped.twist.twist.angular.x = wi.x();
    odomAftMapped.twist.twist.angular.y = wi.y();
    odomAftMapped.twist.twist.angular.z = wi.z();
    Eigen::Matrix3d Cv = Rwi.transpose() * P.block<3,3>(12,12) * Rwi;
    for (int i=0;i<3;++i) for(int j=0;j<3;++j) {
        odomAftMapped.twist.covariance[i*6+j] = Cv(i,j);
        odomAftMapped.twist.covariance[(i+3)*6+j+3] =
            (i == j ? p_imu->cov_gyr(i) : 0.0) + P(15+i,15+j);
    }
    pubOdomAftMapped->publish(odomAftMapped);

''' + anchor)
p.write_text(s)
# Apply the optional gravity extension through the idempotent upgrade path.
import subprocess
subprocess.run([sys.executable, __file__, sys.argv[1]], check=True)
p = p.parent / 'IMU_Processing.hpp'
s = p.read_text().replace('  ofstream fout_imu;', '  V3D angular_velocity() const { return angvel_last; }\n  ofstream fout_imu;')
p.write_text(s)
