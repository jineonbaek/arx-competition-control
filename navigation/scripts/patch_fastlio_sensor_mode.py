#!/usr/bin/env python3
from pathlib import Path
root=Path(__file__).resolve().parents[1];p=root/'workspace/src/FAST_LIO/src/laserMapping.cpp';s=p.read_text()
if 'bool sensor_only_tf = false;' in s:raise SystemExit('Sensor-only patch already applied')
s=s.replace('bool   runtime_pos_log', 'bool sensor_only_tf = false;\nbool   runtime_pos_log',1)
s=s.replace('laserCloudmsg.header.frame_id = "livox_frame";', 'laserCloudmsg.header.frame_id = sensor_only_tf ? "lio_imu" : "livox_frame";',1)
s=s.replace('odomAftMapped.child_frame_id = "livox_frame";', 'odomAftMapped.child_frame_id = sensor_only_tf ? "lio_imu" : "livox_frame";',1)
s=s.replace('  // Publish tf from lidar_odom to base_link','''  if (sensor_only_tf) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odomAftMapped.header;
    transform.child_frame_id = "lio_imu";
    transform.transform.translation.x = odomAftMapped.pose.pose.position.x;
    transform.transform.translation.y = odomAftMapped.pose.pose.position.y;
    transform.transform.translation.z = odomAftMapped.pose.pose.position.z;
    transform.transform.rotation = odomAftMapped.pose.pose.orientation;
    tf_br->sendTransform(transform);
    return;
  }
  // Publish tf from lidar_odom to base_link''',1)
s=s.replace('this->declare_parameter<bool>("publish.path_en", true);','this->declare_parameter<bool>("publish.sensor_only_tf", false);\n        this->declare_parameter<bool>("publish.path_en", true);',1)
s=s.replace('this->get_parameter_or<bool>("publish.path_en", path_en, true);','this->get_parameter_or<bool>("publish.sensor_only_tf", sensor_only_tf, false);\n        this->get_parameter_or<bool>("publish.path_en", path_en, true);',1)
a=s.index('    PointCloudXYZI::Ptr laserCloudFullRes',s.index('void publish_map('));b=s.index('    sensor_msgs::msg::PointCloud2 laserCloudmsg;',a)
old=s[a:b]
s=s[:a]+'''    if (sensor_only_tf) {
        if (!ikdtree.Root_Node) return;
        PointVector map_points;
        ikdtree.flatten(ikdtree.Root_Node, map_points, NOT_RECORD);
        pcl_wait_pub->points = std::move(map_points);
        pcl_wait_pub->width = pcl_wait_pub->points.size();
        pcl_wait_pub->height = 1;
        pcl_wait_pub->is_dense = false;
    } else {
'''+old+'''    }

'''+s[b:]
s=s.replace('void save_to_pcd()\n{\n    pcl::PCDWriter pcd_writer;\n    pcd_writer.writeBinary(map_file_path, *pcl_wait_pub);\n}', '''bool save_to_pcd()
{
    if (pcl_wait_pub->empty() || map_file_path.empty()) return false;
    pcl::PCDWriter pcd_writer;
    const std::string temporary = map_file_path + ".tmp";
    if (pcd_writer.writeBinary(temporary, *pcl_wait_pub) != 0) return false;
    return std::rename(temporary.c_str(), map_file_path.c_str()) == 0;
}''')
s=s.replace('save_to_pcd();\n            res->success = true;\n            res->message = "Map saved.";', 'res->success = save_to_pcd();\n            res->message = res->success ? "Map saved." : "Map is empty or file write failed.";',1)
p.write_text(s)
print('Patched sensor-only TF, current-map publishing and atomic PCD save')
