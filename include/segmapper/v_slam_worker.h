#ifndef VSlamWorker_H
#define VSlamWorker_H

#include <queue>
#include <mutex>

#include <geometry_msgs/PoseStamped.h>
#include <laser_slam/common.hpp>
#include <laser_slam_ros/common.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace v_slam {

typedef std::pair<laser_slam::Pose, std::vector<laser_slam_ros::PointCloud>> LocalMap;
typedef std::map<ros::Time, LocalMap> LocalMapStamped;

struct VSlamWorkerParams
{
  // Map creation & filtering parameters.
  double distance_to_consider_fixed;
  bool separate_distant_map;
  bool create_filtered_map;
  double minimum_distance_to_add_pose;
  double voxel_size_m;
  int minimum_point_number_per_voxel;

  bool remove_ground_from_local_map = false;
  double ground_distance_to_robot_center_m;

  bool use_odometry_information = true;

  // Frames.
  std::string odom_frame;
  std::string sensor_frame;
  std::string world_frame;

  // Topics.
  std::string assembled_cloud_sub_topic;
  std::string assembled_pose_sub_topic;
  std::string trajectory_pub_topic;
  std::string odometry_trajectory_pub_topic;
  std::string full_map_pub_topic;
  std::string local_map_pub_topic;
  std::string distant_map_pub_topic;
  std::string get_laser_track_srv_topic;

  // Map publication.
  bool publish_local_map;
  bool publish_full_map;
  bool publish_distant_map;
  double map_publication_rate_hz;
};
// struct LaserSlamWorkerParams

static VSlamWorkerParams
getLaserSlamWorkerParams (const ros::NodeHandle& nh, const std::string& prefix)
{
  VSlamWorkerParams params;
  const std::string ns = prefix + "/VSlamWorker";

  nh.getParam (ns + "/distance_to_consider_fixed",
               params.distance_to_consider_fixed);
  nh.getParam (ns + "/separate_distant_map", params.separate_distant_map);
  nh.getParam (ns + "/create_filtered_map", params.create_filtered_map);
  nh.getParam (ns + "/minimum_distance_to_add_pose",
               params.minimum_distance_to_add_pose);
  nh.getParam (ns + "/voxel_size_m", params.voxel_size_m);
  nh.getParam (ns + "/minimum_point_number_per_voxel",
               params.minimum_point_number_per_voxel);

  nh.getParam (ns + "/remove_ground_from_local_map",
               params.remove_ground_from_local_map);
  nh.getParam (ns + "/ground_distance_to_robot_center_m",
               params.ground_distance_to_robot_center_m);

  nh.getParam (ns + "/use_odometry_information",
               params.use_odometry_information);

  nh.getParam (ns + "/odom_frame", params.odom_frame);
  nh.getParam (ns + "/sensor_frame", params.sensor_frame);
  nh.getParam (ns + "/world_frame", params.world_frame);

  nh.getParam (ns + "/publish_local_map", params.publish_local_map);
  nh.getParam (ns + "/publish_full_map", params.publish_full_map);
  nh.getParam (ns + "/publish_distant_map", params.publish_distant_map);
  nh.getParam (ns + "/map_publication_rate_hz", params.map_publication_rate_hz);

  nh.getParam (ns + "/assembled_cloud_sub_topic",
               params.assembled_cloud_sub_topic);
  nh.getParam (ns + "/assembled_pose_sub_topic",
               params.assembled_pose_sub_topic);
  nh.getParam (ns + "/trajectory_pub_topic", params.trajectory_pub_topic);
  nh.getParam (ns + "/odometry_trajectory_pub_topic",
               params.odometry_trajectory_pub_topic);
  nh.getParam (ns + "/full_map_pub_topic", params.full_map_pub_topic);
  nh.getParam (ns + "/local_map_pub_topic", params.local_map_pub_topic);
  nh.getParam (ns + "/distant_map_pub_topic", params.distant_map_pub_topic);
  nh.getParam (ns + "/get_laser_track_srv_topic",
               params.get_laser_track_srv_topic);

  return params;
}

class VSlamWorker
{
public:
  VSlamWorker ();

  void
  init (ros::NodeHandle& nh, const VSlamWorkerParams& params,
        unsigned int worker_id = 0u);

  /// \brief Register the local scans to the sliding window estimator.
  void
  scanCallback (const sensor_msgs::PointCloud2& cloud_msg_in);

  void
  poseCallback (const geometry_msgs::PoseStamped& pose_msg_in);

  // Get a vector containing the optimized point clouds recorded since
  // the last call to this method. This call clears the point cloud queue.
  bool
  getQueuedScanAndPose (LocalMap& local_map);

private:
  ros::Subscriber scan_sub_;
  ros::Subscriber pose_sub_;
  VSlamWorkerParams params_;
  unsigned int worker_id_;

  mutable std::recursive_mutex local_map_mutex_;
  LocalMapStamped local_map_stamped_;
  laser_slam_ros::PointCloud scan_;
  laser_slam::Pose pose_;
  std::queue<LocalMap> local_map_queue_;

  static constexpr unsigned int kScanSubscriberMessageQueueSize = 1u;
};
}
#endif // VSlamWorker_H
