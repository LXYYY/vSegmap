#ifndef SEGMAPPER_SEGMAPPER_HPP_
#define SEGMAPPER_SEGMAPPER_HPP_

#include <string>
#include <vector>

#include <laser_slam/benchmarker.hpp>
#include <laser_slam/parameters.hpp>
#include <laser_slam/incremental_estimator.hpp>
#include <laser_slam_ros/laser_slam_worker.hpp>
#include <segmatch/common.hpp>
#include <segmatch/local_map.hpp>
#include <segmatch_ros/common.hpp>
#include <segmatch_ros/segmatch_worker.hpp>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>

#include "segmapper/SaveMap.h"
#include "segmapper/v_slam_worker.h"

struct SegMapperParams {
  // Multi robot parameters.
  int number_of_robots;
  std::string robot_prefix;

  bool clear_local_map_after_loop_closure = true;

  // Enable publishing a tf transform from world to odom.
  bool publish_world_to_odom;
  std::string world_frame;
  double tf_publication_rate_hz;

  // Trajectory estimator parameters.
  laser_slam::EstimatorParams online_estimator_params;
}; // struct SegMapperParams

class SegMapper {

 public:
  explicit SegMapper(ros::NodeHandle& n);
  ~SegMapper();

  /// \brief A thread function for localizing and closing loops with SegMatch.
  void segMatchThread();

 private:
  // Get ROS parameters.
  void getParameters();

  /// The local map for each \c LaserSlamWoker.
  std::vector<segmatch::LocalMap<segmatch::PclPoint, segmatch::MapPoint>> local_maps_;
  std::vector<std::mutex> local_maps_mutexes_;

  // Node handle.
  ros::NodeHandle& nh_;

  v_slam::VSlamWorker v_slam_worker_;
  v_slam::VSlamWorkerParams v_slam_params_;

  // Publisher of the local maps
  ros::Publisher local_map_pub_;
  static constexpr unsigned int kPublisherQueueSize = 50u;

  // Parameters.
  SegMapperParams params_;
  laser_slam::BenchmarkerParams benchmarker_params_;

  tf::TransformBroadcaster tf_broadcaster_;

  // SegMatch objects.
  segmatch_ros::SegMatchWorkerParams segmatch_worker_params_;
  segmatch_ros::SegMatchWorker segmatch_worker_;
  static constexpr double kSegMatchSleepTime_s = 0.01;

  std::vector<unsigned int> skip_counters_;
  unsigned int deactivate_track_when_skipped_x_ = 5u;
  std::vector<bool> first_points_received_;
  
  // Pose of the robot when localization occured. Used to compute statistics on dead-reckoning
  // distances.
  laser_slam::SE3 pose_at_last_localization_;
  bool pose_at_last_localization_set_ = false;


  static constexpr laser_slam::Time kHeadDurationToExport_ns = 60000000000u;

public:

private:
  std::string mimic_scan_topic;
  std::string mimic_pose_topic;
  double mimic_pub_rate_hz;
}; // SegMapper

#endif /* SEGMAPPER_SEGMAPPER_HPP_ */
