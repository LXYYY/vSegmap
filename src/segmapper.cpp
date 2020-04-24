#include "segmapper/segmapper.hpp"

#include <stdlib.h>

#include <laser_slam/benchmarker.hpp>
#include <laser_slam/common.hpp>
#include <laser_slam_ros/common.hpp>
#include <ros/ros.h>
#include <segmatch/utilities.hpp>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

using namespace laser_slam;
using namespace laser_slam_ros;
using namespace segmatch;
using namespace segmatch_ros;

SegMapper::SegMapper(ros::NodeHandle& n)
    : nh_(n)
{
  // Load ROS parameters from server.
  getParameters();

  // TODO: it would be great to have a cleaner check here, e.g. by having the
  // segmenter interface
  // telling us if normals are needed or not. Unfortunately, at the moment the
  // segmenters are
  // created much later ...
  const std::string& segmenter_type = segmatch_worker_params_.segmatch_params.segmenter_params
      .segmenter_type;
  const bool needs_normal_estimation = (segmenter_type == "SimpleSmoothnessConstraints")
      || (segmenter_type == "IncrementalSmoothnessConstraints");

  v_slam_worker_.init(nh_, v_slam_params_);

  // Create a local map for each robot.
  std::unique_ptr<NormalEstimator> normal_estimator = nullptr;
  if (needs_normal_estimation) {
    normal_estimator = NormalEstimator::create(
        segmatch_worker_params_.segmatch_params.normal_estimator_type,
        segmatch_worker_params_.segmatch_params.radius_for_normal_estimation_m);
  }

  local_maps_.emplace_back(segmatch_worker_params_.segmatch_params.local_map_params,
                           std::move(normal_estimator));

  // Configure benchmarker
  Benchmarker::setParameters(benchmarker_params_);

  // Initialize the SegMatchWorker.
  if (segmatch_worker_params_.localize || segmatch_worker_params_.close_loops) {
    segmatch_worker_.init(n, segmatch_worker_params_, params_.number_of_robots);
  }
}

SegMapper::~SegMapper()
{
}

void SegMapper::segMatchThread()
{
  DLOG(INFO)<< "segMatchThread loop starting" << std::endl;
  unsigned int track_id = 0;
  Pose current_pose;
  ros::Rate loop_rate(mimic_pub_rate_hz);
  while (ros::ok())
  {
    v_slam::LocalMap local_map;
    if (v_slam_worker_.getQueuedScanAndPose(local_map))
    {
      DLOG(INFO) << "processing scan in stamp " << local_map.first.time_ns << std::endl;
      current_pose = local_map.first;
      local_maps_[track_id].updatePoseAndAddPoints(local_map.second, current_pose);
      RelativePose loop_closure;

      // If there is a loop closure.
      if (segmatch_worker_.processLocalMap(local_maps_[track_id], current_pose, track_id,
              &loop_closure))
      {
        DLOG(INFO)<< "Found loop closure! track_id_a: " << loop_closure.track_id_a <<
        " time_a_ns: " << loop_closure.time_a_ns <<
        " track_id_b: " << loop_closure.track_id_b <<
        " time_b_ns: " << loop_closure.time_b_ns;
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void SegMapper::getParameters()
{
  // SegMapper parameters.
  const std::string ns = "/SegMapper";
  nh_.getParam(ns + "/number_of_robots", params_.number_of_robots);
  nh_.getParam(ns + "/robot_prefix", params_.robot_prefix);

  CHECK_GE(params_.number_of_robots, 0u);

  nh_.getParam(ns + "/publish_world_to_odom", params_.publish_world_to_odom);
  nh_.getParam(ns + "/world_frame", params_.world_frame);
  nh_.getParam(ns + "/tf_publication_rate_hz", params_.tf_publication_rate_hz);

  nh_.getParam(ns + "/clear_local_map_after_loop_closure",
               params_.clear_local_map_after_loop_closure);

  nh_.getParam(ns + "/mimic_pub_rate_hz", mimic_pub_rate_hz);

  v_slam_params_ = v_slam::getLaserSlamWorkerParams(nh_, ns);

  // Benchmarker parameters.
  benchmarker_params_ = laser_slam_ros::getBenchmarkerParams(nh_, ns);

  // ICP configuration files.
  nh_.getParam("icp_configuration_file",
               params_.online_estimator_params.laser_track_params.icp_configuration_file);
  nh_.getParam("icp_input_filters_file",
               params_.online_estimator_params.laser_track_params.icp_input_filters_file);

  // SegMatchWorker parameters.
  segmatch_worker_params_ = segmatch_ros::getSegMatchWorkerParams(nh_, ns);
  segmatch_worker_params_.world_frame = params_.world_frame;
}
