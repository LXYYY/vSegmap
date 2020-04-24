#include "segmapper/v_slam_worker.h"

#include <Eigen/Dense>
#include <glog/logging.h>
#include <pcl_conversions/pcl_conversions.h>

namespace v_slam {
VSlamWorker::VSlamWorker()
{
}

void VSlamWorker::init(ros::NodeHandle& nh, const VSlamWorkerParams& params, unsigned int worker_id)
{
  params_ = params;
  worker_id_ = worker_id;

  LOG(INFO)<<"VSlamWorker inited"<<std::endl;
  LOG(INFO)<<"cloud topic: "<<params_.assembled_cloud_sub_topic<<std::endl;
  LOG(INFO)<<"pose_topic: "<<params_.assembled_pose_sub_topic<<std::endl;

  // Setup subscriber.
  scan_sub_ = nh.subscribe(params_.assembled_cloud_sub_topic, kScanSubscriberMessageQueueSize,
                           &VSlamWorker::scanCallback, this);

  pose_sub_ = nh.subscribe(params_.assembled_pose_sub_topic, kScanSubscriberMessageQueueSize,
                           &VSlamWorker::poseCallback, this);
}

void VSlamWorker::scanCallback(const sensor_msgs::PointCloud2& cloud_msg_in)
{
  // Add the local scans to the full point cloud.
  pcl::fromROSMsg(cloud_msg_in, scan_);
  if (scan_.size() > 0u) {
    std::lock_guard<std::recursive_mutex> lock(local_map_mutex_);
    if (local_map_stamped_.find(cloud_msg_in.header.stamp) == local_map_stamped_.end()) {
      local_map_stamped_.insert(
          LocalMapStamped::value_type(
              cloud_msg_in.header.stamp,
              LocalMap(laser_slam::Pose(), std::vector<laser_slam_ros::PointCloud>(1, scan_))));
    } else {
      local_map_stamped_[cloud_msg_in.header.stamp].second =
          std::vector<laser_slam_ros::PointCloud>(1, scan_);
      local_map_queue_.push(local_map_stamped_[cloud_msg_in.header.stamp]);
      local_map_stamped_.erase(cloud_msg_in.header.stamp);
    }
  }
}

void VSlamWorker::poseCallback(const geometry_msgs::PoseStamped& pose_msg_in)
{
  std::lock_guard<std::recursive_mutex> lock(local_map_mutex_);
  pose_.time_ns = pose_msg_in.header.stamp.toNSec();
  pose_.T_w = laser_slam::SE3(
      Eigen::Quaternion<double>(pose_msg_in.pose.orientation.w, pose_msg_in.pose.orientation.x,
                                pose_msg_in.pose.orientation.y, pose_msg_in.pose.orientation.z),
      Eigen::Vector3d(pose_msg_in.pose.position.x, pose_msg_in.pose.position.y,
                      pose_msg_in.pose.position.z));

  if (local_map_stamped_.find(pose_msg_in.header.stamp) == local_map_stamped_.end()) {
    local_map_stamped_.insert(
        LocalMapStamped::value_type(
            pose_msg_in.header.stamp,
            LocalMap(pose_,
                     std::vector<laser_slam_ros::PointCloud>(1, laser_slam_ros::PointCloud()))));
  } else {
    local_map_stamped_[pose_msg_in.header.stamp].first = pose_;
    local_map_queue_.push(local_map_stamped_[pose_msg_in.header.stamp]);
    local_map_stamped_.erase(pose_msg_in.header.stamp);
  }
}

bool VSlamWorker::getQueuedScanAndPose(LocalMap& local_map)
{
  std::lock_guard<std::recursive_mutex> lock(local_map_mutex_);
  if (local_map_queue_.empty())
    return false;
  local_map = local_map_queue_.front();
  local_map_queue_.pop();
  return true;
}
}
