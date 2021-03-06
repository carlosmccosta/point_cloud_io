/*
 * Read.cpp
 *
 *  Created on: Aug 7, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "point_cloud_io/Read.hpp"

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
// define the following in order to eliminate the deprecated headers warning
#define VTK_EXCLUDE_STRSTREAM_HEADERS
#include <pcl/io/vtk_lib_io.h>

using namespace std;
using namespace ros;
using namespace pcl;
using namespace pcl::io;
using namespace pcl_conversions;

namespace point_cloud_io {

Read::Read(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      pointCloudMessage_(new sensor_msgs::PointCloud2)
{
  if (!readParameters()) ros::requestShutdown();
  pointCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(pointCloudTopic_, 1, true);
  initialize();
}

Read::~Read()
{

}

bool Read::readParameters()
{
  bool allParametersRead = true;
  if (!nodeHandle_.getParam("file_path", filePath_)) allParametersRead = false;
  if (!nodeHandle_.getParam("topic", pointCloudTopic_)) allParametersRead = false;
  if (!nodeHandle_.getParam("frame", pointCloudFrameId_)) allParametersRead = false;

  nodeHandle_.param("load_only_valid_points", loadOnlyValidPoints_, true);
  nodeHandle_.param("remove_points_in_sensor_origin", removePointsInSensorOrigin_, removePointsInSensorOrigin_);

  double updateRate;
  nodeHandle_.param("rate", updateRate, 0.0);
  if (updateRate == 0.0)
  {
    isContinousPublishing_ = false;
  }
  else
  {
    isContinousPublishing_ = true;
    updateDuration_.fromSec(1.0 / updateRate);
  }

  nodeHandle_.param("shutdown_delay_if_not_continous_publishing", shutdownDelayIfNotContinousPublishing_, -1.0);
  nodeHandle_.param("start_publishing_delay", startPublishingDelay_, 1.0);

  if (!nodeHandle_.getParam("pointcloud_scale_factor", pointCloudScaleFactor_) || pointCloudScaleFactor_ <= 0.0)
    pointCloudScaleFactor_ = 1.0;

  if (!allParametersRead)
  {
    ROS_WARN("Could not read all parameters. Typical command-line usage:\n"
        "rosrun point_cloud_io read"
        " _file_path:=/home/user/my_point_cloud.ply"
        " _topic:=/my_topic"
        " _frame:=sensor_frame"
        " (optional: _load_only_valid_points:=true/false"
                   " _remove_points_in_sensor_origin:=true/false"
                   " _rate:=publishing_rate"
                   " _start_publishing_delay:=delay"
                   " _shutdown_delay_if_not_continous_publishing:=delay"
                   " _pointcloud_scale_factor:=scale)");
    return false;
  }

  return true;
}

void Read::initialize()
{
  if (!readFile(filePath_, pointCloudFrameId_)) ros::requestShutdown();

  if (startPublishingDelay_ > 0.0)
    ros::Duration(startPublishingDelay_).sleep();

  if (isContinousPublishing_)
  {
    timer_ = nodeHandle_.createTimer(updateDuration_, &Read::timerCallback, this);
  }
  else
  {
    Duration(1.0).sleep(); // Need this to get things ready before publishing.
    if (!publish()) ROS_ERROR("Something went wrong when trying to read and publish the point cloud file.");
    
    if (shutdownDelayIfNotContinousPublishing_ > 0.0)
    {
      Duration(shutdownDelayIfNotContinousPublishing_).sleep(); // Give time for the subscribers to receive the point cloud.
      ros::requestShutdown();
    }
  }
}

bool Read::readFile(const std::string& filePath, const std::string& pointCloudFrameId)
{
  PointCloud<PointXYZRGBNormal> pointCloud;

  if (filePath.find(".ply") != std::string::npos) {
    // Load .ply file.
    if (loadPLYFile(filePath, pointCloud) != 0) return false;
  }
  else if (filePath.find(".vtk") != std::string::npos) {
    // Load .vtk file.
    PolygonMesh polygonMesh;
    loadPolygonFileVTK(filePath, polygonMesh);
    pcl::fromPCLPointCloud2(polygonMesh.cloud, pointCloud);
  }
  else {
    ROS_ERROR_STREAM("Data format not supported.");
    return false;
  }

  if (pointCloudScaleFactor_ != 1.0)
  {
    Eigen::Matrix4f scale_matrix = Eigen::Matrix4f(Eigen::Matrix4f::Identity()) * pointCloudScaleFactor_;
    pcl::transformPointCloudWithNormals<PointXYZRGBNormal>(pointCloud, pointCloud, scale_matrix);
  }

  if (loadOnlyValidPoints_) {
    Write::removeNaNs(pointCloud);
  }

  if (removePointsInSensorOrigin_) {
    Write::removePointsInSensorOrigin(pointCloud);
  }
  
  toROSMsg(pointCloud, *pointCloudMessage_);

  pointCloudMessage_->header.frame_id = pointCloudFrameId;

  ROS_INFO_STREAM("Loaded point cloud with " << pointCloudMessage_->height*pointCloudMessage_->width << " points.");
  return true;
}

void Read::timerCallback(const ros::TimerEvent& timerEvent)
{
  if (!publish()) ROS_ERROR("Something went wrong when trying to read and publish the point cloud file.");
}

bool Read::publish()
{
  pointCloudMessage_->header.stamp = Time::now();
  pointCloudPublisher_.publish(pointCloudMessage_);
  ROS_INFO_STREAM("Point cloud published to topic \"" << pointCloudTopic_ << "\".");
  return true;
}

} /* namespace */
