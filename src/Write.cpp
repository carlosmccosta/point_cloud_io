/*
 * Write.cpp
 *
 *  Created on: Nov 13, 2015
 *      Author: Remo Diethelm
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "point_cloud_io/Write.hpp"

//PCL
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace ros;
using namespace pcl;
using namespace pcl::io;

namespace point_cloud_io {

Write::Write(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      filePrefix_("point_cloud"),
      fileEnding_("ply"),
      tf2TransformListener_(tf2Buffer_)
{
  if (!readParameters()) ros::requestShutdown();
  pointCloudSubscriber_ = nodeHandle_.subscribe(pointCloudTopic_, 1, &Write::pointCloudCallback, this);
  ROS_INFO_STREAM("Subscribed to topic \"" << pointCloudTopic_ << "\".");
}

Write::~Write()
{

}

bool Write::readParameters()
{
  bool allParametersRead = true;
  if (!nodeHandle_.getParam("topic", pointCloudTopic_)) allParametersRead = false;
  if (!nodeHandle_.getParam("folder_path", folderPath_)) allParametersRead = false;
  nodeHandle_.getParam("file_prefix", filePrefix_);
  nodeHandle_.getParam("file_ending", fileEnding_);
  nodeHandle_.getParam("add_counter_to_path", addCounterToPath_);
  nodeHandle_.getParam("add_frame_id_to_path", addFrameIdToPath_);
  nodeHandle_.getParam("add_stamp_sec_to_path", addStampSecToPath_);
  nodeHandle_.getParam("add_stamp_nsec_to_path", addStampNSecToPath_);
  nodeHandle_.getParam("save_only_one_pointcloud", saveOnlyOnePointcloud_);
  nodeHandle_.getParam("save_normals", saveNormals_);
  nodeHandle_.getParam("tf_target_frame_id", tfTargetFrameId_);
  nodeHandle_.getParam("tf_override_point_cloud_frame_id", tfOverridePointCloudFrameId_);
  nodeHandle_.getParam("tf_lookup_timeout", tfLookupTimeout_);

  if (tfLookupTimeout_ <= 0.0)
    tfLookupTimeout_ = 3.0;

  if (!allParametersRead)
  {
    ROS_WARN("Could not read all parameters. Typical command-line usage:\n"
        "rosrun point_cloud_io write"
        " _topic:=/my_topic"
        " _folder_path:=/home/user/my_point_clouds"
        " (optional: _file_prefix:=my_prefix"
                   " _file_ending:=my_ending"
                   " _add_counter_to_path:=true/false"
                   " _add_frame_id_to_path:=true/false"
                   " _add_stamp_sec_to_path:=true/false"
                   " _add_stamp_nsec_to_path:=true/false"
                   " _save_only_one_pointcloud:=true/false"
                   " _save_normals:=true/false"
                   " _tf_target_frame_id:=frame_id)"
                   " _tf_override_point_cloud_frame_id:=frame_id"
                   " _tf_lookup_timeout:=timeout");
    return false;
  }

  return true;
}

void Write::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  ROS_INFO_STREAM("Received point cloud with " << cloud->height*cloud->width << " points.");
  std::cout << folderPath_ << std::endl;
  stringstream filePath;
  filePath << folderPath_ << "/";
  if (!filePrefix_.empty()) {
    filePath << filePrefix_;
  }
  if (addCounterToPath_) {
    filePath << "_" << counter_;
    counter_++;
  }
  if (addFrameIdToPath_) {
    filePath << "_" << cloud->header.frame_id;
  }
  if (addStampSecToPath_) {
    filePath << "_" << cloud->header.stamp.sec;
  }
  if (addStampNSecToPath_) {
    filePath << "_" << cloud->header.stamp.nsec;
  }
  filePath << ".";
  filePath << fileEnding_;

  std::string filePathStr = filePath.str();

  if (fileEnding_ == "ply") {
    // Write .ply file.

    bool transformPointcloud = false;
    geometry_msgs::TransformStamped tf;
    Eigen::Quaterniond tfRotation;
    Eigen::Vector3d tfOrigin;

    std::string sourceFrameId = tfOverridePointCloudFrameId_.empty() ? cloud->header.frame_id : tfOverridePointCloudFrameId_;

    if (!tfTargetFrameId_.empty() && tfTargetFrameId_ != sourceFrameId)
    {
      try
      {
        tf = tf2Buffer_.lookupTransform(tfTargetFrameId_, sourceFrameId, cloud->header.stamp, ros::Duration(tfLookupTimeout_));
        transformPointcloud = true;
      }
      catch (...)
      {
        ROS_WARN_STREAM("Missing TF [" << sourceFrameId << " -> " << tfTargetFrameId_ << "]\nTrying with current time...");
        try
        {
          tf = tf2Buffer_.lookupTransform(tfTargetFrameId_, sourceFrameId, ros::Time(0), ros::Duration(tfLookupTimeout_));
          transformPointcloud = true;
        }
        catch (...)
        {
          ROS_ERROR("Failed to transform point cloud!");
        }
      }

      if (transformPointcloud)
      {
        tfRotation = Eigen::Quaterniond(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);
        tfOrigin = Eigen::Vector3d(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
      }
    }

    bool saved = false;
    if (saveNormals_)
    {
      PointCloud<PointXYZRGBNormal> pclCloudWithNormals;
      fromROSMsg(*cloud, pclCloudWithNormals);

      if (transformPointcloud)
        pcl::transformPointCloudWithNormals(pclCloudWithNormals, pclCloudWithNormals, tfOrigin, tfRotation);

      saved = savePointCloud(filePathStr, pclCloudWithNormals);
    }
    else
    {
      PointCloud<PointXYZRGB> pclCloud;

      if (transformPointcloud)
        pcl::transformPointCloud(pclCloud, pclCloud, tfOrigin, tfRotation);

      fromROSMsg(*cloud, pclCloud);
      saved = savePointCloud(filePathStr, pclCloud);
    }

    if (!saved)
    {
      ROS_ERROR("Something went wrong when trying to write the point cloud file.");
      return;
    }
  }
  else {
    ROS_ERROR_STREAM("Data format not supported.");
    return;
  }

  ROS_INFO_STREAM("Saved point cloud (" << (saveNormals_ ? "PointXYZRGBNormal" : "PointXYZRGB") << ") to " << filePathStr << ".");

  if (saveOnlyOnePointcloud_)
    ros::shutdown();
}

template<typename PointT>
bool Write::savePointCloud(const std::string& filePath, const PointCloud<PointT>& pclCloud)
{
  PLYWriter writer;
  return (writer.write(filePath, pclCloud, true) == 0);
}

} /* namespace */
