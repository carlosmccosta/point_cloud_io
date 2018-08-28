/*
 * Write.hpp
 *
 *  Created on: Nov 13, 2015
 *      Author: Remo Diethelm
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <string>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace point_cloud_io {

class Write
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  Write(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~Write();

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Point cloud callback function
   * @param cloud point cloud message.
   */
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);

  /*!
   * Method for saving a point cloud to file
   * @param filePath Path where the file will be saved
   * @param pclCloud Point cloud to save
   */
  template<typename PointT>
  bool savePointCloud(const std::string& filePath, const pcl::PointCloud<PointT>& pclCloud);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! Point cloud subscriber.
  ros::Subscriber pointCloudSubscriber_;

  //! Point cloud topic to subscribe to.
  std::string pointCloudTopic_;

  //! Path to the point cloud folder.
  std::string folderPath_;

  //! Point cloud file prefix.
  std::string filePrefix_;

  //! Point cloud file ending.
  std::string fileEnding_;

  //! Point cloud counter.
  unsigned int counter_ = 0;

  //! Settings for generating file name.
  bool addCounterToPath_ = true;
  bool addFrameIdToPath_ = false;
  bool addStampSecToPath_ = false;
  bool addStampNSecToPath_ = false;

  //! Setting for saving a single point cloud and exiting
  bool saveOnlyOnePointcloud_ = true;

  //! Setting for specifying if saving normals is required
  bool saveNormals_ = true;
};

} /* namespace */
