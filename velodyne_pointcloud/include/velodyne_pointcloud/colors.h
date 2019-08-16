/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2012 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 *  Modified 2018 Kaarta - Shawn Hanna
 *  $Id$
 */

/** @file

    Interface for converting a Velodyne 3D LIDAR PointXYZIR cloud to
    PointXYZRGB, assigning colors for visualization of the laser
    rings.

    @author Jack O'Quin
*/

#ifndef _VELODYNE_POINTCLOUD_COLORS_H_
#define _VELODYNE_POINTCLOUD_COLORS_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <velodyne_pointcloud/point_types.h>

namespace velodyne_pointcloud
{
  // shorter names for point cloud types in this namespace
  typedef Kaarta::KaartaSensorPoint VPoint;
  typedef pcl::PointCloud<VPoint> VPointCloud;

  class RingColors
  {
  public:

    RingColors(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~RingColors() {}

  private:

    void convertPoints(const VPointCloud::ConstPtr &inMsg);

    ros::Subscriber input_;
    ros::Publisher output_;
  };

} // namespace velodyne_pointcloud

#endif // _VELODYNE_POINTCLOUD_COLORS_H_
