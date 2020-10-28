/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Modified 2018 Kaarta - Shawn Hanna
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#ifndef _VELODYNE_POINTCLOUD_CONVERT_H_
#define _VELODYNE_POINTCLOUD_CONVERT_H_ 1

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/pointcloudXYZIADRT.h>

#include <dynamic_reconfigure/server.h>
#include <velodyne_pointcloud/CloudNodeConfig.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

namespace velodyne_pointcloud
{
  class Convert
  {
  public:

    Convert(ros::NodeHandle &node, ros::NodeHandle &private_nh);
    ~Convert() {}

    bool initSuccessful();

  private:
    
    void callback(velodyne_pointcloud::CloudNodeConfig &config,
                uint32_t level);
    void processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg);
    void calculateRPM(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg);

    ///Pointer to dynamic reconfigure service srv_
    boost::shared_ptr<dynamic_reconfigure::Server<velodyne_pointcloud::
      CloudNodeConfig> > srv_;
    
    boost::shared_ptr<velodyne_rawdata::RawData> data_;
    ros::Subscriber velodyne_scan_;
    ros::Publisher output_;
    ros::Publisher rpm_pub_;

    /** diagnostics updater */
    ros::Timer diag_timer_;
    diagnostic_updater::Updater diagnostics_;
    double diag_min_freq_;
    double diag_max_freq_;
    boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
    void diagTimerCallback(const ros::TimerEvent &event);
    void timestampDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void rpmDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void pointCountDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);

    float last_rpm_, last_rpm_raw_;
    
    // A point cloud with same time and frame ID as raw data
    PointcloudXYZIADRT outMsg;

    velodyne_rawdata::VPointCloudRaw::Ptr rawCloud;
    
    bool init_success;
  };

} // namespace velodyne_pointcloud

#endif // _VELODYNE_POINTCLOUD_CONVERT_H_
