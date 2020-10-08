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

#include "velodyne_pointcloud/convert.h"

#include <pcl_conversions/pcl_conversions.h>

#include <clay_lib/time_profiler.h>

inline velodyne_rawdata::VPoint convertRawPointToXYZ(const velodyne_rawdata::VPointRaw& point, const velodyne_pointcloud::LaserCorrection& param)
{
  velodyne_rawdata::VPoint vpoint;

  auto dist_part = (param.distance_scale_m * point.distance + param.dist_correction) * param.cos_vert_correction;
  auto sin_point_azi = sin(point.azimuth);
  auto cos_point_azi = cos(point.azimuth);
  auto cos_horizontal_rot_correction = param.cos_rot_correction;
  auto sin_horizontal_rot_correction = param.sin_rot_correction;

  vpoint.x = dist_part * 
            ( sin_point_azi*cos_horizontal_rot_correction - cos_point_azi*sin_horizontal_rot_correction )
            - param.horiz_offset_correction * (cos_point_azi*cos_horizontal_rot_correction + sin_point_azi*sin_horizontal_rot_correction);

  vpoint.y = dist_part * 
            ( cos_point_azi*cos_horizontal_rot_correction + sin_point_azi*sin_horizontal_rot_correction )
            + param.horiz_offset_correction * (sin_point_azi*cos_horizontal_rot_correction - cos_point_azi*sin_horizontal_rot_correction);

  vpoint.z = (param.distance_scale_m * point.distance + param.dist_correction) * param.sin_vert_correction + param.vert_offset_correction;
  
  auto x_coord = vpoint.y;
  auto y_coord = -vpoint.x;
  auto z_coord = vpoint.z;

  vpoint.x = x_coord;
  vpoint.y = y_coord;
  vpoint.z = z_coord;

  vpoint.azimuth = point.azimuth;
  vpoint.intensity = point.intensity;
  vpoint.time = point.time;
  vpoint.distance = param.distance_scale_m * point.distance + param.dist_correction;
  vpoint.ring = point.ring;
  return vpoint;
}

namespace velodyne_pointcloud
{
  using namespace diagnostic_updater;
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle &node, ros::NodeHandle &private_nh):
    data_(new velodyne_rawdata::RawData()),
    init_success(true)
  {
    rawCloud.reset(new velodyne_rawdata::VPointCloudRaw());

    int result = data_->setup(private_nh);

    if (result < 0){
      init_success = false;
      return;
    }


    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);
      
    srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_pointcloud::
      CloudNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig>::
      CallbackType f;
    f = boost::bind (&Convert::callback, this, _1, _2);
    srv_->setCallback (f);

    // subscribe to VelodyneScan packets
    velodyne_scan_ =
      node.subscribe("velodyne_packets", 200,
                     &Convert::processScan, (Convert *) this,
                     ros::TransportHints().tcpNoDelay(true));

    diagnostics_.setHardwareID("Velodyne");
    diag_min_freq_ = diag_max_freq_ = 10;
    diag_topic_.reset(new TopicDiagnostic("velodyne_pointcloud", diagnostics_,
                                          FrequencyStatusParam(&diag_min_freq_,
                                                              &diag_max_freq_,
                                                              0.1, 5),
                                          TimeStampStatusParam(-1, 1)));
    diag_timer_ = private_nh.createTimer(ros::Duration(0.2), &Convert::diagTimerCallback,this);
  }
  
  bool Convert::initSuccessful(){
    return init_success;
  }
  
  void Convert::callback(velodyne_pointcloud::CloudNodeConfig &config,
                uint32_t level)
  {
  ROS_INFO("Reconfigure Request");
  data_->setParameters(config.min_range, config.max_range, config.view_direction,
                       config.view_width);
  }

  /** @brief Callback for raw scan messages. */
  void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if (output_.getNumSubscribers() == 0)         // no one listening?
      return;                                     // avoid much work

    TIME_THIS_SCOPE(velo_raw_data_process_scan);

    outMsg.pc->points.clear();
    // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
    outMsg.pc->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    outMsg.pc->header.frame_id = scanMsg->header.frame_id;
    outMsg.pc->height = 1;

    rawCloud->points.clear();
    // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
    rawCloud->header = outMsg.pc->header;

    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    {
      data_->unpackRAW(scanMsg->packets[i], rawCloud, scanMsg->header.stamp);
    }

    auto laser_corrections = data_->getCalibrations().laser_corrections;

    outMsg.pc->points.resize(rawCloud->points.size());

    int i=0;
    for (auto pt : rawCloud->points)
    {
      outMsg.pc->points[i] = convertRawPointToXYZ(pt, laser_corrections[pt.laser_num]);
      ++i;
    }
    outMsg.finalize();

    // publish the accumulated cloud message
    ROS_DEBUG_STREAM("Publishing " << outMsg.pc->height * outMsg.pc->width
                     << " Velodyne points, time: " << outMsg.pc->header.stamp);
    output_.publish(outMsg.pc);
    diag_topic_->tick(scanMsg->header.stamp);
    diagnostics_.update();
  }

  void Convert::diagTimerCallback(const ros::TimerEvent &event)
  {
    (void)event;
    // Call necessary to provide an error when no velodyne packets are received
    diagnostics_.update();
  }
} // namespace velodyne_pointcloud
