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

#include <std_msgs/Float32.h>

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
    init_success(true),
    last_rpm_(-1),
    last_rpm_raw_(-1)
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

    rpm_pub_ =
      node.advertise<std_msgs::Float32>("velodyne_rpm", 10);
      
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
    diag_min_freq_ = 5;
    diag_max_freq_ = 20;
    ROS_INFO("expected frequencies: %.3lf (Hz) to %.3lf", diag_min_freq_, diag_max_freq_);
    diag_topic_.reset(new TopicDiagnostic("velodyne pointcloud convert", diagnostics_,
                                          FrequencyStatusParam(&diag_min_freq_,
                                                              &diag_max_freq_,
                                                              0.1, 5),
                                          TimeStampStatusParam(-1, 1)));
    diag_timer_ = private_nh.createTimer(ros::Duration(0.2), &Convert::diagTimerCallback,this);
    diagnostics_.add("Velodyne Timestamp Status", this, &Convert::timestampDiagnostic);
    diagnostics_.add("Velodyne RPM", this, &Convert::rpmDiagnostic);
    diagnostics_.add("Velodyne Cloud Point Size", this, &Convert::pointCountDiagnostic);

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

  void Convert::calculateRPM(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    static const std::size_t azimuth_data_pos = 100*0+2;
    // calculate RPM
    if (scanMsg->packets.size() > 2){
      int last_azimuth = *( (u_int16_t*) (&(scanMsg->packets.back().data[azimuth_data_pos])));
      int first_azimuth = *( (u_int16_t*) (&scanMsg->packets.front().data[azimuth_data_pos]));

      // figure out how many revolutions we've gone through in this packet. we can't assume we guessed correctly, so 
      int num_revolutions = 0;
      int prev_azimuth, azimuth;
      prev_azimuth = azimuth = first_azimuth;
      for (unsigned int i=0; i < scanMsg->packets.size(); ++i){
        azimuth = *( (u_int16_t*) (&scanMsg->packets[i].data[azimuth_data_pos]));
        if (azimuth < prev_azimuth){
          ++num_revolutions;
        }
        prev_azimuth = azimuth;
      }

      // convert delta angle to delta revolutions and then divide by delta time
      float rpm = ( ( (36000*num_revolutions + last_azimuth - first_azimuth) / 100.) / (360) )/((scanMsg->packets.back().stamp - scanMsg->packets.front().stamp).toSec());
      rpm = rpm * 60; // convert rev/sec to rev/min
      last_rpm_raw_ = rpm;
      float rpm_rounded = ( ((int)(rpm+30)) / 60) * 60;
      last_rpm_ = rpm_rounded;
      ROS_DEBUG_THROTTLE(10, "number packets: %d, first azimuth: %f, last_azimuth: %f, num revolutions: %d Detected RPM: %f - rounded to multiple of 60: %f",
                            (int) scanMsg->packets.size(), first_azimuth/100.0f, last_azimuth/100.0f, num_revolutions, rpm, rpm_rounded);

      std_msgs::Float32 out;
      out.data = rpm;
      rpm_pub_.publish(out);
    }
    else
    {
      last_rpm_ = -1;
      last_rpm_raw_ = -1;
      ROS_WARN_THROTTLE(1, "RPM calculation can't continue. Not enough packets %d", (int) scanMsg->packets.size());
    }
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

    calculateRPM(scanMsg);

    diag_topic_->tick(scanMsg->header.stamp);
    diagnostics_.update();
  }
  void Convert::rpmDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){
    stat.add("Latest RPM (rounded)", last_rpm_);
    stat.add("Latest RPM raw", last_rpm_raw_);
    stat.add("Minimum acceptable RPM", 300);
    stat.add("Maximum acceptable RPM", 1200);
    if (last_rpm_ == 600)
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "RPM ok: %f - raw: %f", last_rpm_, last_rpm_raw_);
    else if (last_rpm_ < 300 || last_rpm_ > 1200)
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "RPM not within Velodyne bounds (300-1200 rpm). Got: %f - raw: %f", last_rpm_, last_rpm_raw_);
    else
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "RPM ok but not 600 which we prefer. Detected rounded: %f - raw: %f", last_rpm_, last_rpm_raw_);
  }

  void Convert::timestampDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){
    auto last_time = pcl_conversions::fromPCL(outMsg.pc->header.stamp);
    auto now = ros::Time::now();
    auto diff = now - last_time;
    stat.add("Latest Velodyne Stamp", last_time);
    stat.add("Time of System Clock", now);
    stat.add("Time diff velodyne to system clock", diff);
    stat.add("Number of points in output", outMsg.pc->points.size());

    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Velodyne Cloud OK");
    if (fabs(diff.toSec()) > 1)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Velodyne Cloud time is very different from system clock");
    }
    else if (fabs(diff.toSec()) > 0.3)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Velodyne Cloud time is different from system clock");
    }
  }

  void Convert::pointCountDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    stat.add("Number of Points", outMsg.pc->points.size());
    if (outMsg.pc->points.size() < 10000)
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Velodyne Cloud has only a few points output");
    else
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Velodyne Cloud output size ok");
  }

  void Convert::diagTimerCallback(const ros::TimerEvent &event)
  {
    (void)event;
    // Call necessary to provide an error when no velodyne packets are received
    diagnostics_.update();
  }
} // namespace velodyne_pointcloud
