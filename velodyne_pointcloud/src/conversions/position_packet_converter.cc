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

#include "velodyne_pointcloud/position_packet_converter.h"

#include <pcl_conversions/pcl_conversions.h>
#include <clay_lib/stencil_constants.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  PositionPacketConverter::PositionPacketConverter(ros::NodeHandle& node, ros::NodeHandle& private_nh):
    init_success(true), private_nh_(private_nh), laser_model_(-1), pps_output_delay_(0.999)
  {
    private_nh.param<std::string>("imu_header", imu_frame_id_, "imu");
    private_nh.param<std::string>("gnss_header", gnss_frame_id_, "gnss");

    private_nh.getParam("/laser_model", laser_model_);
    private_nh.getParam("pps_output_delay_", pps_output_delay_);

    imu_data_.reset(new sensor_msgs::Imu());
    pps_data_.reset(new velodyne_msgs::VelodynePPS());
    gnss_raw_data_.reset(new stencil_msgs::GPS_NMEA_Stamped());
    gnss_fix_data_.reset(new sensor_msgs::NavSatFix());

    gnss_fix_data_->header.frame_id = gnss_frame_id_;
    imu_data_->header.frame_id=imu_frame_id_;
    gnss_raw_data_->header.frame_id = gnss_frame_id_;

    // advertise output point cloud (before subscribing to input data)
    imu_output_pub_ =
      node.advertise<sensor_msgs::Imu>("velodyne_imu", 10);
    gnss_raw_output_pub_ =
      node.advertise<stencil_msgs::GPS_NMEA_Stamped>("velodyne_gnss_raw", 10);
    gnss_fix_output_pub_ =
      node.advertise<sensor_msgs::NavSatFix>("velodyne_gnss_fix", 10);
    pps_state_pub_ =
      node.advertise<velodyne_msgs::VelodynePPS>("velodyne_pps_state", 10, true);

    // subscribe to VelodyneScan packets
    velodyne_packet_sub_ =
      node.subscribe("velodyne_position_packets", 10,
                     &PositionPacketConverter::processPacket, (PositionPacketConverter *) this,
                     ros::TransportHints().tcpNoDelay(true));

    diagnostics_.setHardwareID("Velodyne");
    diag_timer_ = private_nh.createTimer(ros::Duration(0.5), &PositionPacketConverter::diagTimerCallback,this);
    diagnostics_.add("Velodyne PPS status", this, &PositionPacketConverter::ppsStatus);
    diagnostics_.add("Velodyne NMEA status", this, &PositionPacketConverter::nmeaStatus);

    diag_min_freq_ = 10;
    diag_max_freq_ = 200;
    using namespace diagnostic_updater;
    diag_position_topic_.reset(new TopicDiagnostic("Velodyne position packets convert", diagnostics_,
                                      FrequencyStatusParam(&diag_min_freq_,
                                                          &diag_max_freq_,
                                                          0.05, 1),
                                      TimeStampStatusParam(0.0, 0.5)));
  }
  
  bool PositionPacketConverter::initSuccessful(){
    return init_success;
  }
  
  /** @brief Callback for raw scan messages. */
  void PositionPacketConverter::processPacket(const velodyne_msgs::VelodynePositionPacket::ConstPtr &pkt)
  {
    diag_position_topic_->tick(pkt->stamp);

    // if (imu_output_pub_.getNumSubscribers() == 0 &&
    //     gnss_raw_output_pub_.getNumSubscribers() == 0 &&
    //     gnss_fix_output_pub_.getNumSubscribers() == 0 &&
    //     pps_state_pub_.getNumSubscribers() == 0)
    //   return;

    bool forceUpdate = false;

    private_nh_.getParamCached("/laser_model", laser_model_);

    // ROS_INFO_STREAM("Got new position packet: " << pkt->stamp);
    // sensor_msgs::Imu::Ptr new_imu_ptr(new sensor_msgs::Imu());
    if (laser_model_ == Kaarta::StencilConstants::TYPE_HDL32 && imu_output_pub_.getNumSubscribers())
    {
      imu_data_.reset(new sensor_msgs::Imu());
      parseImuData(&pkt->data[0], *imu_data_);

      imu_data_->header.stamp = pkt->stamp;
      imu_output_pub_.publish(imu_data_);
    }

    std::string nmea_string((const char*)(&pkt->data[0] + 0xCE));

    if (nmea_string != gnss_raw_data_->sentence)
    {
      gnss_raw_data_.reset(new stencil_msgs::GPS_NMEA_Stamped());
      gnss_fix_data_.reset(new sensor_msgs::NavSatFix());

      parseNmeaString(nmea_string.c_str(), *gnss_fix_data_);
      gnss_fix_data_->header.stamp = pkt->stamp;
      gnss_fix_data_->status.status = 1;
      gnss_fix_output_pub_.publish(gnss_fix_data_);

      gnss_raw_data_->sentence = nmea_string;
      gnss_raw_data_->header.stamp = pkt->stamp;
      gnss_raw_output_pub_.publish(gnss_raw_data_);
    }

    if ( (pps_data_->state != pkt->data[0xCA] || fabs( (pkt->stamp - pps_data_->stamp).toSec() ) >= pps_output_delay_ ) )
    {
      if (pps_data_->state != pkt->data[0xCA]){
        forceUpdate = true;
      }
      pps_data_.reset(new velodyne_msgs::VelodynePPS());
      pps_data_->state = pkt->data[0xCA];
      pps_data_->stamp = pkt->stamp;
      pps_state_pub_.publish(pps_data_);
    }
    if (forceUpdate)
      diagnostics_.force_update();
  }

  void PositionPacketConverter::parseNmeaString(const char * nmea_string, sensor_msgs::NavSatFix& nav_sat_fix){
    std::string s(nmea_string);

    std::string tokens[13];
    std::string delimiter = ",";
    size_t pos;
    int i =0;
    while ((pos = s.find(delimiter)) != std::string::npos) {
      tokens[i++] = s.substr(0, pos);
      s.erase(0, pos + delimiter.length());
    }

    nav_sat_fix.altitude = 0.0;
    nav_sat_fix.position_covariance_type = 0; //COVARIANCE_TYPE_UNKNOWN;
    nav_sat_fix.status.status = 0; //STATUS_FIX;
    nav_sat_fix.status.service = 1; //SERVICE_GPS;

    //latitude;
    float lat = strtof(tokens[3].c_str(), NULL)/100;
    float int_lat = floor(lat);
    if(tokens[4]=="N"){
      nav_sat_fix.latitude = int_lat+ (lat - int_lat)/0.6;
    }else{
      nav_sat_fix.latitude = - (int_lat+ (lat - int_lat)/0.6);
    }
    
    // ROS_DEBUG_STREAM("Velodyne packet latitude:" << nav_sat_fix.latitude);

    //longitude
    float lon = strtof(tokens[5].c_str(), NULL)/100;
    float int_lon = floor(lon);
    if(tokens[6]=="E"){
      nav_sat_fix.longitude = int_lon + (lon - int_lon)/0.6;
    }else{
      nav_sat_fix.longitude = -(int_lon + (lon - int_lon)/0.6);
    }

    // ROS_DEBUG_STREAM("Velodyne packet longitude:" << nav_sat_fix.longitude);
  }

  void PositionPacketConverter::parseImuData(const uint8_t *b, sensor_msgs::Imu& imu_data)
  {
    imu_data.orientation_covariance[0] = -1;

    double gyro1, accel1_x, accel1_y, gyro2, accel2_x, accel2_y, gyro3, accel3_x, accel3_y;

    for (int i = 14; i < 38; i += 2) {
      int j = (i - 14) / 2;
      int type = j % 4;
      if (type == 1) {
        // This is temperature information, which we can ignore for now.
        continue;
      }

      // Parse these two packets, returning a signed 12-bit integer giving a raw measurement (per the datasheet).
      char hex1[3];
      char hex2[3];
      sprintf(&hex1[0], "%02x", *(b + i));
      sprintf(&hex2[0], "%02x", *(b + i + 1));
      char hexCorrected[4] = { hex2[1], hex1[0], hex1[1], 0 };
      int rawData = (int)(strtol(hexCorrected, NULL, 16));
      while (rawData > 2047) {
        rawData -= 4096;
      }

      // Scale the raw measurement based on whatever type of sensor we're reading.
      double scaledData;
      if (type == 0) {
        scaledData = PositionPacketConverter::gyroscopeScale * rawData;
      } else {
        scaledData = PositionPacketConverter::accelerometerScale * rawData;
      }

      // Store the scaled measurements so that we can add them to our sensor_msgs/Imu messsage later.
      switch(j){
        case 0:
          gyro1 = scaledData;
          break;
        case 2:
          accel1_x = scaledData;
          break;
        case 3:
          accel1_y = scaledData;
          break;
        case 4:
          gyro2 = scaledData;
          break;
        case 6:
          accel2_x = scaledData;
          break;
        case 7:
          accel2_y = scaledData;
          break;
        case 8:
          gyro3 = scaledData;
          break;
        case 10:
          accel3_x = scaledData;
          break;
        case 11:
          accel3_y = scaledData;
          break;
      }
    }

    imu_data.linear_acceleration.x = (accel1_y - accel3_x) / 2;
    imu_data.linear_acceleration.y = (accel2_y + accel3_y) / 2;
    imu_data.linear_acceleration.z = (accel1_x + accel2_x) / 2;

    imu_data.angular_velocity.x = gyro2;
    imu_data.angular_velocity.y = -gyro1;
    imu_data.angular_velocity.z = gyro3;

    // imu_data.header.stamp = parseInternalTime(*(b + 198), *(b + 199), *(b + 200), *(b + 201));
    imu_data.header.stamp = ros::Time::now();
    imu_data.header.frame_id = "velodyne";
  }

  void PositionPacketConverter::diagTimerCallback(const ros::TimerEvent&event)
  {
    diagnostics_.update();
  }

  void PositionPacketConverter::nmeaStatus(diagnostic_updater::DiagnosticStatusWrapper &stat){
    stat.add("nmea string", gnss_raw_data_->sentence);
    if (gnss_raw_data_->sentence.length() > 20)
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "NMEA string length ok");
    else
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "NMEA string error");
  }

  void PositionPacketConverter::imuStatus(diagnostic_updater::DiagnosticStatusWrapper &stat){
    stat.add("Laser Model", laser_model_);
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Not checking IMU status");
  }

  void PositionPacketConverter::ppsStatus(diagnostic_updater::DiagnosticStatusWrapper &stat){
    switch(pps_data_->state){
      case velodyne_msgs::VelodynePPS::STATE_NO_PPS_DETECTED:
        stat.add("PPS Status", "No PPS detected");
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "No PPS detected");
        ROS_WARN_THROTTLE(0.5, "No PPS Detected");
        break;
      case velodyne_msgs::VelodynePPS::STATE_SYNCHRONIZING_TO_PPS:
        stat.add("PPS Status", "Synchronizing to PPS");
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Synchronizing to PPS");
        ROS_WARN_THROTTLE(0.5, "Synchronizing PPS");
        break;
      case velodyne_msgs::VelodynePPS::STATE_PPS_LOCKED:
        stat.add("PPS Status", "PPS Locked");
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "PPS Locked");
        break;
      case velodyne_msgs::VelodynePPS::STATE_ERROR:
        stat.add("PPS Status", "Error");
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "PPS Error");
        ROS_WARN_THROTTLE(0.5, "PPS Error");
        break;
      default:
        stat.add("PPS Status", pps_data_->state);
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Unknown PPS status");
        ROS_WARN_THROTTLE(0.5, "Unknown PPS Status");
    };
  }
} // namespace velodyne_pointcloud
