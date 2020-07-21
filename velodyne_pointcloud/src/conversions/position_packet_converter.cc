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
    init_success(true), private_nh_(private_nh), laser_model_(-1)
  {
    private_nh.param<std::string>("imu_header", imu_frame_id_, "imu");
    private_nh.param<std::string>("gnss_header", gnss_frame_id_, "gnss");
    private_nh.getParamCached("/laser_model", laser_model_);

    imu_data_.reset(new sensor_msgs::Imu());
    gnss_raw_data_.reset(new stencil_msgs::GPS_NMEA_Stamped());

    imu_data_->header.frame_id=imu_frame_id_;
    gnss_raw_data_->header.frame_id = gnss_frame_id_;

    // advertise output point cloud (before subscribing to input data)
    imu_output_pub_ =
      node.advertise<sensor_msgs::Imu>("velodyne_imu", 10);
    gnss_raw_output_pub_ =
      node.advertise<stencil_msgs::GPS_NMEA_Stamped>("velodyne_gnss_raw", 10);

    // subscribe to VelodyneScan packets
    velodyne_packet_sub_ =
      node.subscribe("velodyne_position_packets", 10,
                     &PositionPacketConverter::processPacket, (PositionPacketConverter *) this,
                     ros::TransportHints().tcpNoDelay(true));
  }
  
  bool PositionPacketConverter::initSuccessful(){
    return init_success;
  }
  
  /** @brief Callback for raw scan messages. */
  void PositionPacketConverter::processPacket(const velodyne_msgs::VelodynePositionPacket::ConstPtr &pkt)
  {
    if (imu_output_pub_.getNumSubscribers() == 0 && gnss_raw_output_pub_.getNumSubscribers() == 0)
      return;

    private_nh_.getParamCached("/laser_model", laser_model_);

    // ROS_INFO_STREAM("Got new position packet: " << pkt->stamp);
    // sensor_msgs::Imu::Ptr new_imu_ptr(new sensor_msgs::Imu());
    if (laser_model_ == Kaarta::StencilConstants::TYPE_HDL32)
    {
      parseImuData(&pkt->data[0], *imu_data_);

      imu_data_->header.stamp = pkt->stamp;
      imu_output_pub_.publish(imu_data_);
    }

    std::string nmea_string((const char*)(&pkt->data[0] + 0xCE));

    if (nmea_string != gnss_raw_data_->sentence)
    {
      gnss_raw_data_->sentence = nmea_string;
      gnss_raw_data_->header.stamp = pkt->stamp;
      gnss_raw_output_pub_.publish(gnss_raw_data_);
    }
  }

  // void PositionPacketConverter::parseNmeaString(const char * nmea_string){
  //   std::string s(nmea_string);
  //   if (s.compare(last_nmea_sentence_) == 0){
  //     // identical packet. don't rebroadcast
  //     // ROS_DEBUG("Identical NMEA string received: %s", nmea_string);
  //     return;
  //   }
  //   last_nmea_sentence_ = s;

  //   std::string tokens[13];
  //   std::string delimiter = ",";
  //   size_t pos;
  //   int i =0;
  //   while ((pos = s.find(delimiter)) != std::string::npos) {
  //     tokens[i++] = s.substr(0, pos);
  //     s.erase(0, pos + delimiter.length());
  //   }
  //   sensor_msgs::NavSatFix gpsData;

  //   gpsData.header.stamp = ros::Time::now();
  //   gpsData.header.frame_id="gps";
  //   gpsData.altitude = 0.0;
  //   gpsData.position_covariance_type = 0; //COVARIANCE_TYPE_UNKNOWN;
  //   gpsData.status.status = 0; //STATUS_FIX;
  //   gpsData.status.service = 1; //SERVICE_GPS;

  //   //latitude;
  //   float lat = strtof(tokens[3].c_str(), NULL)/100;
  //   float int_lat = floor(lat);
  //   if(tokens[4]=="N"){
  //     gpsData.latitude = int_lat+ (lat - int_lat)/0.6;
  //   }else{
  //     gpsData.latitude = - (int_lat+ (lat - int_lat)/0.6);
  //   }
  //   ROS_DEBUG_STREAM("Velodyne packet latitude:" << gpsData.latitude);

  //   //longitude
  //   float lon = strtof(tokens[5].c_str(), NULL)/100;
  //   float int_lon = floor(lon);
  //   if(tokens[6]=="E"){
  //     gpsData.longitude = int_lon + (lon - int_lon)/0.6;
  //   }else{
  //     gpsData.longitude = -(int_lon + (lon - int_lon)/0.6);
  //   }
  //   ROS_DEBUG_STREAM("Velodyne packet longitude:" << gpsData.longitude);

  //   last_gps_data_ = gpsData;
  //   new_gps_packet_ = true;
  // }

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
} // namespace velodyne_pointcloud
