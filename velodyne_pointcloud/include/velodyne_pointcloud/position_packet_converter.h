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

    This class PositionPacketConverters velodyne position packets to IMU and GNSS messages

*/

#pragma once

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <stencil_msgs/GPS_NMEA_Stamped.h>

#include <velodyne_msgs/VelodynePositionPacket.h>

namespace velodyne_pointcloud
{
  class PositionPacketConverter
  {
  public:

    PositionPacketConverter(ros::NodeHandle& node, ros::NodeHandle& private_nh);
    ~PositionPacketConverter() {}

    bool initSuccessful();

  private:
    
    void processPacket(const velodyne_msgs::VelodynePositionPacket::ConstPtr &pkt);

    /** @brief Parse a position packet from the Velodyne.
     *
     *  @details This overwrites the old data in imuData, so imuMutex should be locked/unlocked before/after calling this.
     *
     *  @param b A pointer to the first byte of the position packet, not including the message header.
     */
    void parseImuData(const uint8_t *b, sensor_msgs::Imu& imu_data);
    
    void parseNmeaString(const char * nmea_string, sensor_msgs::NavSatFix& nav_sat_fix);

    ros::Subscriber velodyne_packet_sub_;
    ros::Publisher  imu_output_pub_, gnss_raw_output_pub_, gnss_fix_output_pub_;

    sensor_msgs::Imu::Ptr imu_data_;
    stencil_msgs::GPS_NMEA_Stamped::Ptr gnss_raw_data_;
    sensor_msgs::NavSatFix::Ptr gnss_fix_data_;

    std::string imu_frame_id_, gnss_frame_id_;

    ros::NodeHandle private_nh_;
    int laser_model_;

    bool init_success;


    static constexpr const double gyroscopeScale = 0.09766 * M_PI / 180; // rad/sec
    // const double temperatureScale = 0.1453; // C
    // const double temperatureOffset = 25; // C

    // used to scale the imu data integer to m/s^2
    static constexpr const double accelerometerScale = (0.001221 * 9.80665); // m/s^2
  };

} // namespace velodyne_pointcloud
