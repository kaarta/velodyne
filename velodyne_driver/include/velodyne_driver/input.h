/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2015, Jack O'Quin
 *  Modified 2018 Kaarta - Shawn Hanna
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file
 *
 *  Velodyne 3D LIDAR data input classes
 *
 *    These classes provide raw Velodyne LIDAR input packets from
 *    either a live socket interface or a previously-saved PCAP dump
 *    file.
 *
 *  Classes:
 *
 *     velodyne::Input -- base class for accessing the data
 *                      independently of its source
 *
 *     velodyne::InputSocket -- derived class reads live data from the
 *                      device via a UDP socket
 *
 *     velodyne::InputPCAP -- derived class provides a similar interface
 *                      from a PCAP dump file
 */

#ifndef __VELODYNE_INPUT_H
#define __VELODYNE_INPUT_H

#include <unistd.h>
#include <stdio.h>
#include <pcap.h>
#include <netinet/in.h>

#include <ros/ros.h>
#include <velodyne_msgs/VelodynePacket.h>
#include <velodyne_msgs/VelodynePositionPacket.h>
#include <thread>
#include <mutex>
#include <condition_variable>

namespace velodyne_driver
{
  static uint16_t DATA_PORT_NUMBER = 2368;     // default data port
  static uint16_t POSITION_PORT_NUMBER = 8308; // default position port

  /** @brief Velodyne input base class */
  class Input
  {
  public:
    Input(ros::NodeHandle private_nh, uint16_t port, uint16_t position_port);
    virtual ~Input() {}

    /** @brief Read one Velodyne packet.
     *
     * @param pkt points to VelodynePacket message
     *
     * @returns 0 if successful,
     *          -1 if end of file
     *          > 0 if incomplete packet (is this possible?)
     */
    virtual int getPacket(velodyne_msgs::VelodynePacket *pkt,
                          const double time_offset) = 0;
    virtual int getPositionPacket(velodyne_msgs::VelodynePositionPacket *pkt, const double time_offset) = 0;

    uint8_t getPPSStatus();
    bool pollPositionPacket(velodyne_msgs::VelodynePositionPacket* packet);

    // convert a velodyne timestamp into current ROS time (currently assumes the velodyne is very closely synchronized to the sensor/IMU timestamps)
    inline ros::Time parseInternalTime(const uint8_t* bytes, const ros::Time& system_time) const;
    velodyne_msgs::VelodynePositionPacket getPositionPacket();

  protected:
    ros::NodeHandle private_nh_;
    uint16_t port_;
    uint16_t position_port_;
    std::string devip_str_;
    std::string last_nmea_sentence_;
    bool process_position_packets_;
    velodyne_msgs::VelodynePositionPacket last_position_packet_;
    uint8_t pps_status_;
    bool new_gps_packet_;
    std::mutex position_data_mutex_;
  };

  /** @brief Live Velodyne input from socket. */
  class InputSocket: public Input
  {
  public:
    InputSocket(ros::NodeHandle private_nh,
                uint16_t port = DATA_PORT_NUMBER, uint16_t position_port = POSITION_PORT_NUMBER);
    virtual ~InputSocket();

    virtual int getPacket(velodyne_msgs::VelodynePacket *pkt, 
                          const double time_offset);
    virtual int getPositionPacket(velodyne_msgs::VelodynePositionPacket *pkt,
                          const double time_offset);
    void setDeviceIP( const std::string& ip );
  private:

  private:
    int sockfd_, position_sockfd_;
    in_addr devip_;
  };


  /** @brief Velodyne input from PCAP dump file.
   *
   * Dump files can be grabbed by libpcap, Velodyne's DSR software,
   * ethereal, wireshark, tcpdump, or the \ref vdump_command.
   */
  class InputPCAP : public Input
  {
  public:
    InputPCAP(ros::NodeHandle private_nh,
              uint16_t port = DATA_PORT_NUMBER,
              uint16_t position_port = POSITION_PORT_NUMBER,
              std::string filename="",
              bool read_once=false,
              bool read_fast=false,
              double repeat_delay=0.0);
    virtual ~InputPCAP();

    virtual int getPacket(velodyne_msgs::VelodynePacket *pkt, 
                          const double time_offset);
    virtual int getPositionPacket(velodyne_msgs::VelodynePositionPacket *pkt,
                          const double time_offset);

    void setDeviceIP( const std::string& ip );

  private:
    ros::Time last_time_;
    std::string filename_;
    pcap_t *pcap_;
    bpf_program pcap_packet_filter_;
    char errbuf_[PCAP_ERRBUF_SIZE];
    bool empty_;
    bool read_once_;
    bool read_fast_;
    double repeat_delay_;
    bool new_position_packet_;
    std::condition_variable position_data_conditional_variable_;
  };

} // velodyne_driver namespace

#endif // __VELODYNE_INPUT_H
