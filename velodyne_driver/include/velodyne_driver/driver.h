/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Modified 2018 Kaarta - Shawn Hanna
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver interface for the Velodyne 3D LIDARs
 */

#ifndef _VELODYNE_DRIVER_H_
#define _VELODYNE_DRIVER_H_ 1

#include <string>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>

#include <velodyne_driver/input.h>
#include <velodyne_driver/VelodyneNodeConfig.h>

namespace velodyne_driver
{

class VelodyneDriver
{
public:

  VelodyneDriver(ros::NodeHandle node,
                 ros::NodeHandle private_nh);
  ~VelodyneDriver() {}

  bool poll(void);
  bool pollPosition(void);

  bool initSuccessful();

private:

  ///Callback for dynamic reconfigure
  void callback(velodyne_driver::VelodyneNodeConfig &config,
              uint32_t level);
  /// Callback for diagnostics update for lost communication with vlp
  void diagTimerCallback(const ros::TimerEvent&event);

  ///Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<velodyne_driver::
              VelodyneNodeConfig> > srv_;

  // configuration parameters
  struct
  {
    std::string frame_id;            ///< tf frame ID
    std::string model;               ///< device model name
    int    npackets;                 ///< number of packets to collect
    double rpm;                      ///< device rotation rate (RPMs)
    int cut_angle;                   ///< cutting angle in 1/100°
    double time_offset;              ///< time in seconds added to each velodyne time stamp
  } config_;

  boost::shared_ptr<Input> input_;
  ros::Publisher output_;
  ros::Publisher position_packet_pub_;
  ros::Publisher rpm_pub_;

  /** diagnostics updater */
  ros::Timer diag_timer_;
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  double diag_max_position_freq_;
  double diag_min_position_freq_;
  bool publish_position_packets_at_same_frequency_as_scans_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_, diag_position_topic_;
  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

  void setRPM(double frequency);        // set the expected RPM of the system
  int num_same_rpm_;
  double last_rpm_;
  double packet_rate;                   // packet frequency (Hz)

  bool init_success;

  bool dual_return_;
  int force_laser_model_;
  void updateNPackets();
};

} // namespace velodyne_driver

#endif // _VELODYNE_DRIVER_H_
