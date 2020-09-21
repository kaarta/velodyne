/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *  Modified 2018 Kaarta - Shawn Hanna
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the Velodyne 3D LIDARs
 */

#include <string>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_msgs/VelodynePositionPacket.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/NavSatFix.h>

#include "velodyne_driver/driver.h"
#include <ros/package.h>

namespace velodyne_driver
{

VelodyneDriver::VelodyneDriver(ros::NodeHandle node,
                               ros::NodeHandle private_nh) : init_success(true)
{
  // use private node handle to get parameters
  private_nh.param("frame_id", config_.frame_id, std::string("velodyne"));
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("Velodyne tf_prefix: " << tf_prefix);
  config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);

  dual_return_ = false;
  laser_model_ = -1;

  // get model name, validate string, determine packet rate
  private_nh.param("model", config_.model, std::string("64E"));
  std::string model_full_name;
  node.getParam("/laser_model_", laser_model_);
  force_laser_model_ = -1000;
  if (node.getParam("/force_laser_model", force_laser_model_))
  {
    laser_model_ = force_laser_model_;
    ROS_WARN("Forcing use of laser model %d in driver", laser_model_);
  }
  if (laser_model_ == 0){
    configureForModelByte(0x22);
    config_.model = "VLP16";
    model_full_name = "VLP-16";
    ROS_INFO("Setting model to: %s", model_full_name.c_str());
  }
  else if (laser_model_ == 1){
    configureForModelByte(0x28);
    config_.model = "32C";
    model_full_name = "VLP-32C";
    ROS_INFO("Setting model to: %s", model_full_name.c_str());
  }
  else if (laser_model_ == 2){
    configureForModelByte(0x21);
    config_.model = "32E";
    model_full_name = std::string("HDL-") + config_.model;
    ROS_INFO("Setting model to: %s", model_full_name.c_str());
  }
  else{
    ROS_ERROR("Could not identify /laser_model_ parameter: %d. Initial driver configuration failed", laser_model_);

    if ((config_.model == "64E_S2") || 
        (config_.model == "64E_S2.1"))    // generates 1333312 points per second
    {                                   // 1 packet holds 384 points
        packet_rate = 3472.17;            // 1333312 / 384
        model_full_name = std::string("HDL-") + config_.model;
    }
    else if (config_.model == "64E")
    {
        packet_rate = 2600.0;
        model_full_name = std::string("HDL-") + config_.model;
    }
    else if (config_.model == "32E")
    {
        packet_rate = 1808.0;
        model_full_name = std::string("HDL-") + config_.model;
    }
      else if (config_.model == "32C")
    {
        packet_rate = 1507.0;
        model_full_name = std::string("VLP-") + config_.model;
    }
    else if (config_.model == "VLP16")
    {
        packet_rate = 754;             // 754 Packets/Second for Last or Strongest mode 1508 for dual (VLP-16 User Manual)
        model_full_name = "VLP-16";
    }
    else
    {
        ROS_ERROR_STREAM("unknown Velodyne LIDAR model: " << config_.model);
        packet_rate = 754;             // 754 Packets/Second for Last or Strongest mode 1508 for dual (VLP-16 User Manual)
        config_.model = "VLP16";
        model_full_name = "VLP-16";
        ROS_INFO("Setting model to: %s", model_full_name.c_str());
    }
    private_nh.getParam("npackets", packet_rate);
  }

  std::string deviceName(std::string("Velodyne ") + model_full_name);

  private_nh.param("rpm", config_.rpm, 600.0);
  last_rpm_ = -1;
  num_same_rpm_ = 11; // initialize the same rpm counter
  ROS_INFO_STREAM(deviceName << " assumed to be rotating at " << config_.rpm << " RPM");

  // set the frequency/packet count
  double frequency = (config_.rpm / 60.0);     // expected Hz rate

  updateNPackets();

  ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

  std::string dump_file;
  private_nh.param("pcap", dump_file, std::string(""));

  private_nh.param("publish_position_packets_at_same_frequency_as_scans", publish_position_packets_at_same_frequency_as_scans_, true);

  double cut_angle;
  private_nh.param("cut_angle", cut_angle, -0.01);
  if (cut_angle < 0.0)
  {
    ROS_INFO_STREAM("Cut at specific angle feature deactivated.");
  }
  else if (cut_angle < (2*M_PI))
  {
      ROS_INFO_STREAM("Cut at specific angle feature activated. " 
        "Cutting velodyne points always at " << cut_angle << " rad.");
  }
  else
  {
    ROS_ERROR_STREAM("cut_angle parameter is out of range. Allowed range is "
    << "between 0.0 and 2*PI or negative values to deactivate this feature.");
    cut_angle = -0.01;
    init_success = false;
  }

  // Convert cut_angle from radian to one-hundredth degree, 
  // which is used in velodyne packets
  config_.cut_angle = int((cut_angle*360/(2*M_PI))*100);

  int udp_port, position_port;
  private_nh.param("port", udp_port, (int) DATA_PORT_NUMBER);
  private_nh.param("position_port", position_port, (int) 0);

  ROS_INFO("Data port: %d, Position port: %d", udp_port, position_port);

  // Initialize dynamic reconfigure
  srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_driver::
    VelodyneNodeConfig> > (private_nh);
  dynamic_reconfigure::Server<velodyne_driver::VelodyneNodeConfig>::
    CallbackType f;
  f = boost::bind (&VelodyneDriver::callback, this, _1, _2);
  srv_->setCallback (f); // Set callback function und call initially

  // initialize diagnostics
  diagnostics_.setHardwareID(deviceName);
  const double diag_freq = frequency;
  diag_max_freq_ = diag_freq;
  diag_min_freq_ = diag_freq;
  ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

  using namespace diagnostic_updater;
  diag_topic_.reset(new TopicDiagnostic("velodyne points", diagnostics_,
                                        FrequencyStatusParam(&diag_min_freq_,
                                                             &diag_max_freq_,
                                                             0.05, 1),
                                        TimeStampStatusParam(0.0, 0.5)));
  // diag_max_position_freq_ = publish_position_packets_at_same_frequency_as_scans_ ? diag_freq : 140;
  diag_min_position_freq_ = diag_max_position_freq_;
  diag_position_topic_.reset(new TopicDiagnostic("velodyne position packets", diagnostics_,
                                        FrequencyStatusParam(&diag_min_position_freq_,
                                                             &diag_max_position_freq_,
                                                             0.05, 1),
                                        TimeStampStatusParam(0.0, 0.5)));
  diag_timer_ = private_nh.createTimer(ros::Duration(0.2), &VelodyneDriver::diagTimerCallback,this);
  diagnostics_.add("velodyne status", this, &VelodyneDriver::produce_diagnostics);

  // open Velodyne input device or file
  if (dump_file != "")                  // have PCAP file?
    {
      ROS_INFO_STREAM("loading file: " << dump_file);
      // read data from packet capture file
      input_.reset(new velodyne_driver::InputPCAP(private_nh, udp_port, position_port,
                                                  dump_file));
    }
  else
    {
      // read data from live socket
      input_.reset(new velodyne_driver::InputSocket(private_nh, udp_port, position_port));
    }

  // raw packet output topic
  output_ =
    node.advertise<velodyne_msgs::VelodyneScan>("velodyne_packets", 10);
  rpm_pub_ =
    node.advertise<std_msgs::Float32>("velodyne_rpm", 10);
  if (position_port != 0){
    position_packet_pub_ =
      node.advertise<velodyne_msgs::VelodynePositionPacket>("velodyne_position_packets", 10);
  }
}

bool VelodyneDriver::initSuccessful(){
  return init_success;
}

bool VelodyneDriver::configureForModelByte(uint8_t new_byte)
{
  switch(force_laser_model_){
    case 0:
      new_byte = 0x22;
      break;
    case 1:
      new_byte = 0x28;
      break;
    case 2:
      new_byte = 0x21;
      break;
    case -1000: // do nothing. not using force model
      break;
    default:
      ROS_WARN_THROTTLE(1, "Invalid force laser model given: %d", force_laser_model_);
  }
  switch (new_byte){
    case 0x22:
      laser_model_ = 0;
      packet_rate = 754;
      updateNPackets();
      break;
    case 0x28:
      laser_model_ = 1;
      packet_rate = 1507.0;
      updateNPackets();
      break;
    case 0x21:
      laser_model_ = 2;
      packet_rate = 1808.0;
      updateNPackets();
      break;
    default:
      ROS_ERROR_THROTTLE(1, "Invalid model byte: %u", uint32_t(new_byte));
  }
}

void VelodyneDriver::updateNPackets()
{
  double frequency = (config_.rpm / 60.0);     // expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)
  config_.npackets = (int) ceil(packet_rate / frequency);
  if (dual_return_)
    config_.npackets *= 2;

}

void VelodyneDriver::setRPM(double rpm){
  if (rpm < 300){
    ROS_ERROR("Invalid RPM: %lf", rpm);
    return;
  }
  double frequency = (rpm / 60.0);     // expected Hz rate

  config_.rpm = rpm;
  updateNPackets();

  const double diag_freq = frequency;
  diag_max_freq_ = diag_freq;
  diag_min_freq_ = diag_freq;
  diag_max_position_freq_ = diag_freq * config_.npackets / 14;
  diag_min_position_freq_ = diag_freq * config_.npackets / 14;

  ROS_INFO_STREAM("New RPM detected. Publishing " << config_.npackets << " packets per scan. RPM = " << config_.rpm);
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool VelodyneDriver::poll(void)
{
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  velodyne_msgs::VelodyneScanPtr scan(new velodyne_msgs::VelodyneScan);

  // Extract base rotation of first block in packet
  static const std::size_t azimuth_data_pos = 100*0+2;

  if( config_.cut_angle >= 0) //Cut at specific angle feature enabled
  {
    static int last_azimuth = -1;
    scan->packets.reserve(config_.npackets);
    velodyne_msgs::VelodynePacket tmp_packet;
    while(true)
    {
      while(true)
      {
        int rc = input_->getPacket(&tmp_packet, config_.time_offset);
        if (rc == 0) break;       // got a full packet?
        if (rc == 1){
          continue; // position packet received?
        }
        if (rc < 0) return false; // end of file reached?
      }
      scan->packets.push_back(tmp_packet);

      if (tmp_packet.data[0x4b5] != expected_factory_byte_)
      {
        configureForModelByte(tmp_packet.data[0x4b5]);
      }

      int azimuth = *( (u_int16_t*) (&tmp_packet.data[azimuth_data_pos]));

      bool dual_mode = (tmp_packet.data[0x4b4] == 0x39);
      if (dual_return_ != dual_mode)
      {
        dual_return_ = dual_mode;
        updateNPackets();
      }
      // ROS_WARN("%d", (last_azimuth - azimuth));

      //if first packet in scan, there is no "valid" last_azimuth
      if (last_azimuth == -1) {
      	 last_azimuth = azimuth;
      	 continue;
      }
      if((last_azimuth < config_.cut_angle && config_.cut_angle <= azimuth)
      	 || ( config_.cut_angle <= azimuth && azimuth < last_azimuth)
      	 || (azimuth < last_azimuth && last_azimuth < config_.cut_angle))
      {
        ROS_DEBUG("Last azimuth. last azimuth: %d. azimuth: %d", last_azimuth, azimuth);
        last_azimuth = azimuth;
        break; // Cut angle passed, one full revolution collected
      }
      last_azimuth = azimuth;
    }
  }
  else // standard behaviour
  {
  // Since the velodyne delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
    scan->packets.resize(config_.npackets);
    for (int i = 0; i < config_.npackets; ++i)
    {
      while (true)
        {
          // keep reading until full packet received
          int rc = input_->getPacket(&scan->packets[i], config_.time_offset);
    
          if (scan->packets.size())
          {
            if (scan->packets.back().data[0x4b5] != expected_factory_byte_)
            {
              configureForModelByte(scan->packets.back().data[0x4b5]);
            }

            bool dual_mode = (scan->packets.back().data[0x4b4] == 0x39);
            if (dual_return_ != dual_mode)
            {
              dual_return_ = dual_mode;
              updateNPackets();
              scan->packets.resize(config_.npackets);
            }
          }
      
          if (rc == 1){
            continue; // position packet received
          }
          if (rc == 0) break;       // got a full packet?
          if (rc < 0) return false; // end of file reached?
        }
    }
  }

  if (force_laser_model_ != -1000)
  {
    uint8_t model_byte;
    switch(force_laser_model_){
      case 0:
        model_byte = 0x22;
        break;
      case 1:
        model_byte = 0x28;
        break;
      case 2:
        model_byte = 0x21;
        break;
      default:
        ROS_ERROR_THROTTLE(1, "Invalid force_laser_model given: %d", force_laser_model_);
        model_byte = 0;
    }

    for (unsigned int i=0; i < scan->packets.size(); ++i)
      scan->packets[i].data[0x4b5] = model_byte;
  }

  // calculate RPM
  if (scan->packets.size() > 2){
    int last_azimuth = *( (u_int16_t*) (&(scan->packets.back().data[azimuth_data_pos])));
    int first_azimuth = *( (u_int16_t*) (&scan->packets.front().data[azimuth_data_pos]));

    // figure out how many revolutions we've gone through in this packet. we can't assume we guessed correctly, so 
    int num_revolutions = 0;
    int prev_azimuth, azimuth;
    prev_azimuth = azimuth = first_azimuth;
    for (unsigned int i=0; i < scan->packets.size(); ++i){
      azimuth = *( (u_int16_t*) (&scan->packets[i].data[azimuth_data_pos]));
      if (azimuth < prev_azimuth){
        ++num_revolutions;
      }
      prev_azimuth = azimuth;
    }

    // convert delta angle to delta revolutions and then divide by delta time
    float rpm = ( ( (36000*num_revolutions + last_azimuth - first_azimuth) / 100.) / (360) )/((scan->packets.back().stamp - scan->packets.front().stamp).toSec());
    rpm = rpm * 60; // convert rev/sec to rev/min
    float rpm_rounded = ( ((int)(rpm+30)) / 60) * 60;
    ROS_DEBUG_THROTTLE(10, "number packets: %d, first azimuth: %f, last_azimuth: %f, num revolutions: %d Detected RPM: %f - rounded to multiple of 60: %f",
                          (int) scan->packets.size(), first_azimuth/100.0f, last_azimuth/100.0f, num_revolutions, rpm, rpm_rounded);

    // check to see if we're getting a few of the same RPM
    if (last_rpm_ == -1){
      setRPM(rpm_rounded);
      last_rpm_ = rpm_rounded;
      num_same_rpm_ = 11;
    }
    else if (last_rpm_ == rpm_rounded){
      if (config_.rpm == rpm_rounded){
        num_same_rpm_ = 11; // we're already set to this value, so don't process more
      }
      else{ // different rpm than what's expected. Let's wait to see if this continues
        num_same_rpm_++;
        if (num_same_rpm_ == 10){
          // set the frequency/packet count
          setRPM(rpm_rounded);
        }
      }
    }
    else{
      num_same_rpm_ = 0;
      last_rpm_ = rpm_rounded;
    }
    std_msgs::Float32 out;
    out.data = rpm;
    rpm_pub_.publish(out);
  }

  // publish message using time of first packet
  if (scan->packets.size() < config_.npackets * .95 || scan->packets.size() > config_.npackets * 1.05)
  {
    ROS_WARN_THROTTLE(0.01, "Weird number of packets: %d. Expected: %d",
                      (int)scan->packets.size(), (int)config_.npackets);

    int delta = 0;
    static int azimuth_prev;
    for (int i=0;i < scan->packets.size(); ++i)
    {
      int azimuth = *( (u_int16_t*) (&scan->packets[i].data[azimuth_data_pos]));

      delta = (azimuth - azimuth_prev + 36000) % 36000;

      ROS_DEBUG("Azimuth[%d]: %8d. delta: %8d", i, azimuth, delta);
      azimuth_prev = azimuth;
    }
  }
  ROS_DEBUG("Publishing a full Velodyne scan.");
  scan->header.stamp = scan->packets.front().stamp;
  scan->header.frame_id = config_.frame_id;
  output_.publish(scan);

  if(publish_position_packets_at_same_frequency_as_scans_){
    velodyne_msgs::VelodynePositionPacket position_packet = input_->getPositionPacket();
    if (position_packet.stamp.sec > 0){
      position_packet_pub_.publish(position_packet);

      // ROS_WARN("GPS data time stamp: " << gps_data_.header.stamp);
      diag_position_topic_->tick(position_packet.stamp);
      diagnostics_.update();
    }
  }

  // notify diagnostics that a message has been published, updating
  // its status
  diag_topic_->tick(scan->header.stamp);
  diagnostics_.update();

  return true;
}

bool VelodyneDriver::pollPosition(void){
  // Since the velodyne delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
  // sensor_msgs::NavSatFix gps_data_;
  velodyne_msgs::VelodynePositionPacket position_packet_;

  while (true)
  {
    // keep reading until full packet received
    int rc = input_->getPositionPacket(&position_packet_, config_.time_offset);
    if (rc == 0){ // got a full packet?
      // if we aren't throttling publishing to only once per scan
      if (!publish_position_packets_at_same_frequency_as_scans_){
        position_packet_pub_.publish(position_packet_);

        // if we have new gps data, publish it
        // ROS_WARN("GPS data time stamp: " << gps_data_.header.stamp);
        diag_position_topic_->tick(position_packet_.stamp);
        diagnostics_.update();
      }
      break;
    }
    else if (rc < 0){
      return false; // end of file reached, or position packet processing disabled
    }
  }
  return true;
}

void VelodyneDriver::callback(velodyne_driver::VelodyneNodeConfig &config,
              uint32_t level)
{
  ROS_INFO("Reconfigure Request");
  config_.time_offset = config.time_offset;
}

void VelodyneDriver::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Velodyne OK");
  switch(input_->getPPSStatus()){
    case 0:
      stat.add("PPS Status", "No PPS detected");
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "No PPS detected");
      ROS_DEBUG_THROTTLE(0.5, "No PPS Detected");
      break;
    case 1:
      stat.add("PPS Status", "Synchronizing to PPS");
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Synchronizing to PPS");
      ROS_WARN_THROTTLE(0.5, "Synchronizing PPS");
      break;
    case 2:
      stat.add("PPS Status", "PPS Locked");
      break;
    case 3:
      stat.add("PPS Status", "Error");
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "PPS Error");
      ROS_WARN_THROTTLE(0.5, "PPS Error");
      break;
    default:
      stat.add("PPS Status", input_->getPPSStatus());
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Unknown PPS Status");
      ROS_WARN_THROTTLE(0.5, "Unknown PPS Status");
  };
  // sensor_msgs::NavSatFix gps_data = input_->getGPSData();
  // velodyne_msgs::VelodynePositionPacket packet_ = input_->getPositionPacket();
  // stat.add("GPS Location Timestamp", gps_data.header.stamp);
  // stat.add("GPS Location Latitude", gps_data.latitude);
  // stat.add("GPS Location Longitude", gps_data.longitude);
  // stat.add("GPS Location Altitude", gps_data.altitude);
  stat.add("Detected RPM", config_.rpm);
  stat.add("Last full revolution of sensor's rpm", last_rpm_);
  // stat.add("Position Packet timestamp", packet_.stamp);
}

void VelodyneDriver::diagTimerCallback(const ros::TimerEvent &event)
{
  (void)event;
  // Call necessary to provide an error when no velodyne packets are received
  diagnostics_.update();
}

} // namespace velodyne_driver
