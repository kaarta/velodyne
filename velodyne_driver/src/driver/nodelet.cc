/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Modified 2018 Kaarta - Shawn Hanna
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver nodelet for the Velodyne 3D LIDARs
 */

#include <string>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "velodyne_driver/driver.h"

namespace velodyne_driver
{

class DriverNodelet: public nodelet::Nodelet
{
public:

  DriverNodelet():
    running_(false),
    running_2_(false)
  {}

  ~DriverNodelet()
  {
    // It's ok to kill this program because ros is going down
     // to prevent hanging, don't "join" threads that won't stop by themselves.
    if (running_ || running_2_)
      exit(1);
    
    // if (running_)
    //   {
    //     std::cerr << "shutting down velodyne points thread";
    //     running_ = false;
    //     deviceThread_->join();
    //     std::cerr << "driver thread stopped";
    //   }
    // if (running_2_)
    //   {
    //     std::cerr << "shutting down velodyne position packets thread";
    //     running_2_ = false;
    //     positionPacketThread_->join();
    //     std::cerr << "position packet thread stopped";
    //   }
  }

private:

  virtual void onInit(void);
  virtual void devicePoll(void);
  virtual void positionPoll(void);

  volatile bool running_;               ///< device thread is running
  volatile bool running_2_;               ///< device thread is running
  boost::shared_ptr<boost::thread> deviceThread_;
  boost::shared_ptr<boost::thread> positionPacketThread_;

  boost::shared_ptr<VelodyneDriver> dvr_; ///< driver implementation class
};

void DriverNodelet::onInit()
{
  // start the driver
  dvr_.reset(new VelodyneDriver(getNodeHandle(), getPrivateNodeHandle()));

  sleep(1); // wait for subscribers to be ready, mostly in the case of pcap reading

  if (dvr_->initSuccessful()){
    // spawn device poll thread
    running_ = true;
    running_2_ = true;
    deviceThread_ = boost::shared_ptr< boost::thread >
      (new boost::thread(boost::bind(&DriverNodelet::devicePoll, this)));
    positionPacketThread_ = boost::shared_ptr< boost::thread >
      (new boost::thread(boost::bind(&DriverNodelet::positionPoll, this)));
  }
}

/** @brief Device poll thread main loop. */
void DriverNodelet::devicePoll()
{
  while(ros::ok())
    {
      // poll device until end of file
      running_ = dvr_->poll();
      if (!running_)
        break;
    }
    ROS_INFO("Finished polling packets");
  running_ = false;
}

/** @brief Device poll thread main loop. */
void DriverNodelet::positionPoll()
{
  while(ros::ok())
    {
      // poll device until end of file
      running_2_ = dvr_->pollPosition();
      if (!running_ || !running_2_)
        break;
    }
    ROS_INFO("Finished polling position packets");
  running_2_ = false;
}

} // namespace velodyne_driver

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters are: class type, base class type
PLUGINLIB_EXPORT_CLASS(velodyne_driver::DriverNodelet, nodelet::Nodelet)
