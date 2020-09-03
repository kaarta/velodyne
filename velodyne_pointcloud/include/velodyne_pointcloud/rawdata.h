/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *  Modified 2018 Kaarta - Shawn Hanna
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file
 *
 *  @brief Interfaces for interpreting raw packets from the Velodyne 3D LIDAR.
 *
 *  @author Yaxin Liu
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 */

#ifndef __VELODYNE_RAWDATA_H
#define __VELODYNE_RAWDATA_H

#include <errno.h>
#include <stdint.h>
#include <string>
#include <boost/format.hpp>
#include <math.h>

#include <ros/ros.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_pointcloud/calibration.h>
#include <velodyne_pointcloud/datacontainerbase.h>
#include <pcl_ros/point_cloud.h>
#include <velodyne_pointcloud/point_types.h>


namespace velodyne_rawdata
{
    // Shorthand typedefs for point cloud representations
  // typedef velodyne_pointcloud::PointXYZIR VPoint;
  typedef Kaarta::KaartaSensorPoint VPoint;
  typedef velodyne_pointcloud::RawSensorPoint VPointRaw;
  // typedef pcl::PointCloud<VPoint> VPointCloud;
  typedef pcl::PointCloud<VPoint> VPointCloud;
  typedef pcl::PointCloud<VPointRaw> VPointCloudRaw;

  /**
   * Raw Velodyne packet constants and structures.
   */
  static const int SIZE_BLOCK = 100;
  static const int RAW_SCAN_SIZE = 3;
  static const int SCANS_PER_BLOCK = 32;
  static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

  static const double ROTATION_RESOLUTION     =     0.01;  // [deg]
  static const uint16_t ROTATION_MAX_UNITS    = 36000u;     // [deg/100]

  /** @todo make this work for both big and little-endian machines */
  static const uint16_t UPPER_BANK = 0xeeff;
  static const uint16_t LOWER_BANK = 0xddff;
  
  
  /** Special Defines for VLP16 support **/
  static const int    VLP16_FIRINGS_PER_BLOCK =   2;
  static const int    VLP16_SCANS_PER_FIRING  =  16;
  static const float  VLP16_DSR_TOFFSET       =   2.304 * 1e-6;   // [µs]
  static const float  VLP16_FIRING_TOFFSET    =  55.296 * 1e-6;   // [µs]
  static const float  VLP16_BLOCK_TDURATION   = 2*VLP16_FIRING_TOFFSET;   // [µs]

  static const int    VLP32C_FIRINGS_PER_BLOCK =   1;
  static const int    VLP32C_SCANS_PER_FIRING  =  32;
  static const float  VLP32C_DSR_TOFFSET       =   2.304 * 1e-6;   // [µs]
  static const float  VLP32C_FIRING_TOFFSET    =  55.296 * 1e-6;   // [µs]
  static const float  VLP32C_BLOCK_TDURATION   =  VLP32C_FIRING_TOFFSET;   // [µs]

  static const int    HDL32E_FIRINGS_PER_BLOCK =   1;
  static const int    HDL32E_SCANS_PER_FIRING  =  32;
  static const float  HDL32E_DSR_TOFFSET       =   1.152 * 1e-6;   // [µs]
  static const float  HDL32E_FIRING_TOFFSET    =  46.080 * 1e-6;   // [µs]
  static const float  HDL32E_BLOCK_TDURATION   =  HDL32E_FIRING_TOFFSET;   // [µs]
  

  /** \brief Raw Velodyne data block.
   *
   *  Each block contains data from either the upper or lower laser
   *  bank.  The device returns three times as many upper bank blocks.
   *
   *  use stdint.h types, so things work with both 64 and 32-bit machines
   */
  typedef struct raw_block
  {
    uint16_t header;        ///< UPPER_BANK or LOWER_BANK
    uint16_t rotation;      ///< 0-35999, divide by 100 to get degrees
    uint8_t  data[BLOCK_DATA_SIZE];
  } raw_block_t;

  /** used for unpacking the first two data bytes in a block
   *
   *  They are packed into the actual data stream misaligned.  I doubt
   *  this works on big endian machines.
   */
  union two_bytes
  {
    uint16_t uint;
    uint8_t  bytes[2];
  };

  static const int PACKET_SIZE = 1206;
  static const int BLOCKS_PER_PACKET = 12;
  static const int PACKET_STATUS_SIZE = 4;
  static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

  /** \brief Raw Velodyne packet.
   *
   *  revolution is described in the device manual as incrementing
   *    (mod 65536) for each physical turn of the device.  Our device
   *    seems to alternate between two different values every third
   *    packet.  One value increases, the other decreases.
   *
   *  \todo figure out if revolution is only present for one of the
   *  two types of status fields
   *
   *  status has either a temperature encoding or the microcode level
   */
  typedef struct raw_packet
  {
    raw_block_t blocks[BLOCKS_PER_PACKET];
    uint16_t revolution;
    uint8_t status[PACKET_STATUS_SIZE]; 
  } raw_packet_t;

  /** \brief Velodyne data conversion class */
  class RawData
  {
  public:

    RawData();
    ~RawData() {}

    /** \brief Set up for data processing.
     *
     *  Perform initializations needed before data processing can
     *  begin:
     *
     *    - read device-specific angles calibration
     *
     *  @param private_nh private node handle for ROS parameters
     *  @returns 0 if successful;
     *           errno value for failure
     */
    int setup(ros::NodeHandle private_nh);

    int setupOffline(int _model, std::string _calibration, bool _upward);
    bool offline_setup;

    /** \brief set up the laser configuration files using an integer laser model
     * @param laser_model
     *   0 = VLP16
     *   1 = VLP32-C
     *   2 = HDL-32e
     * @returns true if model is one of the accepted types. False otherwise
     */
    bool configureLaserParams(int laser_model, bool dual_mode, bool override = false);

    void unpackRAW(const velodyne_msgs::VelodynePacket &pkt, VPointCloudRaw::Ptr& pc, const ros::Time& scan_begin_stamp);
    void unpackRAW_vlp16(const velodyne_msgs::VelodynePacket &pkt, VPointCloudRaw::Ptr& pc, const ros::Time& scan_begin_stamp);
    
    void unpack(const velodyne_msgs::VelodynePacket &pkt, DataContainerBase& data, const ros::Time& scan_begin_stamp);
    
    void setParameters(double min_range, double max_range, double view_direction,
                       double view_width);

    std::string getCalibrationFilename(){ return config_.calibrationFile; };

    velodyne_pointcloud::Calibration& getCalibrations(){ return calibration_;};

    bool isInitialized(){ return initialized_; };

  private:

    /** configuration parameters */
    typedef struct {
      std::string calibrationFile;     ///< calibration file name
      double max_range;                ///< maximum range to publish
      double min_range;                ///< minimum range to publish
      int min_angle;                   ///< minimum angle to publish
      int max_angle;                   ///< maximum angle to publish
      
      double tmp_min_angle;
      double tmp_max_angle;
      uint8_t expected_factory_byte;
      bool dual_return_mode;
    } Config;
    Config config_;


    /** Upward or downward mount*/
    bool upward;

    bool buildTimings();

    /** 
     * Calibration file
     */
    velodyne_pointcloud::Calibration calibration_;
    float sin_rot_table_[ROTATION_MAX_UNITS];
    float cos_rot_table_[ROTATION_MAX_UNITS];
    
    /** add private function to handle the VLP16 **/ 
    void unpack_vlp16(const velodyne_msgs::VelodynePacket &pkt, DataContainerBase& data, const ros::Time& scan_begin_stamp);

    /** in-line test whether a point is in range */
    bool pointInRange(float range)
    {
      return (range >= config_.min_range
              && range <= config_.max_range);
    }

    int last_azimuth_diff;

    std::vector<std::vector<float> > timing_offsets;
    int laser_model;
    int laser_model_forced_;

    bool initialized_;
  };

} // namespace velodyne_rawdata

#endif // __VELODYNE_RAWDATA_H
