/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  Velodyne 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw Velodyne LIDAR packets into useful
 *  formats.
 *
 *  Derived classes accept raw Velodyne data for either single packets
 *  or entire rotations, and provide it in various formats for either
 *  on-line or off-line processing.
 *
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 *
 *  HDL-64E S2 calibration support provided by Nick Hillier
 */

#include <fstream>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>

#include <velodyne_pointcloud/rawdata.h>
#include <kaarta_io/ScanInfoManagerClient.hpp>

namespace velodyne_rawdata
{
  ////////////////////////////////////////////////////////////////////////
  //
  // RawData base class implementation
  //
  ////////////////////////////////////////////////////////////////////////

  RawData::RawData() {}
  
  /** Update parameters: conversions and update */
  void RawData::setParameters(double min_range,
                              double max_range,
                              double view_direction,
                              double view_width)
  {
    config_.min_range = min_range;
    config_.max_range = max_range;

    //converting angle parameters into the velodyne reference (rad)
    config_.tmp_min_angle = view_direction + view_width/2;
    config_.tmp_max_angle = view_direction - view_width/2;
    
    //computing positive modulo to keep theses angles into [0;2*M_PI]
    config_.tmp_min_angle = fmod(fmod(config_.tmp_min_angle,2*M_PI) + 2*M_PI,2*M_PI);
    config_.tmp_max_angle = fmod(fmod(config_.tmp_max_angle,2*M_PI) + 2*M_PI,2*M_PI);
    
    //converting into the hardware velodyne ref (negative yaml and degrees)
    //adding 0.5 perfomrs a centered double to int conversion 
    config_.min_angle = 100 * (2*M_PI - config_.tmp_min_angle) * 180 / M_PI + 0.5;
    config_.max_angle = 100 * (2*M_PI - config_.tmp_max_angle) * 180 / M_PI + 0.5;
    if (config_.min_angle == config_.max_angle)
    {
      //avoid returning empty cloud if min_angle = max_angle
      config_.min_angle = 0;
      config_.max_angle = 36000;
    }
  }

  bool RawData::configureLaserParams(int laser_model_, bool dual_mode, bool override){
    bool res = true;
    ROS_INFO("Configuring laser_model to: %d", laser_model_);
    laser_model = laser_model_;

    if (override){
      // set the parameter and log the change
      Kaarta::ScanInfoManagerROSClient client;
      if (client.init(true)){
        client.publishFormatStr("/adjusted_laser_model", "true");
        client.publishFormatStr("/laser_model", "%d", laser_model);
      }
      ros::param::set("/laser_model", laser_model);
    }
    
    // get path to angles.config file for this device
    config_.dual_return_mode = dual_mode;
    config_.expected_factory_byte = (uint8_t) 0;
    std::string pkgPath = ros::package::getPath("velodyne_pointcloud");
    std::string fallbackCalibrationFile;
    if (laser_model == 0){
      fallbackCalibrationFile = pkgPath + "/params/VLP16db.yaml";
      config_.expected_factory_byte = (uint8_t) 0x22;
    }
    else if (laser_model == 1){
      fallbackCalibrationFile = pkgPath + "/params/VeloView-VLP-32C.yaml";
      config_.expected_factory_byte = (uint8_t) 0x28;
    }
    else if (laser_model == 2){
      fallbackCalibrationFile = pkgPath + "/params/32db.yaml";
      config_.expected_factory_byte = (uint8_t) 0x21;
    }
    else if (laser_model == 16){
      fallbackCalibrationFile = pkgPath + "/params/64e_utexas.yaml";
      config_.expected_factory_byte = (uint8_t) 0;
    }
    else{
      // check calibration file
      res = false;
    }
    std::string calibToUse = fallbackCalibrationFile;
    if (config_.calibrationFile.length() > 0)
    {
      if (boost::filesystem::exists(config_.calibrationFile))
        calibToUse = config_.calibrationFile;
      else
        ROS_ERROR("Calibration file does not exist: %s", config_.calibrationFile.c_str());
    }
    ROS_INFO("Setting calibration file to: %s", calibToUse.c_str());
    
    calibration_.read(calibToUse);
    if (!calibration_.initialized) {
      ROS_ERROR_STREAM("Unable to open calibration file: " << 
          calibToUse);
      res = false;
    }

    // Set up cached values for sin and cos of all the possible headings
    for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
      double rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
      cos_rot_table_[rot_index] = cos(rotation);
      sin_rot_table_[rot_index] = sin(rotation);
    }

    ROS_INFO_STREAM("Number of lasers: " << calibration_.num_lasers << ". Building firing times lookup table. Dual mode = " << config_.dual_return_mode);
    buildTimings();

    return res;
  }

  /** Set up for on-line operation. */
  int RawData::setup(ros::NodeHandle private_nh)
  {
    int res = 0;
    // set laser parameters
    laser_model = 0;
    if (!private_nh.getParam("/laser_model", laser_model))
    {
      ROS_ERROR("No laser model parameter set. Using: %d", laser_model);
    }
    if (!private_nh.getParam("calibration", config_.calibrationFile))
    {
      ROS_ERROR("Failed to read calibration file parameter. Falling back to model specific calibration");
      res = 1;
    }
    if (!configureLaserParams(laser_model, false)){
      res = 2;
    }

    if (!private_nh.getParam("upward", upward))
    {
      ROS_WARN_STREAM("No mounting direction specified! Using upward mounting!");

      // use default upward mounting direction
      upward = true;
    }

    return res;
  }


  bool RawData::buildTimings(){
    // vlp16
    if (laser_model == 0){
      // timing table calculation, from velodyne user manual
      timing_offsets.resize(BLOCKS_PER_PACKET);
      for (size_t i=0; i < timing_offsets.size(); ++i){
        timing_offsets[i].resize(32);
      }
      // constants
      double full_firing_cycle = VLP16_FIRING_TOFFSET; // seconds
      double single_firing = VLP16_DSR_TOFFSET; // seconds
      double dataBlockIndex, dataPointIndex;
      // compute timing offsets
      for (size_t x = 0; x < timing_offsets.size(); ++x){
        for (size_t y = 0; y < timing_offsets[x].size(); ++y){
          if (config_.dual_return_mode){
            dataBlockIndex = (x - (x % 2)) + (y / 16);
          }
          else{
            dataBlockIndex = (x * 2) + (y / 16);
          }
          dataPointIndex = y % 16;
          //timing_offsets[block][firing]
          timing_offsets[x][y] = (full_firing_cycle * dataBlockIndex) + (single_firing * dataPointIndex);
        }
      }
    }
    // vlp32
    else if (laser_model == 1){
      // timing table calculation, from velodyne user manual
      timing_offsets.resize(BLOCKS_PER_PACKET);
      for (size_t i=0; i < timing_offsets.size(); ++i){
        timing_offsets[i].resize(VLP32C_SCANS_PER_FIRING);
      }
      // constants
      double full_firing_cycle = VLP32C_FIRING_TOFFSET; // seconds
      double single_firing = VLP32C_DSR_TOFFSET; // seconds
      double dataBlockIndex, dataPointIndex;
      // compute timing offsets
      for (size_t x = 0; x < timing_offsets.size(); ++x){
        for (size_t y = 0; y < timing_offsets[x].size(); ++y){
          if (config_.dual_return_mode){
            dataBlockIndex = x / 2;
          }
          else{
            dataBlockIndex = x;
          }
          dataPointIndex = y / 2;
          timing_offsets[x][y] = (full_firing_cycle * dataBlockIndex) + (single_firing * dataPointIndex);
        }
      }
    }
    // hdl32
    else if (laser_model == 2){
      // timing table calculation, from velodyne user manual
      timing_offsets.resize(BLOCKS_PER_PACKET);
      for (size_t i=0; i < timing_offsets.size(); ++i){
        timing_offsets[i].resize(HDL32E_SCANS_PER_FIRING);
      }
      // constants
      double full_firing_cycle = HDL32E_FIRING_TOFFSET; // seconds
      double single_firing = HDL32E_DSR_TOFFSET; // seconds
      double dataBlockIndex, dataPointIndex;
      // compute timing offsets
      for (size_t x = 0; x < timing_offsets.size(); ++x){
        for (size_t y = 0; y < timing_offsets[x].size(); ++y){
          if (config_.dual_return_mode){
            dataBlockIndex = x / 2;
          }
          else{
            dataBlockIndex = x;
          }
          dataPointIndex = y / 2;
          timing_offsets[x][y] = (full_firing_cycle * dataBlockIndex) + (single_firing * dataPointIndex);
        }
      }
    }
    else{
      timing_offsets.clear();
    }

    if (timing_offsets.size()){
      // ROS_INFO("VELODYNE TIMING TABLE:");
      for (size_t x = 0; x < timing_offsets.size(); ++x){
        for (size_t y = 0; y < timing_offsets[x].size(); ++y){
          printf("%04.3f ", timing_offsets[x][y] * 1e6);
        }
        printf("\n");
      }
    }
    else{
      ROS_WARN("NO TIMING OFFSETS CALCULATED. ARE YOU USING A SUPPORTED VELODYNE SENSOR?");
      return false;
    }
    return true;
  }

  /** @brief convert raw packet to point cloud
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
  void RawData::unpack(const velodyne_msgs::VelodynePacket &pkt, DataContainerBase& data, const ros::Time& scan_begin_stamp)
  {
    // ROS_WARN_STREAM("Received packet, time: " << pkt.stamp <<" scan begin time = "<<scan_begin_stamp << "diff = " << (scan_begin_stamp - pkt.stamp));

    bool pkt_dual_mode = false;
    if (pkt.data[0x4b4] != 0x37){ // return mode: strongest = 0x37, last = 0x38, dual = 0x39
      switch(pkt.data[0x4b4])
      {
        case 0x38:
          ROS_ERROR_THROTTLE(10, "Expected return mode: 0x37 (strongest). Got: %#02x (last)", pkt.data[0x4b4]);
          break;
        case 0x39:
          ROS_ERROR_THROTTLE(10, "Expected return mode: 0x37 (strongest). Got: %#02x (dual)", pkt.data[0x4b4]);
          pkt_dual_mode = true;
          break;
        default:
          ROS_ERROR_THROTTLE(10, "Expected return mode: 0x37 (strongest). Got: %#02x (unknown)", pkt.data[0x4b4]);
          break;
      }
    }
    
    if (pkt.data[0x4b5] != config_.expected_factory_byte || pkt_dual_mode != config_.dual_return_mode){
      ROS_WARN_THROTTLE(1, "Expected model: %#02x. Data packet gives: %#02x", config_.expected_factory_byte, pkt.data[0x4b5]);
      ROS_WARN_THROTTLE(1, "Expected Dual mode = %d. Dual mode = %d", config_.dual_return_mode, pkt_dual_mode);
      switch(pkt.data[0x4b5]){
        case 0x22:
          ROS_WARN_THROTTLE(1, "Adjusting laser_model param to 0 with dual return mode: ");
          configureLaserParams(0, pkt_dual_mode, true);
          break;
        case 0x28:
          ROS_WARN_THROTTLE(1, "Adjusting laser_model param to 1");
          configureLaserParams(1, pkt_dual_mode, true);
          break;
        case 0x21:
          ROS_WARN_THROTTLE(1, "Adjusting laser_model param to 2");
          configureLaserParams(2, pkt_dual_mode, true);
          break;
        default:
          ROS_ERROR_THROTTLE(1, "Error: unsupported model # in velodyne packet header: %#02x", pkt.data[0x4b5]);
          exit(1);
      }
    }

    /** special parsing for the VLP16 **/
    if (calibration_.num_lasers == 16)
    {
      unpack_vlp16(pkt, data, scan_begin_stamp);
      return;
    }

    velodyne_rawdata::VPoint point;
    float time_diff_start_to_this_packet = (pkt.stamp - scan_begin_stamp).toSec();

    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];
    static int last_azimuth_diff = 20; // TODO: default to 600 RPM assumption

    int block_jump = config_.dual_return_mode ? 2 : 1;
    for (int block = 0; block < BLOCKS_PER_PACKET; block += block_jump) {

      // upper bank lasers are numbered [0..31]
      // NOTE: this is a change from the old velodyne_common implementation
      int bank_origin = 0;
      if (raw->blocks[block].header == LOWER_BANK) {
        // lower bank lasers are [32..63]
        bank_origin = 32;
      }

      uint16_t block_azimuth = raw->blocks[block].rotation;

      float full_firing_cycle = VLP32C_FIRING_TOFFSET; // seconds
      float single_firing = VLP32C_DSR_TOFFSET; // seconds
      float block_duration = VLP32C_BLOCK_TDURATION;
      if (laser_model == 2){
        full_firing_cycle = HDL32E_FIRING_TOFFSET; // seconds
        single_firing = HDL32E_DSR_TOFFSET; // seconds
        block_duration = HDL32E_BLOCK_TDURATION;
      }

      int azimuth_diff = 0;
      if (block < (BLOCKS_PER_PACKET-block_jump)){
        int raw_azimuth_diff = (int)(raw->blocks[block+block_jump].rotation) - (int)raw->blocks[block].rotation;

        // some packets contain an angle overflow where azimuth_diff < 0 
        if(raw_azimuth_diff < 0)
        {
          raw_azimuth_diff = raw_azimuth_diff + 36000;
          // ROS_WARN_STREAM("Angle overflow: " << raw->blocks[block+1].rotation << " - "<<raw->blocks[block].rotation << " < 0. res = " << raw_azimuth_diff);
          azimuth_diff = raw_azimuth_diff;
        }
        else{
          azimuth_diff = raw_azimuth_diff % 36000;
        }
        last_azimuth_diff = azimuth_diff;
      }else{
        azimuth_diff = last_azimuth_diff;
      }

      // ROS_WARN_STREAM_COND(block < BLOCKS_PER_PACKET-1, "Block [" << block << "]. Angle Diff: " << raw->blocks[block+1].rotation << " - "<<raw->blocks[block].rotation << " = " << azimuth_diff);

      for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE) {
        float x, y, z;
        float intensity;
        // float rot_comp;
        uint8_t laser_number;       ///< hardware laser number

        laser_number = j + bank_origin;
        velodyne_pointcloud::LaserCorrection &corrections = 
          calibration_.laser_corrections[laser_number];

        /** Position Calculation */

        union two_bytes tmp;
        tmp.bytes[0] = raw->blocks[block].data[k];
        tmp.bytes[1] = raw->blocks[block].data[k+1];
        intensity = raw->blocks[block].data[k+2];
        if (config_.dual_return_mode)
        {
          // in dual return mode, the packets are in the order of Last, Strongest/Second strongest
          // If the second data block's return has a greater intensity than the first (ie last != strongest)
          if (raw->blocks[block].data[k+2] < raw->blocks[block + 1].data[k+2])
          {
            intensity = raw->blocks[block + 1].data[k+2];
            tmp.bytes[0] = raw->blocks[block + 1].data[k];
            tmp.bytes[1] = raw->blocks[block + 1].data[k+1];
          }
        }
        /** correct for the laser rotation as a function of timing during the firings **/
        uint16_t azimuth_corrected = round( block_azimuth + (azimuth_diff * ( (j / 2) * single_firing) / block_duration) );
        azimuth_corrected = ((int)azimuth_corrected) % 36000;
        // ROS_ERROR_STREAM_COND(num > 1000, "Angle " << j <<": " << azimuth_corrected << " \t time = " << timing_offsets[block][j]);
        // if hdl32, rotate by 90 deg
        if (laser_model == 2) {
          azimuth_corrected = (azimuth_corrected + 9000) % 36000;
        }

        // ROS_WARN_COND(azimuth_corrected > 36000, "Corrected azimuth > 36000: %d", (int)azimuth_corrected);
        // ROS_WARN_COND(block < BLOCKS_PER_PACKET-1 && raw->blocks[block+1].rotation < raw->blocks[block].rotation && !(azimuth_corrected < raw->blocks[block+1].rotation || azimuth_corrected >= raw->blocks[block].rotation), "Corrected azimuth not correct. Azimuth next block = %d. Azimuth current block range = %d: azimuth %d. block = %d", (int)raw->blocks[block+1].rotation, (int)raw->blocks[block].rotation, (int)azimuth_corrected, block);
        // ROS_WARN_COND(block < BLOCKS_PER_PACKET-1 && raw->blocks[block+1].rotation > raw->blocks[block].rotation && (azimuth_corrected >= raw->blocks[block+1].rotation || azimuth_corrected < raw->blocks[block].rotation), "Corrected azimuth not correct. Azimuth next block = %d. Azimuth current block range = %d: azimuth %d. block = %d", (int)raw->blocks[block+1].rotation, (int)raw->blocks[block].rotation, (int)azimuth_corrected, block);

        /*condition added to avoid calculating points which are not
          in the interesting defined area (min_angle < area < max_angle)*/
        if ((azimuth_corrected >= config_.min_angle 
             && azimuth_corrected <= config_.max_angle 
             && config_.min_angle < config_.max_angle)
             ||(config_.min_angle > config_.max_angle 
             && (azimuth_corrected <= config_.max_angle 
             || azimuth_corrected >= config_.min_angle))){
          float distance = tmp.uint * calibration_.distance_resolution_m;

          // skip zero distance (invalid) points
          if (distance < 0.1){
            continue;
          }

          distance += corrections.dist_correction;
  
          float cos_vert_angle = corrections.cos_vert_correction;
          float sin_vert_angle = corrections.sin_vert_correction;
          float cos_rot_correction = corrections.cos_rot_correction;
          float sin_rot_correction = corrections.sin_rot_correction;
  
          // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
          // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
          float cos_rot_angle = 
            cos_rot_table_[azimuth_corrected] * cos_rot_correction + 
            sin_rot_table_[azimuth_corrected] * sin_rot_correction;
          float sin_rot_angle = 
            sin_rot_table_[azimuth_corrected] * cos_rot_correction - 
            cos_rot_table_[azimuth_corrected] * sin_rot_correction;
  
          float horiz_offset = corrections.horiz_offset_correction;
          float vert_offset = corrections.vert_offset_correction;
  
          // Compute the distance in the xy plane (w/o accounting for rotation)
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;
  
          // Calculate temporal X, use absolute value.
          float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
          // Calculate temporal Y, use absolute value
          float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
          if (xx < 0) xx=-xx;
          if (yy < 0) yy=-yy;
    
          // Get 2points calibration values,Linear interpolation to get distance
          // correction for X and Y, that means distance correction use
          // different value at different distance
          float distance_corr_x = 0;
          float distance_corr_y = 0;
          if (corrections.two_pt_correction_available) {
            distance_corr_x = 
              (corrections.dist_correction - corrections.dist_correction_x)
                * (xx - 2.4) / (25.04 - 2.4) 
              + corrections.dist_correction_x;
            distance_corr_x -= corrections.dist_correction;
            distance_corr_y = 
              (corrections.dist_correction - corrections.dist_correction_y)
                * (yy - 1.93) / (25.04 - 1.93)
              + corrections.dist_correction_y;
            distance_corr_y -= corrections.dist_correction;
          }
  
          float distance_x = distance + distance_corr_x;
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle ;
          ///the expression wiht '-' is proved to be better than the one with '+'
          x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
  
          float distance_y = distance + distance_corr_y;
          xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle ;
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
  
          // Using distance_y is not symmetric, but the velodyne manual
          // does this.
          /**the new term of 'vert_offset * cos_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          z = distance_y * sin_vert_angle + vert_offset*cos_vert_angle;
  
          /** Use standard ROS coordinate system (right-hand rule) */
          float x_coord = y;
          float y_coord = -x;
          float z_coord = z;
  
          /** Intensity Calculation */
  
          float min_intensity = corrections.min_intensity;
          float max_intensity = corrections.max_intensity;
    
          float tmp_2 = (1 - corrections.focal_distance / 13100);
          float focal_offset = 256 * tmp_2 * tmp_2;

          float focal_slope = corrections.focal_slope;
          intensity += focal_slope * (std::abs(focal_offset - 256 * 
            (1 - static_cast<float>(tmp.uint)/65535)*(1 - static_cast<float>(tmp.uint)/65535)));
          intensity = (intensity < min_intensity) ? min_intensity : intensity;
          intensity = (intensity > max_intensity) ? max_intensity : intensity;

          if (pointInRange(distance)) {
            // add point to cloud
            point.x = x_coord;
            if (upward){
              point.y = y_coord;
              point.z = z_coord;
            }
            else{
              point.y = -y_coord;
              point.z = -z_coord;
            }
            point.ring = corrections.laser_ring;
            point.azimuth = (azimuth_corrected / 100.0) * (M_PI / 180.0); // 100ths of degrees to rad
            point.distance = distance;
            point.intensity = intensity;
            if (timing_offsets.size())
              point.time = timing_offsets[block][j] + time_diff_start_to_this_packet;

            data.addPoint(point);
          }
        }
      }
    }
    data.finalize();
  }
  
  /** @brief convert raw VLP16 packet to point cloud
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
  void RawData::unpack_vlp16(const velodyne_msgs::VelodynePacket &pkt, DataContainerBase& data, const ros::Time& scan_begin_stamp)
  {
    velodyne_rawdata::VPoint point;
    
    float azimuth;
    float azimuth_diff;
    int raw_azimuth_diff;
    float last_azimuth_diff=0;
    float azimuth_corrected_f;
    int azimuth_corrected;
    float x, y, z;
    float intensity;

    float time_diff_start_to_this_packet = (pkt.stamp - scan_begin_stamp).toSec();
    // ROS_WARN_STREAM("time diff scan to packet: " << time_diff_start_to_this_packet);

    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];

    int block_jump = config_.dual_return_mode ? 2 : 1;
    for (int block = 0; block < BLOCKS_PER_PACKET; block += block_jump) {

      // ignore packets with mangled or otherwise different contents
      if (UPPER_BANK != raw->blocks[block].header) {
        // Do not flood the log with messages, only issue at most one
        // of these warnings per minute.
        ROS_WARN_STREAM_THROTTLE(1, "skipping invalid VLP-16 packet: block "
                                 << block << " header value is "
                                 << raw->blocks[block].header);
        return;                         // bad packet: skip the rest
      }

      // Calculate difference between current and next block's azimuth angle.
      azimuth = (float)(raw->blocks[block].rotation);
      if (block < (BLOCKS_PER_PACKET - block_jump)){
        raw_azimuth_diff = raw->blocks[block + block_jump].rotation - raw->blocks[block].rotation;
        azimuth_diff = (float)((36000 + raw_azimuth_diff)%36000);

        // some packets contain an angle overflow where azimuth_diff < 0 
        if(raw_azimuth_diff < 0)//raw->blocks[block+1].rotation - raw->blocks[block].rotation < 0)
        {
          ROS_INFO_STREAM_THROTTLE(60, "Packet containing angle overflow, first angle: " << raw->blocks[block].rotation << " second angle: " << raw->blocks[block + block_jump].rotation);
          // if last_azimuth_diff was not zero, we can assume that the velodyne's speed did not change very much and use the same difference
          if(last_azimuth_diff > 0){
            azimuth_diff = last_azimuth_diff;
          }
          // otherwise we are not able to use this data
          // TODO: we might just not use the second 16 firings
          else{
            continue;
          }
        }
        last_azimuth_diff = azimuth_diff;
      }else{
        azimuth_diff = last_azimuth_diff;
      }

      for (int firing=0, k=0; firing < VLP16_FIRINGS_PER_BLOCK; firing++){
        for (int dsr=0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k+=RAW_SCAN_SIZE){
          
          velodyne_pointcloud::LaserCorrection &corrections = 
            calibration_.laser_corrections[dsr];

          /** Position Calculation */
          union two_bytes tmp;
          tmp.bytes[0] = raw->blocks[block].data[k];
          tmp.bytes[1] = raw->blocks[block].data[k+1];
          intensity = raw->blocks[block].data[k+2];
          if (config_.dual_return_mode)
          {
            // in dual return mode, the packets are in the order of Last (even blocks), Strongest/Second strongest (odd blocks)
            // If the second data block's return has a greater intensity than the first (ie last != strongest)
            if (raw->blocks[block].data[k+2] < raw->blocks[block + 1].data[k+2])
            {
              intensity = raw->blocks[block + 1].data[k+2];
              tmp.bytes[0] = raw->blocks[block + 1].data[k];
              tmp.bytes[1] = raw->blocks[block + 1].data[k+1];
            }
          }

          float point_time;
          if (config_.dual_return_mode){
            point_time = timing_offsets[block][dsr] + time_diff_start_to_this_packet;
            /** correct for the laser rotation as a function of timing during the firings **/
            // azimuth_corrected_f = azimuth + (azimuth_diff * (( (dsr/2)*VLP16_DSR_TOFFSET) + (firing*VLP16_FIRING_TOFFSET)) / block_duration);
          }
          else{
            point_time = timing_offsets[block][firing * 16 + dsr] + time_diff_start_to_this_packet;
            /** correct for the laser rotation as a function of timing during the firings **/
            // azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr*VLP16_DSR_TOFFSET) + (firing*VLP16_FIRING_TOFFSET)) / block_duration);
          }

          // azimuth_corrected_f = azimuth + (azimuth_diff * time_offset);
          azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;
          
          /*condition added to avoid calculating points which are not
            in the interesting defined area (min_angle < area < max_angle)*/
          if ((azimuth_corrected >= config_.min_angle 
               && azimuth_corrected <= config_.max_angle 
               && config_.min_angle < config_.max_angle)
               ||(config_.min_angle > config_.max_angle 
               && (azimuth_corrected <= config_.max_angle 
               || azimuth_corrected >= config_.min_angle))){

            // convert polar coordinates to Euclidean XYZ
            float distance = tmp.uint * calibration_.distance_resolution_m;

            // skip zero distance (invalid) points

            if (distance < 0.1){
              continue;
            }

            distance += corrections.dist_correction;
            
            float cos_vert_angle = corrections.cos_vert_correction;
            float sin_vert_angle = corrections.sin_vert_correction;
            float cos_rot_correction = corrections.cos_rot_correction;
            float sin_rot_correction = corrections.sin_rot_correction;
    
            // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
            // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
            float cos_rot_angle = 
              cos_rot_table_[azimuth_corrected] * cos_rot_correction + 
              sin_rot_table_[azimuth_corrected] * sin_rot_correction;
            float sin_rot_angle = 
              sin_rot_table_[azimuth_corrected] * cos_rot_correction - 
              cos_rot_table_[azimuth_corrected] * sin_rot_correction;
    
            float horiz_offset = corrections.horiz_offset_correction;
            float vert_offset = corrections.vert_offset_correction;
    
            // Compute the distance in the xy plane (w/o accounting for rotation)
            /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;
    
            // Calculate temporal X, use absolute value.
            float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
            // Calculate temporal Y, use absolute value
            float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
            if (xx < 0) xx=-xx;
            if (yy < 0) yy=-yy;
      
            // Get 2points calibration values,Linear interpolation to get distance
            // correction for X and Y, that means distance correction use
            // different value at different distance
            float distance_corr_x = 0;
            float distance_corr_y = 0;
            if (corrections.two_pt_correction_available) {
              distance_corr_x = 
                (corrections.dist_correction - corrections.dist_correction_x)
                  * (xx - 2.4) / (25.04 - 2.4) 
                + corrections.dist_correction_x;
              distance_corr_x -= corrections.dist_correction;
              distance_corr_y = 
                (corrections.dist_correction - corrections.dist_correction_y)
                  * (yy - 1.93) / (25.04 - 1.93)
                + corrections.dist_correction_y;
              distance_corr_y -= corrections.dist_correction;
            }
    
            float distance_x = distance + distance_corr_x;
            /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle ;
            x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
    
            float distance_y = distance + distance_corr_y;
            /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle ;
            y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
    
            // Using distance_y is not symmetric, but the velodyne manual
            // does this.
            /**the new term of 'vert_offset * cos_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            z = distance_y * sin_vert_angle + vert_offset*cos_vert_angle;
    
            /** Use standard ROS coordinate system (right-hand rule) */
            float x_coord = y;
            float y_coord = -x;
            float z_coord = z;
    
            /** Intensity Calculation */
            float min_intensity = corrections.min_intensity;
            float max_intensity = corrections.max_intensity;
    
            float tmp_2 = (1 - corrections.focal_distance / 13100);
    
            float focal_offset = 256 * tmp_2 * tmp_2;
            float focal_slope = corrections.focal_slope;
            intensity += focal_slope * (std::abs(focal_offset - 256 * 
              (1 - tmp.uint/65535)*(1 - tmp.uint/65535)));
            intensity = (intensity < min_intensity) ? min_intensity : intensity;
            intensity = (intensity > max_intensity) ? max_intensity : intensity;

            // add point to cloud
            if (pointInRange(distance)){
              point.x = x_coord;
              if (upward){
                point.y = y_coord;
                point.z = z_coord;
              }
              else{
                point.y = -y_coord;
                point.z = -z_coord;
              }
              point.ring = corrections.laser_ring;
              point.azimuth = (azimuth_corrected / 100.0) * (M_PI / 180.0); // 100ths of deg to rad
              point.distance = distance;
              point.intensity = intensity;
              if (timing_offsets.size())
                point.time = timing_offsets[block][firing * 16 + dsr] + time_diff_start_to_this_packet;

              data.addPoint(point);
            }
          }
        }
      }
    }
    data.finalize();
  }
} // namespace velodyne_rawdata
