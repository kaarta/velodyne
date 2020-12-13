/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011, 2012 Austin Robot Technology
 *  Modified 2018 Kaarta - Shawn Hanna
 *  License: Modified BSD Software License Agreement
 *
 *  $Id: data_base.h 1554 2011-06-14 22:11:17Z jack.oquin $
 */

/** \file
 *
 *  Point Cloud Library point structures for Velodyne data.
 *
 *  @author Jesse Vera
 *  @author Jack O'Quin
 *  @author Piyush Khandelwal
 */

#ifndef __VELODYNE_POINTCLOUD_POINT_TYPES_H
#define __VELODYNE_POINTCLOUD_POINT_TYPES_H

#include <pcl/point_types.h>
#include <clay_lib/point_definition.h>

namespace velodyne_pointcloud
{
  /** Euclidean Velodyne coordinate, including intensity and ring number. */
  struct PointXYZIR
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

  /** Euclidean Velodyne coordinate, including intensity and ring number. */
  struct RawSensorPoint
  {
    static const uint8_t LAST = 1;
    static const uint8_t SECOND_STRONGEST = 2;
    static const uint8_t STRONGEST = 4;

    RawSensorPoint() : return_number(0){}
    uint8_t  intensity;
    float    azimuth;                   ///< laser azimuth in rad
    uint16_t distance;                  ///< laser distance reading
    uint16_t ring;                      ///< laser ring number
    uint8_t  laser_num;                 ///< laser ring number
    double   time;                      ///< point time reading (seconds since scan start)
    uint8_t  return_number;             ///< return number, last = 1, second strongest = 2, strongest = 4, strongest & last (bitwise) = 5

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

}; // namespace velodyne_pointcloud


POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::RawSensorPoint,
                                  (uint8_t, intensity, intensity)
                                  (float, azimuth, azimuth)
                                  (uint16_t, distance, distance)
                                  (uint16_t, ring, ring)
                                  (double, time, time)
                                  (uint8_t, return_number, return_number)
                                  );

#endif // __VELODYNE_POINTCLOUD_POINT_TYPES_H

