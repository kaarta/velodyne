

#ifndef __DATACONTAINERBASE_H
#define __DATACONTAINERBASE_H

#include <ros/ros.h>
#include <velodyne_pointcloud/point_types.h>

namespace velodyne_rawdata 
{
  class DataContainerBase 
  {
  public:
    virtual void addPoint(const Kaarta::KaartaSensorPoint& point) = 0;
    virtual void finalize() = 0;
  };
}
#endif //__DATACONTAINERBASE_H

