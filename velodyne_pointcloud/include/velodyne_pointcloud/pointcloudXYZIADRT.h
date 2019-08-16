#pragma once

#include <velodyne_pointcloud/rawdata.h>

namespace velodyne_pointcloud 
{
  class PointcloudXYZIADRT : public velodyne_rawdata::DataContainerBase 
  {
  public:
    velodyne_rawdata::VPointCloud::Ptr pc;

    PointcloudXYZIADRT() : pc(new velodyne_rawdata::VPointCloud) {}
  
    virtual void addPoint(const velodyne_rawdata::VPoint& point);
    virtual void finalize();
  };
}
