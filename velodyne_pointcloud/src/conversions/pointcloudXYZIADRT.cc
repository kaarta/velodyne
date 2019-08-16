
#include <velodyne_pointcloud/pointcloudXYZIADRT.h>

namespace velodyne_pointcloud 
{
  void PointcloudXYZIADRT::addPoint(const velodyne_rawdata::VPoint& point)
  {
    // convert polar coordinates to Euclidean XYZ
    // velodyne_rawdata::VPoint point;
    // point.x = x;
    // point.y = y;
    // point.z = z;
    // point.ring = ring;
    // point.azimuth = azimuth;
    // point.distance = distance;
    // point.intensity = intensity;
    // point.time = time;

    // append this point to the cloud
    pc->points.push_back(point);
    // ++pc->width; // do this in the finalize step. MUST DO!
  }
  void PointcloudXYZIADRT::finalize(){
    pc->width = pc->points.size();
  }
}

