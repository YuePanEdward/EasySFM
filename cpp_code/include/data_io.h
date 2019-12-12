#ifndef _INCLUDE_DATA_IO_H_
#define _INCLUDE_DATA_IO_H_

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include "utility.h"

namespace p3dv
{
class DataIO
{
public:
  bool importImages(frame_t &cur_frame, bool show = true);

  bool importCalib(const std::string &fileName, Eigen::Matrix3f &K_mat);

  bool importDistort(const std::string &fileName, cv::Mat &distort_coeff);

  bool writePcdFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud);

  bool writePlyFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud);

  bool writeTxtFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud);

};
} // namespace p3dv
#endif