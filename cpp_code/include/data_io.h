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

    bool writePcdFile(const std::string &fileName, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud);

    bool displaySFM(std::vector<frame_t> &frames, std::vector<bool> &frames_to_process, pointcloud_sparse_t &sparse_pointcloud, bool black_background = 1);
};
} // namespace p3dv
#endif