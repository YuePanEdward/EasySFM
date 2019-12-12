#ifndef _INCLUDE_VIEWER_H_
#define _INCLUDE_VIEWER_H_

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <fstream>

#include "utility.h"

namespace p3dv
{
class MapViewer
{
public:
  bool displaySFM(std::vector<frame_t> &frames, std::vector<bool> &frames_to_process,
                  pointcloud_sparse_t &sparse_pointcloud, std::string viewer_name = "SfM viewer",
                  bool black_background = 1, bool render_point_as_sphere = 0);

  bool displaySFM_on_fly(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                         std::vector<frame_t> &frames, std::vector<bool> &frames_to_process,
                         pointcloud_sparse_t &sparse_pointcloud, double relative_depth = 5.0,
                         int display_time_ms = 1000, bool render_point_as_sphere = 0);

private:
  bool is_frist_frame_ = 1;
  float approximate_scale_ = 1.0;
  bool render_point_as_sphere_ = 0;
};
} // namespace p3dv
#endif //_INCLUDE_VIEWER_H_