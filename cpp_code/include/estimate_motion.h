#ifndef _INCLUDE_ESTIMATE_MOTION_H_
#define _INCLUDE_ESTIMATE_MOTION_H_

#include "utility.h"

namespace p3dv
{
class MotionEstimator
{
public:
  bool estimateE8Points(std::vector<cv::KeyPoint> &keypoints1,
                        std::vector<cv::KeyPoint> &keypoints2,
                        std::vector<cv::DMatch> &matches,
                        Eigen::Matrix3f &K,
                        Eigen::Matrix4f &T);

  bool estimate2D2D_E5P_RANSAC(frame_t &cur_frame_1, frame_t &cur_frame_2,
                               std::vector<cv::DMatch> &matches, std::vector<cv::DMatch> &inlier_matches,
                               Eigen::Matrix4f &T,
                               double ransac_prob = 0.99, double ransac_thre = 1.0, bool show = false);

  bool doTriangulation(frame_t &cur_frame_1, frame_t &cur_frame_2,
                       const std::vector<cv::DMatch> &matches,
                       pointcloud_sparse_t &sparse_pointcloud, bool show = false);

  bool getDepthFast(frame_t &cur_frame_1, frame_t &cur_frame_2, Eigen::Matrix4f &T_21,
                    const std::vector<cv::DMatch> &matches, double &appro_depth, int random_rate = 20);

  bool estimate2D3D_P3P_RANSAC(frame_t &cur_frame, pointcloud_sparse_t &cur_map_3d,
                               double ransac_thre = 2.5, int iterationsCount = 50000, double ransac_prob = 0.99, bool show = false);

  bool transformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out,
                      Eigen::Matrix4f &trans);

  bool doUnDistort(frame_t &cur_frame, cv::Mat distort_coeff);

  bool outlierFilter(pointcloud_sparse_t &sparse_pointcloud, int MeanK = 40, double std = 2.5);

private:
  //transform pixel to camera space coordiante system
  cv::Point2f pixel2cam(const cv::Point2f &p, const cv::Mat &K)
  {
    cv::Point2f cam_pt((p.x - K.at<float>(0, 2)) / K.at<float>(0, 0), (p.y - K.at<float>(1, 2)) / K.at<float>(1, 1));
    //std::cout<<cam_pt<<std::endl;
    return cam_pt;
  }
};
} // namespace p3dv

#endif