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

  bool estimateE5PRANSAC(frame_t &cur_frame_1, frame_t &cur_frame_2,
                         std::vector<cv::DMatch> &matches, std::vector<cv::DMatch> &inlier_matches,
                         Eigen::Matrix3f &K, Eigen::Matrix4f &T,
                         double ransac_prob = 0.99, double ransac_thre = 1.0, bool show = true);

  void doTriangulation(frame_t &cur_frame_1, frame_t &cur_frame_2,
                       const std::vector<cv::DMatch> &matches,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr &sparse_pointcloud);

private:
  cv::Point2f pixel2cam(const cv::Point2f &p, const cv::Mat &K)
  {
    return cv::Point2f(
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
  }
};
} // namespace p3dv

#endif