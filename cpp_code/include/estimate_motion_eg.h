#ifndef _INCLUDE_ESTIMATE_MOTION_EG_H_
#define _INCLUDE_ESTIMATE_MOTION_EG_H_

#include "utility.h"

namespace p3dv
{
class EpipolarEstimator
{
  public:
    bool estimateE8Points(std::vector<cv::KeyPoint> &keypoints1,
                          std::vector<cv::KeyPoint> &keypoints2,
                          std::vector<cv::DMatch> &matches,
                          Eigen::Matrix3f &K,
                          Eigen::Matrix4f &T);

    bool estimateE5Points(std::vector<cv::KeyPoint> &keypoints1,
                          std::vector<cv::KeyPoint> &keypoints2,
                          std::vector<cv::DMatch> &matches,
                          Eigen::Matrix3f &K,
                          Eigen::Matrix4f &T);
};
} // namespace p3dv

#endif