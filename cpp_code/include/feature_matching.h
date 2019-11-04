#ifndef _INCLUDE_FEATURE_MATCHING_H_
#define _INCLUDE_FEATURE_MATCHING_H_

#include "utility.h"

namespace p3dv
{
class FeatureMatching
{
public:
  //FeatureMatching();

  bool detectFeaturesORB(frame_t &cur_frame, bool show = true);

  bool detectFeaturesSURF(frame_t &cur_frame, int minHessian = 400, bool show = true);

  bool matchFeaturesORB(frame_t &cur_frame_1, frame_t &cur_frame_2, std::vector<cv::DMatch> &matches,
                        double ratio_thre = 0.7, bool show = true);

  bool matchFeaturesSURF(frame_t &cur_frame_1, frame_t &cur_frame_2, std::vector<cv::DMatch> &matches,
                         double ratio_thre = 0.6 , bool show = true);
};
} // namespace p3dv
#endif