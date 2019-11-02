#ifndef _INCLUDE_FEATURE_MATCHING_H_
#define _INCLUDE_FEATURE_MATCHING_H_


#include "utility.h"


namespace p3dv
{
class FeatureMatching
{
  public:
    //FeatureMatching();
    
    bool DetectFeatures(frame_t &cur_frame,  bool show);
    
    bool MatchFeatures(frame_t &cur_frame_1, frame_t &cur_frame_2, std::vector<cv::DMatch> &matches, bool show);
    
    bool ImportImages(frame_t &cur_frame, bool show);


};
} // namespace p3dv
#endif