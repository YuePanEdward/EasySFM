#ifndef _INCLUDE_DATA_IO_H_
#define _INCLUDE_DATA_IO_H_


#include "utility.h"

namespace p3dv
{
class DataIO
{
   public:

    bool importImages(frame_t &cur_frame, bool show=true);
    
    bool importCalib(const std::string &fileName, Eigen::Matrix3f &K_mat);

};
} // namespace p3dv
#endif