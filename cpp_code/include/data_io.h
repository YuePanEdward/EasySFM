#ifndef _INCLUDE_DATA_IO_H_
#define _INCLUDE_DATA_IO_H_


#include "utility.h"

namespace p3dv
{
class DataIO
{
   public:

    bool ImportImages(frame_t &cur_frame, bool show=true);

};
} // namespace p3dv
#endif