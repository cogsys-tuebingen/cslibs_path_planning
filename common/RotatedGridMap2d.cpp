/// HEADER
#include "RotatedGridMap2d.h"

lib_path::RotatedGridMap2d::RotatedGridMap2d( const unsigned int w, const unsigned int h, const double yaw, const double r )
    : SimpleGridMap2d(w, h, r), yaw_(yaw)
{
}
