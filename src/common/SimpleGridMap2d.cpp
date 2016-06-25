/**
 * @file SimpleGridMap2d.cpp
 * @date Jan 2012
 * @author marks
 */

// Project
#include <utils_path/common/SimpleGridMap2d.h>

///////////////////////////////////////////////////////////////////////////////
// class SimpleGridMap2d
///////////////////////////////////////////////////////////////////////////////

lib_path::SimpleGridMap2d::SimpleGridMap2d( const unsigned int w, const unsigned int h, const double r )
: width_( w ), height_( h ), res_( r ),  origin_( 0, 0 ),
  lower_thres_( 50 ), upper_thres_( 200 ), no_information_(-1)
{
    data_.resize( width_*height_ );
}
