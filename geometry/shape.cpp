/**

    @year   2015
    @author Karsten Bohlmann
    @email  bohlmann@gmail.com
    @file   shape.cpp


*/

#include "shape.h"

using namespace Eigen;
using namespace path_geom;




ostream& operator<<(ostream& os, const PathPose& pp)
{
    os << "Pose (" << pp.pos_.x()<<","<<pp.pos_.y()<<") phi "<<pp.theta_<<" ";
    return os;
}



Shape::Shape()
{
}
