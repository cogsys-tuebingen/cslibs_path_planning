/**

    @year   2015
    @author Karsten Bohlmann
    @email  bohlmann@gmail.com
    @file   shape.cpp


*/

#include <cslibs_path_planning/geometry/shape.h>
#include <cslibs_path_planning/geometry/line.h>
#include <cslibs_path_planning/geometry/circle.h>

using namespace std;
using namespace Eigen;
using namespace path_geom;

ostream& operator<<(ostream& os, const PathPose& pp)
{
    os << "Pose (" << pp.pos_.x()<<","<<pp.pos_.y()<<") phi "<<pp.theta_<<" ";
    return os;
}




shared_ptr<Shape> Shape::deepCopy(const shared_ptr<Shape> &src)
{
    auto first_line = std::dynamic_pointer_cast<Line>(src);
    auto first_circle = std::dynamic_pointer_cast<Circle>(src);
    if (first_line) {
        return make_aligned<Line>(*first_line);
    } else if (first_circle) {
        return make_aligned<Circle>(*first_circle);
    } else {
        shared_ptr<Shape> ptr;
        return ptr;
    }
}
