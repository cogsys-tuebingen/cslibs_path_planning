/**


    @year   2015
    @author Karsten Bohlmann
    @email  bohlmann@gmail.com
    @file   shape.h


*/

#ifndef SHAPE_H
#define SHAPE_H
#include "utils_general/MathHelper.h"
#include <Eigen/Core>
#include<Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2d)


// ***todo templatisieren http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
namespace path_geom {

    const double DIST_EPS   = 1e-5;
    const double ANGLE_EPS  = 0.01*M_PI/180.0;

    const int    ARC_LEFT   = +1; // arc/circle in mathematically positive direction
    const int    ARC_RIGHT   = -1; // arc/circle in mathematically negative direction
    const int    BACKWARD=-1;
    const int    FORWARD = 1;
class PathPose
{
public:

    PathPose() :
        pos_(0.0,0.0){}
    PathPose(double x, double y, double theta)
        :pos_(x,y),theta_(theta),weight_(0.0) {
    }

    PathPose & operator= (const PathPose& src) {
        pos_=src.pos_;
        theta_=src.theta_;
        weight_=src.weight_;
        return (*this);
    }

    PathPose & assign (const Eigen::Vector2d pos, double theta) {
        pos_=pos;
        theta_ = theta;
        return (*this);
    }

    bool isIdentical(const PathPose& other, double dist_tol=path_geom::DIST_EPS,
                     double angle_tol=path_geom::ANGLE_EPS) {
        return ((pos_-other.pos_).norm()<dist_tol &&
              MathHelper::AngleDelta( theta_, other.theta_)<angle_tol);
    }

    bool isIdentical(const Eigen::Vector2d& other, double dist_tol=path_geom::DIST_EPS) {
        return ((pos_-other).norm()<dist_tol);
    }

    Eigen::Vector2d pos_;
    double theta_ = 0.0;
    double weight_ = 0.0;


};

class Shape
{
public:

    virtual void toPoints(double resolution, std::vector<Eigen::Vector2d>& points) = 0;
    virtual void toPoses(double resolution, std::vector<PathPose>& res_poses, int move_direction,
                         bool with_end_pose) = 0;

};

}
#endif // SHAPE_H
