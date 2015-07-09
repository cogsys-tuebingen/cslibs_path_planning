/**


    @year   2015
    @author Karsten Bohlmann
    @email  bohlmann@gmail.com
    @file   shape.h


*/

#ifndef SHAPE_H
#define SHAPE_H
#include <Eigen/Core>
#include<Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2d)


// ***todo templatisieren http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
namespace path_geom {


class Shape
{
public:
    static const int BACKWARD=-1;
    static const int FORWARD = 1;
    static constexpr double DEFAULT_TOLERANCE = 1e-5;
    virtual void toPoints(double resolution, std::vector<Eigen::Vector2d>& points) = 0;

};

}
#endif // SHAPE_H
