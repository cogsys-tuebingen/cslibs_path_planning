/**


    @year   2015
    @author Karsten Bohlmann
    @email  bohlmann@gmail.com
    @file   shape.h


*/

#ifndef SHAPE_H
#define SHAPE_H
#include <Eigen/Core>
#include <vector>



// ***todo templatisieren http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
namespace path_geom {


class Shape
{
public:

    enum Direction {
        FORWARD,
        BACKWARD
    };

    virtual void toPoints(double resolution, std::vector<Eigen::Vector2d>& points) = 0;

};

}
#endif // SHAPE_H
