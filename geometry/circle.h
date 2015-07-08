/**
    Copyright (C) 2015 Karsten Bohlmann E&K AUTOMATION GmbH
    All rights reserved.

    @year   2015
    @author Karsten Bohlmann
    @email  bohlmann@gmail.com
    @file   circle2d.h


*/

#ifndef CIRCLE2D_H
#define CIRCLE2D_H
#include "../geometry/shape.h"


namespace path_geom {
class Circle : public Shape
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Circle(const Eigen::Vector2d& center, double radius, Shape::Direction dir);
    virtual void toPoints(double resolution, std::vector<Eigen::Vector2d>& points);

    Eigen::Vector2d center() const {return center_;}
    double radius() const {return radius_;}
protected:
    Eigen::Vector2d center_;
    double radius_;
    Shape::Direction dir_;
};
}
#endif // CIRCLE2D_H
