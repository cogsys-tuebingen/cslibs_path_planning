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
#include <cmath>
#include "../geometry/shape.h"


namespace path_geom {
class Circle : public Shape
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Circle(const Eigen::Vector2d& center, double radius, int direction);
    virtual void toPoints(double resolution, std::vector<Eigen::Vector2d>& points);

    void setStartAngle(double angle);
    void setEndAngle(double angle);
    Eigen::Vector2d center() const {return center_;}
    double radius() const {return radius_;}
    double startAngle() {return start_angle_;}
    double endAngle() {return end_angle_;}
    void setStartAngle(const Eigen::Vector2d& p);
    void setEndAngle(const Eigen::Vector2d& p);
    Eigen::Vector2d startPoint();
    Eigen::Vector2d endPoint();
    int direction() const {return dir_;}

    double getArcLength() const;

    double getArcAngle() const;

    /**
     * @brief isInside
     * @param other
     * @return true if this circle is fully inside other circle
     */
    bool isInside(const path_geom::Circle& other);

    bool isPointOnArc(const Eigen::Vector2d& p, double tol = Shape::DEFAULT_TOLERANCE) const;

protected:
    Eigen::Vector2d center_;
    double radius_;
    int dir_;
    double start_angle_=0.0, end_angle_=2*M_PI;

    double getAngleOfPoint(const Eigen::Vector2d& p);
};
}
#endif // CIRCLE2D_H
