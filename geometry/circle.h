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
    Circle(const Eigen::Vector2d& center, double radius, int arc_direction=path_geom::ARC_LEFT);

    /**
     * @brief creates an arc starting at given start pose
     * @param start
     * @param radius
     * @param arc_angle
     * @param arc_direction
     */
    static Circle createArcFrom (const path_geom::PathPose& start, double radius, double arc_angle, int arc_direction=path_geom::ARC_LEFT );
    static Circle createArcTo (const path_geom::PathPose& end, double radius, double arc_angle, int arc_direction=path_geom::ARC_LEFT   );

    virtual void toPoints(double resolution, std::vector<Eigen::Vector2d>& points);
    virtual void toPoses(double resolution, std::vector<path_geom::PathPose>& poses,
                         int move_direction, bool with_end_pose=true);

    void setStartAngle(double angle);
    void setEndAngle(double angle);
    Eigen::Vector2d center() const {return center_;}
    double radius() const {return radius_;}
    double startAngle() {return start_angle_;}
    double endAngle() {return end_angle_;}
    void setStartAngle(const Eigen::Vector2d& p);
    void setEndAngle(const Eigen::Vector2d& p);

    void setArc(double start_angle, double end_angle, int arc_direction);

    Eigen::Vector2d startPoint();
    Eigen::Vector2d endPoint();
    int direction() const {return arc_direction_;}

    /**
     * @brief return length of arc, value is negative for an arc in negative direction
     * @return
     */
    double getArcLength() const;

    /**
     * @brief returns angle covered by arc, value is negative if arc goes in mathematically negative direction
     * @return value in the range [-2*PI, +2*PI]
     */
    double getArcAngle() const;

    /**
     * @brief isInside
     * @param other
     * @return true if this circle is fully inside other circle
     */
    bool isInside(const path_geom::Circle& other);

    bool isPointOnArc(const Eigen::Vector2d& p, double tol = path_geom::DIST_EPS) const;

protected:
    Eigen::Vector2d center_;
    double radius_;
    int arc_direction_;
    double start_angle_=0.0, end_angle_=2*M_PI;

    double getAngleOfPoint(const Eigen::Vector2d& p) const;
};
}
#endif // CIRCLE2D_H
