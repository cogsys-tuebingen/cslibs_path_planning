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
#include <memory>
#include "../geometry/line.h"
#include "../geometry/shape.h"


namespace path_geom {

/**
 * circle/arc class representation
 * teh arc is defined by center, radius, direction, start_angle and arc angle
 */

class Circle : public Shape
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Circle(const Eigen::Vector2d& center, double radius, int arc_direction=path_geom::ARC_LEFT);

    /**
     * @brief creates an arc starting at given start pose with pose orientation tangenting the arc
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

    virtual bool selectStartPoint(const Eigen::Vector2d& start, double tol=path_geom::DIST_EPS) {
        if (fabs((start-center_).norm()-radius_)<tol) {
            setStartAngle(start);
            return true;
        } else {
            return false;
        }
    }

    virtual bool selectEndPoint(const Eigen::Vector2d& end, double tol=path_geom::DIST_EPS) {
        if (fabs((end-center_).norm()-radius_)<tol) {
            setEndAngle(end);
            return true;
        } else {
            return false;
        }
    }

    virtual double distanceTo(const Vector2d &point) const;
    virtual Eigen::Vector2d nearestPointTo(const Eigen::Vector2d& p) const;

    /**
     * @brief sets the start angle of the arc, end angle is fix, thus arc_angle is changed
     * @param angle
     */
    void setStartAngle(double angle);

    /**
     * @brief sets the end angle of the arc, start angle is fix, thus arc_angle is changed
     * @param angle
     */
    void setEndAngle(double angle);

    /**
     * @brief sets the arc angle, start angel is fix, thus end_angel is changed
     * @param arc_angle
     */
    void setArcAngle(double arc_angle);

    void intersect (const Line& line, std::vector<Eigen::Vector2d>& ipoints, double tol=path_geom::DIST_EPS);
    void intersect (const Circle& circle, std::vector<Eigen::Vector2d>& ipoints, double tol=path_geom::DIST_EPS);


    Eigen::Vector2d center() const {return center_;}
    double radius() const {return radius_;}
    double startAngle() const {return start_angle_;}
    double endAngle() const;
    void setStartAngle(const Eigen::Vector2d& p);
    void setEndAngle(const Eigen::Vector2d& p);
    virtual Eigen::Vector2d startPoint() const;
    virtual Eigen::Vector2d endPoint() const;
    void setArc(double start_angle, double arc_angle, int arc_direction);

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


    /**
     * @brief compareArcAngle comparison function object which returns ?true if a is less than b.
     * @param c1
     * @param c2
     * @return
     */
    static bool compareArcAngle(const Circle& a, const Circle& b);

    static bool compareArcAnglePtr(const shared_ptr<Circle>& a, const shared_ptr<Circle>& b);

protected:
    Eigen::Vector2d center_;
    double radius_;
    int arc_direction_;
    double arc_angle_ = 2*M_PI;
    double start_angle_=0.0;

    double getAngleOfPoint(const Eigen::Vector2d& p) const;
};





}
#endif // CIRCLE2D_H
