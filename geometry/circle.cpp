/**
    Copyright (C) 2015 Karsten Bohlmann E&K AUTOMATION GmbH
    All rights reserved.

    @year   2015
    @author Karsten Bohlmann
    @email  bohlmann@gmail.com
    @file   circle2d.cpp


*/
#include <cmath>
#include <iostream>
#include <utils_general/MathHelper.h>
#include "../geometry/line.h"
#include "../geometry/circle.h"
using namespace path_geom;
using namespace Eigen;

Circle::Circle(const Vector2d& center, double radius, int direction)
    :center_(center), radius_(fabs(radius)), arc_direction_(direction/abs(direction))
{
    if (arc_direction_==path_geom::ARC_RIGHT) {
        start_angle_=2*M_PI;
        end_angle_=0.0;
    } else {
        start_angle_=0.0;
        end_angle_=2*M_PI;
    }
}


path_geom::Circle Circle::createArcFrom(const PathPose &start, double radius, double arc_angle,
                                    int arc_direction)
{
    double start_angle, end_angle;
    if (arc_direction==path_geom::ARC_LEFT) {
        start_angle=start.theta_- M_PI/2.0;
        end_angle = start_angle+arc_angle;
    } else {
        start_angle=start.theta_+ M_PI/2.0;
        end_angle = start_angle-arc_angle;
    }
    Vector2d cs(radius*cos(start_angle),radius*sin(start_angle));

    Vector2d center=start.pos_-cs;
    Circle circle(center,radius,arc_direction);
    circle.setArc(start_angle,end_angle,arc_direction);
    return circle;
}


path_geom::Circle Circle::createArcTo(const PathPose &end, double radius, double arc_angle,
                                    int arc_direction)
{
    double start_angle, end_angle;
    if (arc_direction==path_geom::ARC_LEFT) {
        end_angle=end.theta_- M_PI/2.0;
        start_angle = end_angle-arc_angle;
    } else {
        end_angle=end.theta_+ M_PI/2.0;
        start_angle = end_angle+arc_angle;
    }
    Vector2d cs(radius*cos(end_angle),radius*sin(end_angle));

    Vector2d center=end.pos_-cs;
    Circle circle(center,radius,arc_direction);
    circle.setArc(start_angle,end_angle,arc_direction);
    return circle;
}



void Circle::setStartAngle(double angle)
{

    while (angle > 2*M_PI)
      angle -= 2.0 * M_PI;
    while (angle < 0.0)
      angle += 2.0 * M_PI;
    start_angle_ = angle;
}


void Circle::setEndAngle(double angle)
{

    while (angle > 2*M_PI)
      angle -= 2.0 * M_PI;
    while (angle < 0.0)
      angle += 2.0 * M_PI;
    end_angle_ = angle;
}


void Circle::setArc(double start_angle, double end_angle, int arc_direction)
{
    setStartAngle(start_angle);
    setEndAngle(end_angle);
    if (arc_direction>=0) {
        arc_direction_=path_geom::ARC_LEFT;
    } else {
        arc_direction_=path_geom::ARC_RIGHT;
    }
}


double Circle::getArcLength() const
{
    return getArcAngle()*radius_;
}


double Circle::getArcAngle() const
{
    double arc_angle;
    if (arc_direction_==path_geom::ARC_LEFT) {
        arc_angle = end_angle_-start_angle_;
    } else {
        arc_angle = start_angle_ -end_angle_;
    }
    while (arc_angle > 2*M_PI)
      arc_angle -= 2.0 * M_PI;
    while (arc_angle < 0.0)
      arc_angle += 2.0 * M_PI;
    return arc_angle;
}


void Circle::toPoints(double resolution, std::vector<Vector2d> &points)
{
    double arc_length = getArcLength() ;
    double arc_angle = getArcAngle();
    int steps = abs(arc_length/resolution);
    double dphi = arc_angle/steps;
    points.resize(steps);
    double phi = start_angle_;
    for (int i=0;i<steps;++i) {
        points[i].x() = radius_*cos(phi);
        points[i].y() = radius_*sin(phi);
        phi+=dphi;

    }
}


void Circle::toPoses(double resolution, std::vector<PathPose> &poses, int move_direction,
                     bool with_start_pose)
{
    if (fabs(resolution)<1e-10) {
        poses.clear();
        return;
    }
    double arc_length = getArcLength() ;
    double arc_angle = getArcAngle();
    int steps = abs(arc_length/resolution);
    double dphi = arc_angle/steps;
    double phi;
    // the delta phi depends on the movement direction
    if (move_direction==path_geom::FORWARD) {
        phi = start_angle_;
    } else {
        phi = end_angle_;
        dphi = -1.0*dphi;
    }
    // the delta phi depends on the arc direction - whether it is a left or right arc
    if (arc_direction_==path_geom::ARC_RIGHT) {
        dphi = -1.0*dphi;
    }
    if (with_start_pose) {
        poses.resize(steps+1);
    } else {
        // omit the first point
        poses.resize(steps);
        phi+=dphi;
    }

    for (int i=0;i<steps;++i) {
        poses[i].pos_ = center_+ Vector2d(cos(phi),sin(phi))*radius_;
        poses[i].theta_ = MathHelper::AngleClamp(phi+arc_direction_*M_PI/2.0);
        phi+=dphi;
    }
    if (with_start_pose) {
        poses[steps].pos_ = center_+ radius_*Vector2d(cos(phi),sin(phi));
        poses[steps].theta_ = MathHelper::AngleClamp(phi+arc_direction_*M_PI/2.0);
    }
}


double Circle::getAngleOfPoint(const Vector2d &p) const
{
    Vector2d cp = p-center_;
    double angle = atan2(cp.y(),cp.x());
    if (angle<0 ) angle+=2*M_PI;
    return angle;
}


void Circle::setStartAngle(const Vector2d &p)
{
    setStartAngle(getAngleOfPoint(p));
}


void Circle::setEndAngle(const Vector2d &p)
{
    setEndAngle(getAngleOfPoint(p));
}


Vector2d Circle::startPoint()
{

    return Vector2d(center_.x()+radius_*cos(start_angle_),
                      center_.y()+radius_*sin(start_angle_));
}


Vector2d Circle::endPoint()
{
    return Vector2d(center_.x()+radius_*cos(end_angle_),
                      center_.y()+radius_*sin(end_angle_));
}


bool Circle::isPointOnArc(const Vector2d &p, double tol) const
{
    double d = (center_-p).norm();
    if (fabs(d-radius_)>tol) {
        // point not on circle
        return false;
    }
    double p_angle = getAngleOfPoint(p);
    if (arc_direction_==path_geom::ARC_LEFT) {
        if (end_angle_>=start_angle_) {
            if (p_angle>=start_angle_ && p_angle<=end_angle_) {
                return true;
            }
        } else {
            if (p_angle>=start_angle_ || p_angle<=end_angle_) {
                return true;
            }
        }
    } else {
        if (start_angle_>=end_angle_) {
            if (p_angle>=end_angle_ && p_angle<=start_angle_) {
                return true;
            }
        } else {
            if (p_angle>=end_angle_ || p_angle<=start_angle_) {
                return true;
            }
        }
    }
    return false;
}
