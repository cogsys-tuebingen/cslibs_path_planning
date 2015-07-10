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
#include "../geometry/line.h"
#include "../geometry/circle.h"
using namespace path_geom;

Circle::Circle(const Eigen::Vector2d& center, double radius, int direction)
    :center_(center), radius_(fabs(radius)), dir_(direction/abs(direction))
{
    if (dir_==Shape::BACKWARD) {
        start_angle_=2*M_PI;
        end_angle_=0.0;
    } else {
        start_angle_=0.0;
        end_angle_=2*M_PI;
    }
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


double Circle::getArcLength() const
{
    return getArcAngle()*radius_;
}


double Circle::getArcAngle() const
{
    double arc_angle;
    if (dir_==Shape::FORWARD) {
        arc_angle = end_angle_-start_angle_;
    } else {
        arc_angle = start_angle_ -end_angle_;
    }

    if (arc_angle<0)
        arc_angle+=2*M_PI;
    if (arc_angle>2*M_PI)
        arc_angle-=2*M_PI;
    return arc_angle;
}

void Circle::toPoints(double resolution, std::vector<Eigen::Vector2d> &points)
{


    double arc_length = getArcLength() ;
    double arc_angle = getArcAngle();
    int steps = arc_length/resolution;
    double dtheta = dir_*arc_angle/steps;
    points.resize(steps);
    double theta = start_angle_;
    for (int i=0;i<steps;++i) {
        points[i].x() = radius_*cos(theta);
        points[i].y() = radius_*sin(theta);
        theta+=dtheta;
    }
}


double Circle::getAngleOfPoint(const Eigen::Vector2d &p) const
{
    Eigen::Vector2d cp = p-center_;
    double angle = atan2(cp.y(),cp.x());
    if (angle<0 ) angle+=2*M_PI;
    return angle;
}


void Circle::setStartAngle(const Eigen::Vector2d &p)
{
    setStartAngle(getAngleOfPoint(p));
}


void Circle::setEndAngle(const Eigen::Vector2d &p)
{
    setEndAngle(getAngleOfPoint(p));
}


Eigen::Vector2d Circle::startPoint()
{

    return Eigen::Vector2d(center_.x()+radius_*cos(start_angle_),
                      center_.y()+radius_*sin(start_angle_));
}


Eigen::Vector2d Circle::endPoint()
{
    return Eigen::Vector2d(center_.x()+radius_*cos(end_angle_),
                      center_.y()+radius_*sin(end_angle_));
}


bool Circle::isPointOnArc(const Eigen::Vector2d &p, double tol) const
{
    double d = (center_-p).norm();
    if (fabs(d-radius_)>tol) {
        // point not on circle
        return false;
    }
    double p_angle = getAngleOfPoint(p);
    if (dir_==Shape::FORWARD) {
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
