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
#include "circle.h"
using namespace path_geom;

Circle::Circle(const Eigen::Vector2d& center, double radius, Shape::Direction dir)
    :center_(center), radius_(fabs(radius)), dir_(dir)
{
    std::cout << "cx "<<center_.x() << " cy "<< center_.y() << std::endl;

}


void Circle::toPoints(double resolution, std::vector<Eigen::Vector2d> &points)
{
    double arc_length = 2*M_PI*radius_;
    int steps = arc_length/resolution;
    double dtheta = 2*M_PI/steps;
    points.resize(steps);
    double theta = 0;
    for (int i=0;i<steps;++i) {
        points[i].x() = radius_*cos(theta);
        points[i].y() = radius_*sin(theta);
        theta+=dtheta;
    }
}
