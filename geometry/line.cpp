/**
    Copyright (C) 2015 Karsten Bohlmann E&K AUTOMATION GmbH
    All rights reserved.

    @year   2015
    @author Karsten Bohlmann
    @email  bohlmann@gmail.com
    @file   line.cpp


*/

#include "../geometry/line.h"
using namespace path_geom;
using namespace Eigen;
Line::Line(const Eigen::Vector2d& start, const Eigen::Vector2d& end, Shape::Direction dir)
    :start_(start), end_(end), dir_(dir)
{

}

path_geom::Line Line::parallel(const Line &line, double dist)
{
    Vector2d ds= (line.end_-line.start_).normalized();
    Vector2d ortho(-dist*ds.y(), dist*ds.x());
    return Line(line.start_+ortho, line.end_+ortho, line.dir_);


}

void Line::toPoints(double resolution, std::vector<Eigen::Vector2d> &points)
{
    Vector2d ds= (end_-start_);
    double l = ds.norm();
    Vector2d step=ds.normalized()*resolution;
    Vector2d p=start_;
    int n = l/resolution;
    points.resize(n+1);
    for (int i=0;i<n;++i) {
        points[i]=p;
        p+=step;
    }
    // add end point as well
    points[n]=end_;




}
