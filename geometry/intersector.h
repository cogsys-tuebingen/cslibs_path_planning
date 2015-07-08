/**
    Copyright (C) 2015 Karsten Bohlmann E&K AUTOMATION GmbH
    All rights reserved.

    @year   2015
    @author Karsten Bohlmann
    @email  bohlmann@gmail.com
    @file   intersector.h


*/

#ifndef INTERSECTOR_H
#define INTERSECTOR_H

#include "circle.h"
#include "line.h"
namespace path_geom {
class Intersector
{
public:

    static void intersect (const Circle& c1, const Circle& c2,
                           std::vector<Eigen::Vector2d>& res_points, double tol = 1e-5);
    static void intersect(const Line& line, const Circle& circle,std::vector<Eigen::Vector2d> &res_points);

};
}
#endif // INTERSECTOR_H
