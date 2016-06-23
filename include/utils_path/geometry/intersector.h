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
                           std::vector<Eigen::Vector2d>& res_points, double tol = path_geom::DIST_EPS);
    static void intersect(const Line& line, const Circle& circle,std::vector<Eigen::Vector2d> &res_points,
                          double tol = path_geom::DIST_EPS);
    /**
     * @brief intersect circles, takes into account the start and end angles of the circles
     * @param c1
     * @param c2
     * @param res_points
     * @param tol
     */
    static void intersectArcs (const Circle& c1, const Circle& c2,
                           std::vector<Eigen::Vector2d>& res_points, double tol = path_geom::DIST_EPS);



};
}
#endif // INTERSECTOR_H
