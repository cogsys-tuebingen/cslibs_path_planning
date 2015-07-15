/**
    Copyright (C) 2015 Karsten Bohlmann E&K AUTOMATION GmbH
    All rights reserved.

    @year   2015
    @author Karsten Bohlmann
    @email  bohlmann@gmail.com
    @file   tangentor.h


*/

#ifndef TANGENTOR_H
#define TANGENTOR_H
#include "circle.h"
#include "line.h"
namespace path_geom {

class Tangentor
{
public:
    static void tangentCircles(const path_geom::Line& line, const path_geom::Circle& circle, double radius,
                        std::vector<path_geom::Circle>& res, double tol=path_geom::DIST_EPS);

    /**
     * @brief returns the tangenting arcs starting from line and going to circle
     * @param line
     * @param circle
     * @param radius
     * @param res
     * @param tol
     */
    static void tangentArc(const path_geom::Line& line, const path_geom::Circle& circle, double radius,
                           std::vector<path_geom::Circle>& res, double tol=path_geom::DIST_EPS);
    /**
     * @brief returns the tangenting arcs starting from circle and going to line
     * @param line
     * @param circle
     * @param radius
     * @param res
     * @param tol
     */
    static void tangentArc(const path_geom::Circle& circle, const path_geom::Line& line,  double radius,
                           std::vector<path_geom::Circle>& res, double tol=path_geom::DIST_EPS);
    /**
     * @brief returns the circles tangenting circle1 and circle 2 of which the centers are inside circle2
     * @param circle1
     * @param circle2
     * @param radius
     * @param res
     * @param tol
     */
    static void tangentInnerCircles(const path_geom::Circle& circle1, const path_geom::Circle& circle2, double radius,
                                    std::vector<path_geom::Circle>& res, double tol=path_geom::DIST_EPS);

    /**
     * @brief returns arcs going from circle 1 to circle 2 from the inside directions of circles are considered
     * @param circle1
     * @param circle2
     * @param radius
     * @param res
     * @param tol
     */
    static void tangentInnerArcs(const path_geom::Circle& circle1, const path_geom::Circle& circle2, double radius,
                                 std::vector<path_geom::Circle>& res, double tol=path_geom::DIST_EPS);

};
}
#endif // TANGENTOR_H
