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
#include <memory>
#include "circle.h"
#include "line.h"
namespace path_geom {

class Tangentor
{
public:
    static void tangentCircles(const path_geom::Line& line, const path_geom::Circle& circle, double radius,
                        path_geom::aligned<path_geom::Circle>::vector& res, double tol=path_geom::DIST_EPS);



    static void tangentPath(const shared_ptr<path_geom::Shape>& shape, const path_geom::Circle& circle, double radius,
                            bool first_to_second, std::vector<std::shared_ptr<Shape>>& path, double tol=path_geom::DIST_EPS);

    /**
     * @brief returns a smooth path starting from given line to given circle on an arc with given radius
     * results are returned in vector path containing a line from original start point to beginning of arc,
     * the tangent arc, and the circle with start angle set to touch point arc-circle
     * @param line
     * @param circle
     * @param radius
     * @param[out] path
     * @param tol
     */
    static void tangentPath(const path_geom::Line& line, const path_geom::Circle& circle, double radius,
                            std::vector<std::shared_ptr<Shape>>& path, double tol=path_geom::DIST_EPS);

    static void tangentPath(const Circle &circle, const Line &line, double radius,
                            std::vector<std::shared_ptr<Shape> > &path, double tol=path_geom::DIST_EPS);


    static void tangentPath(const path_geom::Circle& small, const path_geom::Circle& large, double radius,bool from_small,
                                 std::vector<std::shared_ptr<Shape> > &path, double tol=path_geom::DIST_EPS);


    /**
     * @brief returns the tangenting arcs starting from line and going to circle
     * @param line
     * @param circle
     * @param radius
     * @param res
     * @param tol
     */
    static void tangentArc(const path_geom::Line& line, const path_geom::Circle& circle, double radius,
                           path_geom::aligned<path_geom::Circle>::vector& res, double tol=path_geom::DIST_EPS);
    /**
     * @brief returns the tangenting arcs starting from circle and going to line
     * @param line
     * @param circle
     * @param radius
     * @param res
     * @param tol
     */
    static void tangentArc(const path_geom::Circle& circle, const path_geom::Line& line,  double radius,
                           path_geom::aligned<path_geom::Circle>::vector& res, double tol=path_geom::DIST_EPS);
    /**
     * @brief returns the circles tangenting circle1 and circle 2 of which the centers are inside circle2
     * @param circle1
     * @param circle2
     * @param radius
     * @param res
     * @param tol
     */
    static void tangentInnerCircles(const path_geom::Circle& circle1, const path_geom::Circle& circle2, double radius,
                                    path_geom::aligned<path_geom::Circle>::vector& res, double tol=path_geom::DIST_EPS);

    /**
     * @brief returns arcs going from circle 1 to circle 2 from the inside directions of circles are considered
     * @param circle1
     * @param circle2
     * @param radius
     * @param res
     * @param tol
     */
    static void tangentInnerArcs(const path_geom::Circle& circle1, const path_geom::Circle& circle2, double radius,
                                 path_geom::aligned<path_geom::Circle>::vector& res, double tol=path_geom::DIST_EPS);




};
}
#endif // TANGENTOR_H
