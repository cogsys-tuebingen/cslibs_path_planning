/**
    Copyright (C) 2015 Karsten Bohlmann E&K AUTOMATION GmbH
    All rights reserved.

    @year   2015
    @author Karsten Bohlmann
    @email  bohlmann@gmail.com
    @file   tangentor.cpp


*/
#include <iostream>
#include "tangentor.h"


#include "intersector.h"


using namespace path_geom;
using namespace Eigen;

void Tangentor::tangentCircles(const path_geom::Line &line, const path_geom::Circle &circle, double radius,
                        std::vector<path_geom::Circle>& res,  double tol)
{
    res.clear();
    // large circle around original circle with summed up radius
    Circle c2(circle.center(),circle.radius()+radius,circle.direction());
    std::vector<Line> parallels;
    std::vector<std::vector<Eigen::Vector2d>> ipoints;
    parallels.push_back(Line::parallel(line, radius));
    parallels.push_back(Line::parallel(line, -radius));
    ipoints.resize(2);
    for (size_t j=0;j<2;++j) {
        Intersector::intersect(parallels[j], c2, ipoints[j],tol);
        for (size_t i=0;i<ipoints[j].size();++i) {
            // tangent circles always get the reversed direction of original circle
            // ***todo what if the circle is inside orginal circle?
            res.push_back(Circle(ipoints[j][i],radius,circle.direction()*(-1)));
        }
    }

}


void Tangentor::tangentArc(const path_geom::Line &line, const path_geom::Circle &circle, double radius,
                           std::vector<path_geom::Circle> &res, double tol)
{
    res.clear();
    std::vector<path_geom::Circle> tangent_circles;

    tangentCircles(line,circle,radius, tangent_circles, tol);
    if (tangent_circles.size()<1) {
        return;
    }
    for (unsigned i=0;i<tangent_circles.size();++i) {
        path_geom::Circle& tangent_circle=tangent_circles[i];
        int pos = line.pointRelativePosition(tangent_circle.center(),tol);
        if (pos==0) {
            // somethign is wrong ..center of tangent circle shouldnt be on line
            continue;
        } else if ((tangent_circle.direction()>0 && pos>0)
                   || (tangent_circle.direction()<0 && pos<0)) {

            // start point ist the touch point between line and tangent circle
            tangent_circle.setStartAngle(line.footPoint(tangent_circle.center()));
            std::cout << "foot point "<< line.footPoint(tangent_circle.center()) << std::endl;
            // line connecting centers of both circles
            Line ccline(tangent_circle.center(),circle.center());
            std::vector<Eigen::Vector2d> ipoints;
            Intersector::intersect(ccline, tangent_circle,ipoints,tol);
            if (ipoints.size()!=1) {
                // something is wrong
                std::cout << "error no interscetion points"<<std::endl;
                continue;
            } else {
                std::cout << "touch point to circle "<<   ipoints[0] << std::endl;
                tangent_circle.setEndAngle(ipoints[0]);
            }
            res.push_back(tangent_circle);
        }
    }
}


void Tangentor::tangentArc(const path_geom::Circle &circle,const path_geom::Line &line,  double radius,
                           std::vector<path_geom::Circle> &res, double tol)
{
    res.clear();
    std::vector<path_geom::Circle> tangent_circles;

    tangentCircles(line,circle,radius, tangent_circles, tol);
    if (tangent_circles.size()<1) {
        return;
    }
    for (unsigned i=0;i<tangent_circles.size();++i) {
        path_geom::Circle& tangent_circle=tangent_circles[i];
        int pos = line.pointRelativePosition(tangent_circle.center(),tol);
        if (pos==0) {
            // something is wrong ..center of tangent circle shouldnt be on line
            continue;
        } else if ((tangent_circle.direction()>0 && pos>0)
                   || (tangent_circle.direction()<0 && pos<0)) {


            // line connecting centers of both circles
            Line ccline(tangent_circle.center(),circle.center());
            std::vector<Eigen::Vector2d> ipoints;
            Intersector::intersect(ccline, tangent_circle,ipoints,tol);
            if (ipoints.size()!=1) {
                // something is wrong
                std::cout << "error no interscetion points"<<std::endl;
                continue;
            } else {
                std::cout << "touch point to circle "<<   ipoints[0] << std::endl;
                tangent_circle.setStartAngle(ipoints[0]);
            }
            // end point ist the touch point between line and tangent circle
            tangent_circle.setEndAngle(line.footPoint(tangent_circle.center()));
            std::cout << "foot point "<< line.footPoint(tangent_circle.center()) << std::endl;
            res.push_back(tangent_circle);
        }
    }
}


void Tangentor::tangentInnerCircles(const Circle &circle1, const Circle &circle2, double radius, std::vector<Circle> &res, double tol)
{
    res.clear();
    if (radius>=circle2.radius()) {
        // no solution
        return;
    }
    // large circle extended by radius around first circle
    Circle large_circ(circle1.center(),circle1.radius()+radius,circle1.direction());
    // small circle made smaller by radius around second circle
    Circle small_circ(circle2.center(),circle2.radius()-radius,circle2.direction());
    std::vector<Eigen::Vector2d> ipoints;

    Intersector::intersect(large_circ,small_circ, ipoints,tol);
    for (size_t i=0;i<ipoints.size();++i) {
        res.push_back(Circle(ipoints[i],radius,circle2.direction()));
    }

}


void Tangentor::tangentInnerArcs(const Circle &circle1, const Circle &circle2, double radius, std::vector<Circle> &res, double tol)
{
    res.clear();
    if (circle1.direction()==circle2.direction()) {
        // directions of circles are incompatible
        return;
    }
    std::vector<path_geom::Circle> tangent_circles;
    tangentInnerCircles(circle1,circle2, radius,tangent_circles,tol);
    for (auto& tangent_circle : tangent_circles) {
        // find the end point of the arc
        std::vector<Eigen::Vector2d> ipoints;

        Intersector::intersectArcs(circle2, tangent_circle, ipoints,tol);
    }

}
