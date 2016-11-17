/**
    Copyright (C) 2015 Karsten Bohlmann E&K AUTOMATION GmbH
    All rights reserved.

    @year   2015
    @author Karsten Bohlmann
    @email  bohlmann@gmail.com
    @file   tangentor.cpp


*/
#include <iostream>
#include <limits>
#include <cslibs_path_planning/geometry/tangentor.h>


#include <cslibs_path_planning/geometry/intersector.h>

using namespace std;
using namespace path_geom;
using namespace Eigen;


void Tangentor::tangentPath(const shared_ptr<Shape> &shape, const Circle &circle, double radius,
                            bool first_to_second, std::vector<std::shared_ptr<Shape> > &path, double tol)
{
    auto first_line = std::dynamic_pointer_cast<Line>(shape);
    auto first_circle = std::dynamic_pointer_cast<Circle>(shape);
    if (first_line) {
        if (first_to_second) {
            Tangentor::tangentPath(*first_line,circle,radius,path);
        } else {
            std::cout << "search tangent from circle to line "<<*first_line << std::endl;
            Tangentor::tangentPath(circle,*first_line, radius,path);
        }
    } else if (first_circle) {

        Tangentor::tangentPath(circle,*first_circle,radius,!first_to_second, path);
    } else {
        path.clear();
    }

}


void Tangentor::tangentCircles(const path_geom::Line &line, const path_geom::Circle &circle, double radius,
                        path_geom::aligned<path_geom::Circle>::vector& res,  double tol)
{
    res.clear();
    // large circle around original circle with summed up radius
    Circle c2(circle.center(),circle.radius()+radius,circle.direction());
    LineVec parallels;
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


void Tangentor::tangentPath(const Line &line, const Circle &circle, double radius,
                            std::vector<std::shared_ptr<Shape> > &path, double tol)
{
    path.clear();
    path_geom::aligned<path_geom::Circle>::vector tangent_arcs;
    tangentArc(line,circle,radius,tangent_arcs);
    if (tangent_arcs.size()==0) {
        return;
    }
    vector<Vector2d> ipoints;


    auto min_it = min_element(begin(tangent_arcs), end(tangent_arcs), Circle::compareArcAngle);

    std::shared_ptr<Circle> tangent_arc = make_aligned<Circle>(*min_it);


    // line from start point of original line to beginning of arc
    auto start_line = make_aligned<Line>(line.startPoint(),tangent_arc->startPoint());
    path.push_back(start_line);

    path.push_back(tangent_arc);
    auto end_circle = make_aligned<Circle>(circle);
    end_circle->setStartAngle(tangent_arc->endPoint());
    end_circle->setArcAngle(2*M_PI);
    path.push_back(end_circle);
}


void Tangentor::tangentPath(const Circle &circle, const Line &line, double radius, std::vector<std::shared_ptr<Shape> > &path, double tol)
{
    path.clear();
    aligned<Circle>::vector  tangent_arcs;
    tangentArc(circle,line,radius,tangent_arcs);
    if (tangent_arcs.size()==0) {
        return;
    }

    auto min_it = min_element(begin(tangent_arcs), end(tangent_arcs), Circle::compareArcAngle);

    std::shared_ptr<Circle> tangent_arc = make_aligned<Circle>(*min_it);


    auto start_circle = make_aligned<Circle>(circle);

    auto end_line = make_aligned<Line>(tangent_arc->endPoint(),line.endPoint());
    start_circle->selectEndPoint(tangent_arc->startPoint());

    path.push_back(start_circle);
    path.push_back(tangent_arc);
    path.push_back(end_line);
}


void Tangentor::tangentArc(const path_geom::Line &line, const path_geom::Circle &circle, double radius,
                           path_geom::aligned<path_geom::Circle>::vector &res, double tol)
{
    res.clear();
    aligned<Circle>::vector tangent_circles;

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
            //std::cout << "foot point "<< line.footPoint(tangent_circle.center()) << std::endl;
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
                           path_geom::aligned<path_geom::Circle>::vector &res, double tol)
{
    res.clear();
    path_geom::aligned<path_geom::Circle>::vector tangent_circles;

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


void Tangentor::tangentInnerCircles(const Circle &circle1, const Circle &circle2, double radius, path_geom::aligned<Circle>::vector &res, double tol)
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

    // calculate intersection points between large_circ and small_circ
    std::vector<Eigen::Vector2d> ipoints;
    Intersector::intersect(large_circ,small_circ, ipoints,tol);

    // create the tangenting circles with intersection points for centers
    for (auto& ipoint : ipoints) {
        res.push_back(Circle(ipoint,radius,circle2.direction()));
    }
}


void Tangentor::tangentPath(const Circle &small, const Circle &large, double radius, bool from_small, std::vector<std::shared_ptr<Shape> > &path, double tol)
{
    path.clear();
    path_geom::aligned<Circle>::vector tangent_circles;

    tangentInnerCircles(small,large, radius,tangent_circles,tol);
    std::cout << "tangent inner circles found "<<tangent_circles.size() << std::endl;


    path_geom::aligned<path_geom::Circle>::vector tangent_arcs;
    std::vector<Eigen::Vector2d> ipoints_small,ipoints_large;

    for (auto &tcircle : tangent_circles) {
        std::cout << "tangent circle "<<tcircle.center().x() << " "<<tcircle.center().y() << std::endl;
        Intersector::intersectArcs(small, tcircle, ipoints_small,tol);
        Intersector::intersectArcs(tcircle, large, ipoints_large,tol);
        if (ipoints_small.size()==1 && ipoints_large.size()==1) {
            std::cout << "touches both small and large" << std::endl;
            if (!from_small) {
                tcircle.selectStartPoint(ipoints_large.front());
                tcircle.selectEndPoint(ipoints_small.front());
            }
            tangent_arcs.push_back(tcircle);
        } else {
            std::cout << "touch small " << ipoints_small.size() << " large " << ipoints_large.size()<< std::endl;
        }
    }
    std::cout.flush();
    if (tangent_arcs.empty()) {
        // no solution
        return;
    }



    // select tangent arc with shortest arc length in case there are more than one solutions
    auto min_it = min_element(begin(tangent_arcs), end(tangent_arcs), Circle::compareArcAngle);

    // arc lengths greater than pi are not desired
    if (min_it->getArcAngle()>M_PI) {
        return;
    }
    std::shared_ptr<Circle> tangent_arc = make_aligned<Circle>(*min_it);
    std::shared_ptr<Circle> small_arc = make_aligned<Circle>(small);
    std::shared_ptr<Circle> large_arc = make_aligned<Circle>(large);
    if (from_small) {
        small_arc->selectEndPoint(tangent_arc->startPoint());
        large_arc->selectStartPoint(tangent_arc->endPoint());
        path.push_back(small_arc);
        path.push_back(tangent_arc);
        path.push_back(large_arc);
    } else {
        large_arc->selectEndPoint(tangent_arc->startPoint());
        small_arc->selectStartPoint(tangent_arc->endPoint());
       // tangent_arc->setStartAngle(ipoints_large.front());
       // tangent_arc->setEndAngle(ipoints_small.front());
        path.push_back(large_arc);
        path.push_back(tangent_arc);
        path.push_back(small_arc);

    }

}


void Tangentor::tangentInnerArcs(const Circle &circle1, const Circle &circle2, double radius, path_geom::aligned<Circle>::vector &res, double tol)
{
    res.clear();
    if (circle1.direction()==circle2.direction()) {
        // directions of circles are incompatible
        return;
    }
    path_geom::aligned<Circle>::vector arcs;
    path_geom::aligned<path_geom::Circle>::vector tangent_circles;
    tangentInnerCircles(circle1,circle2, radius,tangent_circles,tol);
    std::cout << "tangentinnerarcs: found "<< tangent_circles.size()<<" circles"<<std::endl;
    for (auto& tangent_circle : tangent_circles) {
        // find the end point of the arc
        std::vector<Eigen::Vector2d> ipoints;
        //std::cout << "circle1 center "<<circle1.center() << " tangent circle "<<tangent_circle.center() << std::endl;
        //std::cout << "radius circle1 "<<circle1.radius() << " radius tangentcircle2 "<< tangent_circle.radius()<<std::endl;
        Intersector::intersectArcs(circle1, tangent_circle, ipoints,tol);
        //std::cout << "intersection points circle 1 to arc:"<<ipoints.size() << std::endl;
        if (ipoints.size()==1) {
            tangent_circle.setStartAngle(ipoints[0]);
            Intersector::intersectArcs(tangent_circle, circle2, ipoints,tol);
          //  std::cout << "intersection points arc to circle 2 :"<<ipoints.size() << std::endl;
            if (ipoints.size()==1) {
                tangent_circle.setEndAngle(ipoints[0]);
                arcs.push_back(tangent_circle);
            }
        }
    }
    if (arcs.size()==1) {
        res.push_back(arcs.front());
    } else if (arcs.size()>=2) {
        // somehow two arcs , use the shorter one
        double max_arc_angle = std::numeric_limits<double>::max();
        size_t min_idx = 0;
        for (size_t i=0;i<arcs.size();++i) {
            double arc_angle = arcs[i].getArcAngle();
            if (arc_angle<max_arc_angle) {
                min_idx = i;
                max_arc_angle =arc_angle;
            }
        }
        res.push_back(arcs[min_idx]);
    }
    // no else.. res is empty

}
