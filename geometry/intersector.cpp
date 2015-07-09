/**
    Copyright (C) 2015 Karsten Bohlmann E&K AUTOMATION GmbH
    All rights reserved.

    @year   2015
    @author Karsten Bohlmann
    @email  bohlmann@gmail.com
    @file   intersector.cpp


*/
#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include "intersector.h"

using namespace path_geom;
using namespace Eigen;
void Intersector::intersect(const Circle &c1, const Circle &c2, std::vector<Eigen::Vector2d> &res_points,
                            double tol)
{
    tol=fabs(tol);
    //Calculate distance between centres of circle
    Vector2d c1c2 = c2.center()-c1.center();
    double d = (c1c2).norm();
    double r1 = c1.radius();
    double r2 = c2.radius();
    double m = r1+r2;
    double n = fabsf(r1-r2);
    res_points.clear();
    //No solns
    if ( d > m )
        return;
    //Circle are contained within each other
    if ( d < n )
        return;
    //Circles are the same
    if ( fabs(d)<tol && fabs(r1 -r2)<tol )
        return;
    //Solve for a
    double a = ( r1 * r1 - r2 * r2 + d * d ) / (2 * d);
    //Solve for h
    float h = sqrt( r1 * r1 - a * a );



    //Calculate point p, where the line through the circle intersection points crosses the line between the circle centers.
    Vector2d p(c1.center().x() + ( a / d ) * ( c1c2.x()),
            c1.center().y() + ( a / d ) * ( c1c2.y() ));

    //1 soln , circles are touching
    if ( fabs(r1 + r2 -d)<tol) {
        res_points.push_back(p);

        return;
    } else {
        res_points.push_back(p+h/d*Vector2d(c1c2.y(),-1.0*c1c2.x()));
        res_points.push_back(p+h/d*Vector2d(-1.0*c1c2.y(),c1c2.x()));
        /*        p1.x = p.x + ( h / d ) * ( c2.centre.y - c1.centre.y );
        p1.y = p.y - ( h / d ) * ( c2.centre.x - c1.centre.x );

        p2.x = p.x - ( h / d ) * ( c2.centre.y - c1.centre.y );
        p2.y = p.y + ( h / d ) * ( c2.centre.x - c1.centre.x );

        *sol1 = p1;
        *sol2 = p2;*/

    }
}


void Intersector::intersectArcs(const Circle &c1, const Circle &c2, std::vector<Vector2d> &res_points, double tol)
{
    std::vector<Vector2d> ipoints;
    intersect(c1,c2,ipoints,tol);
    for (auto& p : ipoints) {

    }
}


void Intersector::intersect(const Line& line, const Circle& circle,std::vector<Eigen::Vector2d> &res_points, double tol)
{
    tol = fabs(tol);
    Vector2d p1=line.start();
    Vector2d p2=line.end();
    Vector2d c1=circle.center();
    Vector2d p1p2=p2-p1;
    Vector2d c1p1=p1-c1;
    double r = circle.radius();
    double a=p1p2.squaredNorm();
    double b = 2*p1p2.dot(c1p1);
    double c = c1.squaredNorm()+p1.squaredNorm()-2*c1.dot(p1)-r*r;
    res_points.clear();
    double bb4ac = b*b-4*a*c;
    // ***todo not easy to account for numerical errors here ...
    if (bb4ac<-tol*tol) {
        return;
    } else if (fabs(bb4ac)<tol*tol) {

        double mu = -b/(2*a);

        if (mu>=0.0 && mu <=1.0) {
            res_points.resize(1);
            res_points[0]=p1+mu*p1p2;
            return;
        } else {
            return;
        }
    } else {
        double mu1 = (-b - sqrt(bb4ac)) / (2 * a);
        double mu2 = (-b + sqrt(bb4ac)) / (2 * a);
        // ***todo incorporate tolerance here as well?
        if (mu1<0.0 && mu2<0.0) {
            return;
        }
        if (mu1>=0.0 && mu1<=1.0) {
            res_points.push_back(p1+mu1*p1p2);
        }
        if (mu2>=0.0 && mu2<=1.0) {
            res_points.push_back(p1+mu2*p1p2);
        }


    }
}
