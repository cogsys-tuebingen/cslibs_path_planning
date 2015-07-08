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

void Tangentor::tangent(const Line &line, const Circle &circle, double radius,
                        std::vector<path_geom::Circle>& res,  double tol)
{
    res.clear();
    // large circle around original circle with summed up radius
    Circle c2(circle.center(),circle.radius()+radius,Shape::FORWARD);
    Line parallel = Line::parallel(line, radius);

    std::vector<Eigen::Vector2d> ipoints;
    Intersector::intersect(parallel, c2, ipoints);
    std::cout << "c2 radius" << c2.radius() << " center y "<<c2.center().y()
              << " parallel start x " << parallel.start().x()
                 << " parallel start y "<<parallel.start().y()

                 << " parallel end x " << parallel.end().x()
                    << " parallel end y "<<parallel.end().y() << std::endl;

    if (ipoints.size()<1) {

        // no solution
        return;
    } else {
        std::cout << " intersection y "<<ipoints[0].y() << std::endl;
    }
    // first solution
    res.push_back(Circle(ipoints[0],radius,Shape::FORWARD));

}
