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
    static void tangent(const Line& line, const Circle& circle, double radius,
                        std::vector<path_geom::Circle>& res, double tol=1e-5);
};
}
#endif // TANGENTOR_H
