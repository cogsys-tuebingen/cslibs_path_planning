/**
    Copyright (C) 2015 Karsten Bohlmann E&K AUTOMATION GmbH
    All rights reserved.

    @year   2015
    @author Karsten Bohlmann
    @email  bohlmann@gmail.com
    @file   line.h


*/

#ifndef LINE_H
#define LINE_H
#include "../geometry/shape.h"
namespace path_geom{


class Line : public Shape
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Line(const Eigen::Vector2d& start, const Eigen::Vector2d& end);

    static Line parallel(const path_geom::Line& line, double dist);

    virtual void toPoints(double resolution, std::vector<Eigen::Vector2d>& points);

    Eigen::Vector2d start() const {return start_;}
    Eigen::Vector2d end() const {return end_;}
    int pointRelativePosition(const Eigen::Vector2d &p, double tol=Shape::DEFAULT_TOLERANCE) const;

    double pointSignedDistance(const Eigen::Vector2d &p) const;

    Eigen::Vector2d footPoint(const Eigen::Vector2d &p) const;

protected:
    Eigen::Vector2d start_, end_;
};


}
#endif // LINE_H
