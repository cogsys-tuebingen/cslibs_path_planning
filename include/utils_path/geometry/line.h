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
#include <iostream>

namespace path_geom{


class Line : public Shape
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Line(const Eigen::Vector2d& startPoint, const Eigen::Vector2d& endPoint);
    Line(const PathPose& startPoint, double length, int direction);
    Line(const Line& src) {
        start_=src.start_;
        end_=src.end_;
    }

    static Line parallel(const path_geom::Line& line, double dist);

    virtual void toPoints(double resolution, std::vector<Eigen::Vector2d>& points);
    virtual void toPoses(double resolution, PathPoseVec& res_poses, int move_direction,  bool with_start_pose=true);


    virtual bool selectStartPoint(const Eigen::Vector2d& start, double tol=path_geom::DIST_EPS);
    virtual bool selectEndPoint(const Eigen::Vector2d& end,  double tol=path_geom::DIST_EPS);

    virtual Eigen::Vector2d startPoint() const {return start_;}
    virtual Eigen::Vector2d endPoint() const {return end_;}
    int pointRelativePosition(const Eigen::Vector2d &p, double tol=path_geom::DIST_EPS) const;

    double pointSignedDistance(const Eigen::Vector2d &p) const;

    Eigen::Vector2d footPoint(const Eigen::Vector2d &p) const;

    bool isPointOnSegment(const Eigen::Vector2d &p, double tol=path_geom::DIST_EPS) const;

    virtual double distanceTo(const Eigen::Vector2d& point) const;
    virtual Eigen::Vector2d nearestPointTo(const Vector2d &p) const;

    Eigen::Vector2d projectPoint(const Vector2d &p) const;

    friend ostream& operator<<(ostream& os, const Line& line);

protected:
    Eigen::Vector2d start_, end_;
};

typedef std::vector<Line,Eigen::aligned_allocator<Line>> LineVec;

}
#endif // LINE_H
