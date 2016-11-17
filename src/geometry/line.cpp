/**
    Copyright (C) 2015 Karsten Bohlmann E&K AUTOMATION GmbH
    All rights reserved.

    @year   2015
    @author Karsten Bohlmann
    @email  bohlmann@gmail.com
    @file   line.cpp


*/

#include <cslibs_path_planning/geometry/line.h>

using namespace std;
using namespace path_geom;
using namespace Eigen;

Line::Line(const Eigen::Vector2d& start, const Eigen::Vector2d& end)
    :start_(start), end_(end)
{

}


Line::Line(const PathPose &start, double length, int direction)

{
    direction=direction/abs(direction);
    if (direction==path_geom::FORWARD) {
        start_=start.pos_;
        end_ = start.pos_+fabs(length)*Eigen::Vector2d(cos(start.theta_),sin(start.theta_));
    } else {
        end_ =start.pos_;
        start_ = end_-fabs(length)*Eigen::Vector2d(cos(start.theta_),sin(start.theta_));
    }

}


path_geom::Line Line::parallel(const Line &line, double dist)
{
    Vector2d ds= (line.end_-line.start_).normalized();
    Vector2d ortho(-dist*ds.y(), dist*ds.x());
    return Line(line.start_+ortho, line.end_+ortho);


}


void Line::toPoses(double resolution, PathPoseVec &poses, int move_direction, bool with_start_pose)
{
    if (fabs(resolution)<1e-10) {
        poses.clear();
        return;
    }
    Vector2d ds= (end_-start_);
    double l = ds.norm();
    int n = l/resolution;
    Vector2d step=ds.normalized()*resolution;

    double theta;
    Vector2d p;
    if (move_direction==path_geom::BACKWARD) {
        p = end_;
        step = step*-1.0;
    } else {
        p = start_;


    }
    theta = atan2(ds.y(),ds.x());
    if (with_start_pose) {
        poses.resize(n+1);
    } else {
        poses.resize(n);
        p=p+step;
    }
    for (int i=0;i<n;++i) {
        poses[i].pos_=p;
        poses[i].theta_ = theta;
        p+=step;
    }
/*    if (with_end_pose) {
    // add end point as well
        if (move_direction==path_geom::BACKWARD) {
            poses[n].assign(start_,theta);
        } else {
            poses[n].assign(end_,theta);
        }
    }*/
}


void Line::toPoints(double resolution, std::vector<Eigen::Vector2d> &points)
{
    Vector2d ds= (end_-start_);
    double l = ds.norm();
    Vector2d step=ds.normalized()*resolution;
    Vector2d p=start_;
    int n = l/resolution;
    points.resize(n+1);
    for (int i=0;i<n;++i) {
        points[i]=p;
        p+=step;
    }
    // add end point as well
    points[n]=end_;

}


int Line::pointRelativePosition(const Vector2d &c, double tol) const
{
    Vector2d ab = end_-start_;
    Vector2d ac = c-start_;
    double d = (ab.x())*(ac.y()) - (ab.y())*(ac.x());
    if (d>fabs(tol)) {
        return 1;
    } else if (d<-fabs(tol)) {
        return -1;
    } else {
        return 0;
    }

}


double Line::pointSignedDistance(const Vector2d &c) const
{
    Vector2d ab = end_-start_;
    Vector2d ac = c-start_;
    return ((ab.x())*(ac.y()) - (ab.y())*(ac.x()))/ab.norm();

}


Vector2d Line::footPoint(const Vector2d &p) const
{
    double dist = pointSignedDistance(p);
    Vector2d ds= (end_-start_).normalized();
    Vector2d ortho(-dist*ds.y(), dist*ds.x());
    return (p-ortho);
}


bool Line::selectStartPoint(const Vector2d &start,  double tol)
{
    int rel_pos = pointRelativePosition(start,tol);
    if (!rel_pos) {
        // point is on line
        start_ = start;
        return true;
    } else {
        return false;
    }
}


bool Line::selectEndPoint(const Vector2d &end,  double tol)
{
    int rel_pos = pointRelativePosition(end,tol);
    if (!rel_pos) {
        // point is on line
        end_ = end;
        return true;
    } else {
        return false;
    }
}


ostream& path_geom::operator<<(ostream& os, const Line& line)
{
    const Vector2d& start = line.startPoint();
    const Vector2d& end = line.endPoint();
    os << "Line S(" << start.x()<<","<<start.y()<<") E("<<end.x()<<","
       <<end.y()<<") ";
    return os;
}


bool Line::isPointOnSegment(const Vector2d &p, double tol) const
{
    double d1 = (p-start_).norm();
    double d2 = (p-end_).norm();
    double dl = (end_-start_).norm();
    if (fabs(d1+d2-dl)<tol) {
        return true;
    } else {
        return false;
    }
}

double Line::distanceTo(const Vector2d &p) const
{

    Vector2d f = footPoint(p);
    if (isPointOnSegment(f)) {
        return (f-p).norm();
    } else {
        return std::min((p-start_).norm(),(p-end_).norm());
    }
}

Vector2d Line::nearestPointTo(const Vector2d &p) const
{
    Vector2d f = footPoint(p);
    if (isPointOnSegment(f)) {
        return f;
    } else {
        double d1 = (p-start_).norm();
        double d2 = (p-end_).norm();
        if (d1<d2) {
            return start_;
        } else {
            return end_;
        }
    }
}

Vector2d Line::projectPoint(const Vector2d &p) const
{
    Vector2d v = p - start_;
    Vector2d s = end_ - start_;
    return start_ + v.dot(s) / (s.dot(s)) * s;
}
