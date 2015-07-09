/**

    @year   2015
    @author Karsten Bohlmann
    @email  bohlmann@gmail.com
    @file   test_geometry.cpp


*/

#include <iostream>
#include "../geometry/intersector.h"
#include "../geometry/tangentor.h"

using namespace path_geom;
using namespace Eigen;

int main(int argc, char **argv)
{
    Eigen::Vector2d c1(2.0, 0.0);
    Eigen::Vector2d c2(3.0, 0.0);
    Eigen::Vector2d c3(2.0, 2.0);
    Eigen::Vector2d c4(2.0, -0.5);
    Eigen::Vector2d p1(0.0, 0.0);
    Eigen::Vector2d p2(4.0, 0.0);
    path_geom::Circle circ1(c1,1.0,Shape::FORWARD);
    path_geom::Line line1(p1,p2);

    // check point online / left /right
    int rel_pos1 = line1.pointRelativePosition(c3);
    int rel_pos2 = line1.pointRelativePosition(c2);
    int rel_pos4 = line1.pointRelativePosition(c4);
    if (rel_pos1>0 && rel_pos2==0 && rel_pos4<0) {
        std::cout << "success pointrelpos"<< std::endl;
    } else {
        std::cout << "failure"<< std::endl;
    }
    Eigen::Vector2d foot = line1.footPoint(c3);
    std::cout << "dist "<< line1.pointSignedDistance(c3) << std::endl;
    std::cout << "foot "<<foot << std::endl;
    if ((fabs(foot.x()-c3.x())<1e-5)&& (fabs(foot.y())<1e-5)) {
        std::cout << "success footpoint"<< std::endl;
    } else {
        std::cout << "failure footpoint"<< std::endl;
    }



    std::vector<Vector2d> results;
    Intersector::intersect(line1,circ1,results);
    if (results.size()==2) {
        std::cout << "success"<< std::endl;
    } else {
        std::cout << "failure"<< std::endl;
    }

    Eigen::Vector2d p3(0.0, 0.0);
    Eigen::Vector2d p4(0.0, 4.0);
    path_geom::Line line2(p3,p4);

    path_geom::Line line3=path_geom::Line::parallel(line2,2.0);

    std::cout << "sx "<< line3.start().x() << " sy "<< line3.start().y()
        << "ex "<< line3.end().x() << " ey "<< line3.end().y() << std::endl;

    path_geom::Circle circ2(c2,1.0,Shape::FORWARD);
    Intersector::intersect(circ1,circ2,results);
    if (results.size()==2) {
        std::cout << "success circle"<< std::endl;
    } else {
        std::cout << "failure circle"<< std::endl;
    }
    path_geom::Circle circ3(c3,1.0,Shape::FORWARD);
    Intersector::intersect(circ1,circ3,results);
    if (results.size()==1) {
        std::cout << "success circle"<< std::endl;
    } else {
        std::cout << "failure circle"<< std::endl;
    }

    // a real example
    Eigen::Vector2d A(0.0,0.0),B(0.0,9.0),C(0.0,15.0),E(0.0,8.4);
    Line path1(A,B);
    Line path2(B,C);
    Line path3(A,C);
    Circle obstacle(E,2.0,Shape::FORWARD);
    std::vector<Circle> tangent_circles;
    Tangentor::tangentCircles(path1,obstacle,2.0,tangent_circles);
    for (size_t i=0;i<tangent_circles.size();++i) {
        std::cout << "tangenting circle center x "<<tangent_circles[i].center().x()
                     << " y "<<tangent_circles[i].center().y()
                     << " radius "<<tangent_circles[i].radius() <<std::endl;
    }

    Tangentor::tangentArc(path1, obstacle,2.0, tangent_circles);
    if (tangent_circles.size()>0) {
        std::cout << "tangenting circle start "<<tangent_circles[0].startPoint()<< std::endl;
        std::cout << "tangenting circle end "<<tangent_circles[0].endPoint()<< std::endl;
    } else {
        std::cout << "ERROR tangent circle"<< std::endl;
    }
    Tangentor::tangentArc(obstacle,path2, 2.0, tangent_circles);
    if (tangent_circles.size()>0) {
        std::cout << "tangenting circle start "<<tangent_circles[0].startPoint()<< std::endl;
        std::cout << "tangenting circle end "<<tangent_circles[0].endPoint()<< std::endl;
    } else {
        std::cout << "ERROR tangent circle"<< std::endl;
    }
    std::cout << "long path "<< std::endl;
    Tangentor::tangentArc(obstacle,path3, 2.0, tangent_circles);
    for (auto& circ : tangent_circles) {
        std::cout << "tangenting circle start "<<circ.startPoint()<< std::endl;
        std::cout << "tangenting circle end "<<circ.endPoint()<< std::endl;
        std::cout << " tangenting circle center "<<circ.center() << std::endl;
    }




    return 0;
}
