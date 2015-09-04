/**
 * @file CollisionGridMap2d.cpp
 * @date Feb 2014
 * @author buck
 */

/// Project
#include "CollisionGridMap2d.h"

/// SYSTEM
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

///////////////////////////////////////////////////////////////////////////////
// class CollisionGridMap2d
///////////////////////////////////////////////////////////////////////////////

using namespace lib_path;

RobotArea::RobotArea(CollisionGridMap2d const*  parent, double forward, double backward, double width, double theta)
    : parent_(parent), x_(0), y_(0), fw_(forward), bw_(backward), hw_(width/2.0), theta_(theta), init_(false)
{
    Eigen::Vector2d fr_(forward, -hw_);
    Eigen::Vector2d fl_(forward, hw_);
    Eigen::Vector2d br_(backward, -hw_);
    Eigen::Vector2d bl_(backward, hw_);

    Eigen::Rotation2D<double> rot(theta);

    centre = Eigen::Vector2d (x_,y_);

    fr = rot * fr_;
    fl = rot * fl_;
    br = rot * br_;
    bl = rot * bl_;
}

void RobotArea::setPosition(int x, int y)
{
    x_ = x;
    y_ = y;
    centre = Eigen::Vector2d (x_,y_);
    init_ = true;
}

/**
     * @brief Start a new iteration.
     */
void RobotArea::begin() {
    idx = -1;

    assert(init_);

    pts_.clear();

    int steps = 32;
    double inc = 1.0 / steps;
    double i = 0.0;
    for(int j = 0; j < steps; ++j) {
	    pts_.push_back(fr + i * (fl-fr));
        pts_.push_back(br + i * (bl-br));
        pts_.push_back(bl + i * (fl-bl));
        pts_.push_back(fr + i * (br-fr));

        i += inc;
    }

    
}

void RobotArea::paint(bool free) {
    assert(init_);

    static int max_step = 1000;
    static int step = 0;

    int rows = parent_->getHeight();
    int cols = parent_->getWidth();
    double scale = 4;

    static cv::Mat dbg_img;

    if(step == 0) {
        dbg_img = cv::Mat(rows, cols, CV_8UC3, cv::Scalar::all(0));
        for(int row = 0; row < rows; ++row) {
            for(int col = 0; col < cols; ++col) {
                dbg_img.at<cv::Vec3b>(row, col) = cv::Vec3b::all(parent_->SimpleGridMap2d::isFree(col, row) ? 255 : 0);
            }
        }

        cv::resize(dbg_img, dbg_img, cv::Size(), scale, scale, CV_INTER_NN);
    }

    if(!free) {
        for(int i = 0; i < pts_.size(); ++i) {
            const Eigen::Vector2d& pt = centre + pts_[i];
            cv::circle(dbg_img, cv::Point2f(scale*pt(0),scale*pt(1)), 1, cv::Scalar(0x00, 0x00, 0xFF), CV_FILLED, CV_AA);
        }
    }
//    cv::circle(dbg_img, cv::Point2f(scale*x_,scale*y_), 1, cv::Scalar(0x00, 0xCC, 0xFF), CV_FILLED, CV_AA);
    //cv::circle(dbg_img, cv::Point2f(scale*(x_ + 4* std::cos(theta_)),scale*(y_ + 4* std::sin(theta_))), 3, cv::Scalar(0x00, 0xCC, 0xFF), CV_FILLED, CV_AA);

    if(!free) {
        cv::Scalar col = free ? cv::Scalar(0x00, 0xFF, 0x00) : cv::Scalar(0x00, 0x00, 0xFF);

        cv::line(dbg_img, cv::Point2f(scale*(centre+fl)(0), scale*(centre+fl)(1)), cv::Point2f(scale*(centre+fr)(0), scale*(centre+fr)(1)), col, 1, CV_AA);
        cv::line(dbg_img, cv::Point2f(scale*(centre+br)(0), scale*(centre+br)(1)), cv::Point2f(scale*(centre+fr)(0), scale*(centre+fr)(1)), col, 1, CV_AA);
        cv::line(dbg_img, cv::Point2f(scale*(centre+fl)(0), scale*(centre+fl)(1)), cv::Point2f(scale*(centre+bl)(0), scale*(centre+bl)(1)), col, 1, CV_AA);
        cv::line(dbg_img, cv::Point2f(scale*(centre+br)(0), scale*(centre+br)(1)), cv::Point2f(scale*(centre+bl)(0), scale*(centre+bl)(1)), col, 1, CV_AA);
    }

    if(step == 0) {        
        cv::imshow("dbg", dbg_img);
        cv::waitKey(20);
    }

    if(++step > max_step) {
        step = 0;
    }

}

/**
     * @brief Select next cell.
     * @return False if there are no more cells.
     */
bool RobotArea::next() {
    return ++idx < pts_.size();
}

/**
     * @brief Get the cell coordinates of the currently selected cell.
     * @param x x-coordinates of the cell.
     * @param y y coordinate of the cell.
     */
void RobotArea::getCell( int& x, int& y ) const {
    const Eigen::Vector2d& pt = centre + pts_[idx];
    x = pt(0);
    y = pt(1);
}

/**
     * @brief Get the value of the currently selected cell.
     * @return The value.
     */
uint8_t RobotArea::getValue() const
{ return 0; }

/**
     * @brief Set the value of the currently selected cell.
     * @param value The new value.
     */
void RobotArea::setValue( const uint8_t value )
{}








CollisionGridMap2d::CollisionGridMap2d(const unsigned int w, const unsigned int h, const double yaw , double r, double forward, double backward, double width)
    : RotatedGridMap2d(w,h, yaw, r)
{
    for(int t = 0; t< ANGLE_DISCRETIZATION; ++t) {
        double theta = (2 * M_PI) * (t / (ANGLE_DISCRETIZATION + 1.0));
        double res = getResolution();

        areas_[t] = new RobotArea(this, forward / res, backward / res, width / res, theta);
    }
}

CollisionGridMap2d::~CollisionGridMap2d()
{
    for(int t = 0; t< ANGLE_DISCRETIZATION; ++t) {
        delete areas_[t];
    }
}

bool CollisionGridMap2d::isFree(const unsigned int x, const unsigned int y, const double theta) const
{
    if(!isInMap((int) x,(int) y) || !SimpleGridMap2d::isFree(x, y)) {
        return false;
    }

    double t = theta;
    while(t < 0) t += 2*M_PI;
    while(t >= 2*M_PI) t -= 2*M_PI;

    // check boundary
    RobotArea* a = areas_[(int) round(t)];
    a->setPosition(x,y);
    bool free = isAreaFree(*a);
    //a->paint(free);
    return free;
}


bool CollisionGridMap2d::isNoInformation(const unsigned int x, const unsigned int y, const double theta) const
{
    if(!isInMap((int) x,(int) y)) {
        return false;
    }
    if(SimpleGridMap2d::isOccupied(x, y)) {
        return false;
    }

    double t = theta;
    while(t < 0) t += 2*M_PI;
    while(t >= 2*M_PI) t -= 2*M_PI;

    // check boundary
    RobotArea* a = areas_[(int) round(t)];
    a->setPosition(x,y);
    bool free = isAreaFreeOrUnknown(*a);
    //a->paint(free);
    return free;
}
