/**
 * @file CollisionGridMap2d.cpp
 * @date Feb 2014
 * @author buck
 */

/// Project
#include <utils_path/common/CollisionGridMap2d.h>

/// SYSTEM
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

///////////////////////////////////////////////////////////////////////////////
// class CollisionGridMap2d
///////////////////////////////////////////////////////////////////////////////

using namespace lib_path;

RobotArea::RobotArea(CollisionGridMap2d const*  parent, double forward, double backward, double width, double theta)
    : parent_(parent), x_(0), y_(0), hw_(width/2.0), fw_(forward), bw_(backward),  theta_(theta), init_(false)
{
    Eigen::Vector2d fr_(forward, -hw_);
    Eigen::Vector2d fl_(forward, hw_);
    Eigen::Vector2d br_(backward, -hw_);
    Eigen::Vector2d bl_(backward, hw_);

    Eigen::Rotation2D<double> rot(theta);

    Eigen::Vector2d fr = rot * fr_;
    Eigen::Vector2d fl = rot * fl_;
    Eigen::Vector2d br = rot * br_;
    Eigen::Vector2d bl = rot * bl_;

//    double m_per_cell = parent->getResolution();
    double m_per_step = width / 8;
//    double m_per_step = m_per_cell * cell_per_step;

    int steps_row = std::ceil(width / m_per_step);
    int steps_col = std::ceil((forward - backward) / m_per_step);
    double inc_row = 1.0 / steps_row;
    double inc_col = 1.0 / steps_col;

    double i_row = 0.0;
    for(int row = 0; row <= steps_row; ++row) {
        Eigen::Vector2d left = bl + i_row * (fl-bl);
        Eigen::Vector2d right = fr + i_row * (br-fr);

        double i_col = 0.0;
        for(int col = 0; col <= steps_col; ++col) {
            cells_.emplace_back(left + i_col * (right-left));

            i_col += inc_col;
        }

        i_row += inc_row;
    }


    //    {
    //        int steps_row = width / f;
    //        double inc = 1.0 / steps_row;
    //        double i = 0.0;
    //        for(int row = 0; row <= steps_row; ++row) {
    //            cells_.emplace_back(bl + i * (fl-bl));
    //            cells_.emplace_back(fr + i * (br-fr));

    //            i += inc;
    //        }
    //    }

    //    {
    //        int steps_col = (forward - backward) / f;
    //        double inc = 1.0 / steps_col;
    //        double i = 0.0;
    //        for(int col = 0; col <= steps_col; ++col) {
    //            cells_.emplace_back(fr + i * (fl-fr));
    //            cells_.emplace_back(br + i * (bl-br));

    //            i += inc;
    //        }
    //    }
}

void RobotArea::setPosition(int x, int y)
{
    x_ = x;
    y_ = y;
    init_ = true;
}

void RobotArea::setParent(const CollisionGridMap2d *parent)
{
    parent_ = parent;
}

/**
     * @brief Start a new iteration.
     */
void RobotArea::begin() {
    idx = -1;

    assert(init_);
}

/**
     * @brief Select next cell.
     * @return False if there are no more cells.
     */
bool RobotArea::next() {
    return ++idx < cells_.size();
}

/**
     * @brief Get the cell coordinates of the currently selected cell.
     * @param x x-coordinates of the cell.
     * @param y y coordinate of the cell.
     */
void RobotArea::getCell( int& x, int& y ) const {
    //const Eigen::Vector2d& pt = centre + pts_[idx];
    const Cell& cell = cells_[idx];
    x = x_ + cell.x;
    y = y_ + cell.y;
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


CollisionGridMap2d::CollisionGridMap2d(const CollisionGridMap2d& copy)
    : RotatedGridMap2d(copy)
{
    for(int t = 0; t< ANGLE_DISCRETIZATION; ++t) {
        areas_[t] = new RobotArea(*copy.areas_[t]);
        areas_[t]->setParent(this);
    }
}

CollisionGridMap2d::~CollisionGridMap2d()
{
    for(int t = 0; t< ANGLE_DISCRETIZATION; ++t) {
        delete areas_[t];
    }
}

bool CollisionGridMap2d::isOccupied(const unsigned int x, const unsigned int y, const double theta) const
{
    if(!isInMap((int) x,(int) y)) {
        return false;
    }

    if(SimpleGridMap2d::isOccupied(x, y)) {
        return true;
    }

    double t = theta;
    while(t < 0) t += 2*M_PI;
    while(t >= 2*M_PI) t -= 2*M_PI;

    // check boundary
    RobotArea* a = areas_[(int) round(t)];
    a->setPosition(x,y);
    bool occ = isAreaOccupied(*a);
    return occ;
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
    bool no_info = isAreaNoInformation(*a);
    return no_info;
}
