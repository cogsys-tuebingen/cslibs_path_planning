/**
 * @file CollisionGridMap2d.h
 * @date Jan 2012
 * @author marks
 */

#ifndef CollisionGridMap2d_H
#define CollisionGridMap2d_H

// C/C++
#include <stdint.h>
#include <vector>

// Project
#include "RotatedGridMap2d.h"


namespace lib_path {

class RobotArea;

/**
 * @class CollisionGridMap2d
 * @brief A simple implementation of a 2d grid map.
 */
class CollisionGridMap2d : public RotatedGridMap2d {
public:
    static const int ANGLE_DISCRETIZATION = 128;

    /**
     * @brief Create a map with the given size and resolution.
     *
     * The origin will be (0,0) and the lower threshold 50, the upper threshold 200.
     *
     * @param w Number of cells in x-direction.
     * @param h Number of cells in y-direction.
     * @param r Size of one cell in meter.
     */
    CollisionGridMap2d( const unsigned int w, const unsigned int h, const double yaw, const double res, double forward, double backward, double width);
    CollisionGridMap2d( const CollisionGridMap2d& copy);
    ~CollisionGridMap2d();

    virtual bool isFree( const unsigned int x, const unsigned int y, const double theta ) const;
    virtual bool isNoInformation(const unsigned int x, const unsigned int y, const double theta) const;

private:
    RobotArea* areas_[ANGLE_DISCRETIZATION];
};


class RobotArea : public MapArea2d {
public:
    RobotArea(CollisionGridMap2d const*  parent, double forward, double backward, double width, double theta);

    void setPosition(int x, int y);
    void setParent(CollisionGridMap2d const*  parent);

    virtual void begin();
    void paint(bool free);
    virtual bool next();
    virtual void getCell( int& x, int& y ) const;
    virtual uint8_t getValue() const;
    virtual void setValue( const uint8_t value );

private:
    CollisionGridMap2d const*  parent_;
    double x_;
    double y_;
    double hw_;
    double fw_;
    double bw_;
    double theta_;
    bool init_;

    int idx;
    std::vector<Eigen::Vector2d> pts_;

    Eigen::Vector2d centre;
    Eigen::Vector2d fr, fl, br, bl;
};

} // namespace "lib_path"

#endif // CollisionGridMap2d_H
