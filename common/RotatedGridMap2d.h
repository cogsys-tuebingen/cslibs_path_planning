#ifndef ROTATED_GRID_MAP_2D_H
#define ROTATED_GRID_MAP_2D_H

/// COMPONENT
#include "SimpleGridMap2d.h"

/// SYSTEM
#include <stdint.h>
#include <vector>


namespace lib_path {

/**
 * @class RotatedGridMap2d
 * @brief An implementation of a 2d grid map that can be rotated.
 */
class RotatedGridMap2d : public SimpleGridMap2d {
public:

    /**
     * @brief Create a map with the given size and resolution.
     *
     * The origin will be (0,0) and the lower threshold 50, the upper threshold 200.
     *
     * @param w Number of cells in x-direction.
     * @param h Number of cells in y-direction.
     * @param r Size of one cell in meter.
     */
    RotatedGridMap2d( const unsigned int w, const unsigned int h, const double yaw, const double r );

    virtual ~RotatedGridMap2d() { /* Nothing to do */ }

    /**
     * @brief Set new map data.
     * @param data The new map data. Size should be >= w * h
     * @param w New map width (number of cells in x-direction)
     * @param h new map height (number of cells in y-direction)
     */
    void set( const std::vector<uint8_t>& data, const unsigned int w, const unsigned int h, const double yaw ) {
        width_ = w;
        height_ = h;
        yaw_ = yaw;
        data_.assign( data.begin(), data.end());
    }

    void cell2point( const unsigned int x, const unsigned int y, float& px, float& py ) const {
        double nx = res_*(double)(x+0.5);
        double ny = res_*(double)(y+0.5);

        double c = std::cos(yaw_);
        double s = std::sin(yaw_);

        px = c * nx - s * ny + origin_.x;
        py = s * nx + c * ny + origin_.y;
    }


    bool point2cell( const double px, const double py, unsigned int& x, unsigned int& y ) const {
        int nx = (px - origin_.x)/res_;
        int ny = (py - origin_.y)/res_;

        double c = std::cos(yaw_);
        double s = std::sin(yaw_);
        x = (unsigned int)(c * nx - s * ny);
        y = (unsigned int)(s * nx + c * ny);

        std::cerr << "angle is " << yaw_ << ", point is " << px << ", " << py << ", cell is " << x << ", " << y << std::endl;

        if ( !isInMap( (int)x, (int)y ))
            return false;
        return true;
    }

    void cell2point( const unsigned int x, const unsigned int y, double& px, double& py ) const {
        double nx = res_*(double)(x+0.5);
        double ny = res_*(double)(y+0.5);

        double c = std::cos(yaw_);
        double s = std::sin(yaw_);

        px = c * nx - s * ny + origin_.x;
        py = s * nx + c * ny + origin_.y;
    }

    virtual void cell2pointSubPixel( const double x, const double y, double& px, double& py ) const {
        double nx = res_ * x;
        double ny = res_ * y;

        double c = std::cos(yaw_);
        double s = std::sin(yaw_);
        px = c * nx - s * ny + origin_.x;
        py = s * nx + c * ny + origin_.y;
    }

    bool isInMap( const double x, const double y ) const {
        int nx = (x - origin_.x)/res_;
        int ny = (y - origin_.y)/res_;

        double c = std::cos(yaw_);
        double s = std::sin(yaw_);
        int px = c * nx - s * ny;
        int py = s * nx + c * ny;

        return px >= 0 && py >= 0 && px < (int) width_ && py < (int) height_;
    }

protected:
    /// Rotation of the map
    double yaw_;
};

} // namespace "lib_path"

#endif // ROTATED_GRID_MAP_2D_H
