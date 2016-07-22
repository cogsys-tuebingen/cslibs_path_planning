/*
 * MapManager.hpp
 *
 *  Created on: Feb 02, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef MAPMANAGER_H
#define MAPMANAGER_H

/// COMPONENT
#include "Common.hpp"
#include "Heuristics.hpp"

/// PROJECT
#include "../common/GridMap2d.h"
#include "../common/Bresenham2d.h"

namespace lib_path
{

template <class NodeT, class MapType>
class Manager
{
public:
    typedef MapType MapT;
    typedef NodeT NodeType;

    template <class PointT>
    struct NodeHolder {
        typedef PointT NodeType;
    };

    Manager()
        : w(0), h(0), theta_slots(1), padding(0), map_(NULL)
    {}

    virtual ~Manager() {
    }

    const MapT* getMap() const {
        assert(map_ != NULL);
        return map_;
    }

    uint8_t getValue(const double sx, const double sy) const {
        return map_->getValue(sx, sy);
    }

    virtual void setMap(const MapT* map) {
        map_ = map;

        bool replace = w != map->getWidth() || h != map_->getHeight();

        w = map->getWidth();
        h = map->getHeight();

        initMap(replace);
    }

    double getResolution() const {
        return map_->getResolution();
    }

    virtual void initMap(bool replace) = 0;

    bool isFree(const NodeType* reference) const {
        //return map_->isFree(reference->x, reference->y);
        return isFree(reference->x, reference->y, reference->theta);
    }

    bool isOccupied(const NodeType* reference) const {
        return isOccupied(reference->x, reference->y, reference->theta);
    }

    bool isFree(const double sx, const double sy, const double ex, const double ey) const {
        assert(sx >= 0);
        assert(sy >= 0);
        assert(ex >= 0);
        assert(ey >= 0);

        bresenham.setGrid(map_, std::floor(sx), std::floor(sy), std::floor(ex),std::floor(ey));

        double theta = std::atan2(ey-sy,ex-sx);

        unsigned x,y;
        while(bresenham.next()) {
            bresenham.coordinates(x,y);
            if(!map_->isFree(x,y,theta)) {
                return false;
            }
//            if(bresenham.isOccupied()) {
//                return false;
//            }
        }

        return true;
    }

    bool isFreeOrUnknown(const double sx, const double sy, const double ex, const double ey) const {
        assert(sx >= 0);
        assert(sy >= 0);
        assert(ex >= 0);
        assert(ey >= 0);

        bresenham.setGrid(map_, std::floor(sx), std::floor(sy), std::floor(ex),std::floor(ey));

        double theta = std::atan2(ey-sy,ex-sx);

        unsigned x,y;
        while(bresenham.next()) {
            bresenham.coordinates(x,y);
            if(!map_->isNoInformation(x,y,theta) && !map_->isFree(x,y,theta)) {
                return false;
            }
        }

        return true;
    }

    bool isFree(int x, int y, double theta) const {
        return map_->isFree(x,y,theta);
    }

    bool isOccupied(int x, int y, double theta) const {
        return map_->isOccupied(x,y,theta);
    }


    bool contains(const int x, const int y) const {
        return map_->isInMap(x, y);
    }

    bool contains(const double x, const double y) const {
        return map_->isInMap((int) std::floor(x), (int) std::floor(y));
    }

    bool hasNode(const int x, const int y, unsigned t, bool forward) const {
        return contains(x,y);
    }

public:
    unsigned w;
    unsigned h;
    unsigned theta_slots;

    int padding;

protected:
    const MapT* map_;
    mutable Bresenham2d bresenham;

};



template <class NodeT>
struct GridMapManager :
    public Manager<NodeT, GridMap2d>
{
    typedef Manager<NodeT, GridMap2d> ManagerT;
    typedef typename ManagerT::NodeType NodeType;

    template <class PointT>
    struct NodeHolder {
        typedef PointT NodeType;
    };

    using ManagerT::w;
    using ManagerT::h;

    GridMapManager()
        : data(NULL)
    {}

    virtual ~GridMapManager() {
        delete [] data;
    }

    void initMap(bool replace) {
        bool alloc = data == NULL || replace;
        bool delete_first = alloc && data != NULL;

        if(delete_first) {
            std::cout << "delete old map" << std::endl;
            delete[] data;
        }

        if(alloc) {
            std::cout << "[GMM] allocate new map" << std::endl;
            data = new NodeType[w * h];
        }
        for(unsigned y = 0; y < h; ++y) {
            for(unsigned x = 0; x < w; ++x) {
                NodeType::init(data[index(x,y)], x, y);
            }
        }
    }

    inline unsigned index(unsigned x, unsigned y, unsigned t = 1, bool f = true) const {
        return y * w + x;
    }

    template <class PointT>
    NodeType* lookup(const PointT& reference) const {
        unsigned y = reference.y;
        unsigned x = reference.x;
        if((x < 0) || (y < 0) || (x >= w) || (y >= h)) {
            throw OutsideMapException();
        }
        return &data[index(x,y)];
    }

    NodeType* lookup(const int x, const int y) const {
        if((x < 0) || (y < 0) || (x >= w) || (y >= h)) {
            throw OutsideMapException();
        }
        return &data[index(x,y)];
    }
    NodeType* lookup(const double x, const double y, double ignored = 0) const {
        if((x < 0) || (y < 0)) {
            throw OutsideMapException();
        }
        return lookup((int) std::floor(x), (int) std::floor(y));
    }
public:
    NodeType* data;
};



template <class NodeT>
class StateSpaceManager :
    public Manager<NodeT, GridMap2d>
{
public:
    typedef Manager<NodeT, GridMap2d> ManagerT;
    typedef typename ManagerT::NodeType NodeType;

    template <class PointT>
    struct NodeHolder {
        typedef PointT NodeType;
    };

    using ManagerT::w;
    using ManagerT::h;
    using ManagerT::theta_slots;

    StateSpaceManager()
        : data(NULL)
    {}

    virtual ~StateSpaceManager() {
        delete [] data;
    }

    void initMap(bool replace) {
        theta_slots = 36;
        double stheta = 0 - M_PI;
        double dtheta = 2 * M_PI / theta_slots;

        bool alloc = data == NULL || replace;
        bool delete_first = alloc && data != NULL;

        if(delete_first) {
            std::cout << "delete old map" << std::endl;
            delete[] data;
        }

        if(alloc) {
            std::cout << "[SSM] allocate new map" << std::endl;
            data = new NodeType[w * h * theta_slots];
        }

        for(unsigned y = 0; y < h; ++y) {
            for(unsigned x = 0; x < w; ++x) {
                for(unsigned t = 0; t < theta_slots; ++t) {
                    NodeType::init(data[index(x,y,t)], x, y, stheta + dtheta * t);
                }
            }
        }
    }


    inline unsigned index(unsigned x, unsigned y, unsigned t) const {
        if((x < 0) || (y < 0) || (x >= w) || (y >= h) || (t < 0) || (t >= theta_slots)) {
            throw typename ManagerT::OutsideMapException();
        }
        return y * w + x + t*w*h;
    }
    inline unsigned angle2index(double theta) const {
        return (MathHelper::AngleClamp(theta) + M_PI) / (2 * M_PI) * (theta_slots - 1);
    }

    template <class PointT>
    NodeType* lookup(const PointT& reference) const {
        unsigned y = reference.y;
        unsigned x = reference.x;
        unsigned t = angle2index(reference.theta);
        return &data[index(x,y,t)];
    }

    NodeType* lookup(const int x, const int y, double theta) const {
        if((x < 0) || (y < 0) || (x >= w) || (y >= h)) {
            throw typename ManagerT::OutsideMapException();
        }
        return &data[index(x,y,angle2index(theta))];
    }
    NodeType* lookup(const double x, const double y, double theta) const {
        if((x < 0) || (y < 0) || (x >= w) || (y >= h)) {
            throw typename ManagerT::OutsideMapException();
        }
        return lookup((int) std::floor(x), (int) std::floor(y), theta);
    }

public:
    NodeType* data;
};



template <class NodeT>
class DirectionalStateSpaceManager :
    public Manager<NodeT, GridMap2d>
{
public:
    typedef Manager<NodeT, GridMap2d> ManagerT;
    typedef typename ManagerT::NodeType NodeType;

    using ManagerT::w;
    using ManagerT::h;
    using ManagerT::theta_slots;

    DirectionalStateSpaceManager()
        : data(NULL)
    {}

    virtual ~DirectionalStateSpaceManager() {
        delete [] data;
    }

    template <class PointT>
    struct NodeHolder {
        typedef DirectionalNode<PointT> NodeType;
    };

    void initMap(bool replace) {

        theta_slots = 32;
        double stheta = 0 - M_PI;
        double dtheta = 2 * M_PI / theta_slots;

        dimension = w * h * theta_slots;

        bool alloc = data == NULL || replace;
        bool delete_first = alloc && data != NULL;

        if(delete_first) {
            std::cout << "delete old map" << std::endl;
            delete[] data;
        }

        if(alloc) {
            std::cout << "[DSSM] allocate new map of size " << (2 * dimension * sizeof(NodeType) / 1e6) << " MB" << std::endl;
            data = new NodeType[2 * dimension];
        }


        for(unsigned y = 0; y < h; ++y) {
            for(unsigned x = 0; x < w; ++x) {
                for(unsigned t = 0; t < theta_slots; ++t) {
                    for(unsigned f = 0; f <= 1; ++f) {
                        NodeType::init(data[index(x,y,t, f)], x, y, stheta + dtheta * t);
                    }
                }
            }
        }
    }


    inline unsigned index(int x, int y, int t=0, bool forward=true) {
        if((x < 0) || (y < 0) || (x >= (int) w) || (y >= (int) h) || (t < 0) || (t >= (int) theta_slots)) {
            throw typename ManagerT::OutsideMapException();
        }

        return y * w + x + t*w*h + (forward ? 0 : dimension);

    }
    inline unsigned angle2index(double theta) {
        return (MathHelper::AngleClamp(theta) + M_PI) / (2 * M_PI) * (theta_slots - 1);
    }

    template <class PointT>
    NodeType* lookup(const PointT& reference) const {
        unsigned y = reference.y;
        unsigned x = reference.x;
        unsigned t = angle2index(reference.theta);
        return &data[index(x,y,t, reference.forward)];
    }
    NodeType* lookup(const Pose2d& reference) const {
        unsigned y = reference.y;
        unsigned x = reference.x;
        unsigned t = angle2index(reference.theta);
        return &data[index(x,y,t, true)];
    }

    NodeType* lookup(const int x, const int y, double theta, bool forward) const {
        int t = angle2index(theta);
        if((x < 0) || (y < 0) || (x >= (int) w) || (y >= (int) h) || (t < 0) || (t >= (int) theta_slots)) {
            throw typename ManagerT::OutsideMapException();
        }
        return &data[index(x,y,t, forward)];
    }
    NodeType* lookup(const double x, const double y, double theta, bool forward) const {
        if((x < 0) || (y < 0) || (x >= w) || (y >= h)) {
            throw typename ManagerT::OutsideMapException();
        }
        return lookup((int) std::floor(x), (int) std::floor(y), theta, forward);
    }

private:
    unsigned dimension;

public:
    NodeType* data;
};




/**
 * This state space manager lazily allocates memory in chunks instead of a monolithic block.
 */
template <class NodeT>
class DynamicDirectionalStateSpaceManager
{
public:
    typedef GridMap2d MapT;
    typedef NodeT NodeType;

    class Chunk
    {
    public:
        Chunk(int cx, int cy, unsigned size, unsigned theta_slots)
            : ox(cx), oy(cy), chunk_size(size), chunk_dimension(chunk_size * chunk_size * theta_slots), chunk_data(new NodeType[2*chunk_dimension])
        {
            double stheta = 0 - M_PI;
            double dtheta = 2 * M_PI / theta_slots;

            for(unsigned y = 0; y < chunk_size; ++y) {
                for(unsigned x = 0; x < chunk_size; ++x) {
                    for(unsigned t = 0; t < theta_slots; ++t) {
                        for(unsigned f = 0; f <= 1; ++f) {
                            int nx = ox+x;
                            int ny = oy+y;
                            std::size_t ci = chunk_index(nx, ny, t, f);
                            NodeType::init(chunk_data[ci], nx, ny, stheta + dtheta * t);
                        }
                    }
                }
            }
        }

        ~Chunk()
        {
            delete [] chunk_data;
        }

        inline unsigned chunk_index(int mx, int my, int t=0, bool forward=true) const {
            int cx = mx - ox;
            int cy = my - oy;
            if((cx < 0) || (cy < 0) || (cx >= (int) chunk_size) || (cy >= (int) chunk_size)) {
                throw OutsideMapException();
            }

            return cy * chunk_size + cx + t*chunk_size*chunk_size + (forward ? 0 : chunk_dimension);
        }

        inline NodeType* lookup(int mx, int my, int t=0, bool forward=true) const {
            std::size_t index = chunk_index(mx,my,t,forward);
            return &chunk_data[index];
        }

    private:
        int ox;
        int oy;

        unsigned chunk_size;

        unsigned chunk_dimension;
        NodeType* chunk_data;
    };


    DynamicDirectionalStateSpaceManager()
        : chunks_w(0), chunks_h(0)
    {
        chunk_size = 32;
        theta_slots = 32;

        padding_chunks = 1;
        padding = padding_chunks * chunk_size;
    }

     ~DynamicDirectionalStateSpaceManager() {
        clearChunks();
    }

    void setMap(const MapT* map) {
        map_ = map;

        bool replace = w != map->getWidth() || h != map_->getHeight();

        w = map->getWidth();
        h = map->getHeight();

        initMap(replace);
    }

    const MapT* getMap() {
        assert(map_ != NULL);
        return map_;
    }
    double getResolution() {
        return map_->getResolution();
    }

    uint8_t getValue(const double sx, const double sy) {
        return map_->getValue(sx, sy);
    }

    void clearChunks() noexcept
    {
        for(Chunk* c : chunks) {
            try {
                delete c;
            } catch(...) {
                // ignore...
            }
        }
        chunks.clear();
    }

    template <class PointT>
    struct NodeHolder {
        typedef DirectionalNode<PointT> NodeType;
    };

    void initMap(bool /*replace*/) {

        int ww = w + 2 * padding;
        int hh = h + 2 * padding;

        chunks_w = ww / chunk_size + 1;
        chunks_h = hh / chunk_size + 1;

        dimension = ww * hh * theta_slots;


        clearChunks();

        chunks.resize(chunks_w * chunks_h);
        memset(chunks.data(), 0, chunks.size() * sizeof(Chunk*));
    }


    inline unsigned angle2index(double theta) const {
        return (MathHelper::AngleClamp(theta) + M_PI) / (2 * M_PI) * (theta_slots - 1);
    }

    template <class PointT>
    NodeType* lookup(const PointT& reference) const {
        return lookup(reference.x,reference.y,reference.theta, reference.forward);
    }
    NodeType* lookup(const Pose2d& reference) const {
        return lookup(reference.x,reference.y,reference.theta, true);
    }

    NodeType* lookup(const int x, const int y, double theta, bool forward) const {
        return lookup(x,y,angle2index(theta), forward);
    }
    NodeType* lookup(const double x, const double y, double theta, bool forward) const {
        return lookup((int) std::floor(x), (int) std::floor(y), theta, forward);
    }
    NodeType* lookup(const int x, const int y, unsigned t, bool forward) const {
        if((x < -padding) || (y < -padding) || (x >= (int) (w + padding)) || (y >= (int) (h + padding)) || (t < 0) || (t >= (int) theta_slots)) {
            throw OutsideMapException();
        }
        int cx = (x / (int) chunk_size);
        if(x < 0) cx -= 1;
        int cy = (y / (int) chunk_size);
        if(y < 0) cy -= 1;

        int cxp = cx + padding_chunks;
        int cyp = cy + padding_chunks;


        Chunk* chunk = chunks.at(cyp * chunks_w + cxp);

        if(!chunk) {
            chunk = new Chunk(cx * chunk_size, cy * chunk_size, chunk_size, theta_slots);
            chunks.at(cyp * chunks_w + cxp) = chunk;
        }

        return chunk->lookup(x,y ,t, forward);
    }


    bool hasNode(const int x, const int y, unsigned t, bool forward) const {
        if((x < -padding) || (y < -padding) || (x >= (int) (w + padding)) || (y >= (int) (h + padding)) || (t < 0) || (t >= (int) theta_slots)) {
            std::cerr << "! has node: " << x << " / " << y << std::endl;
            throw OutsideMapException();
        }
        int cx = (x / chunk_size);
        int cy = (y / chunk_size);

        Chunk* chunk = chunks.at(cy * chunks_w + cx);
        return chunk != nullptr;
    }


    bool contains(const int x, const int y) const {
        if((x < -padding) || (y < -padding) || (x >= (int) (w + padding)) || (y >= (int) (h + padding))) {
            return false;
        }
        return true;
    }

    bool contains(const double wx, const double wy) const {
        int x = std::floor(wx);
        int y = std::floor(wy);

        if((x < -padding) || (y < -padding) || (x >= (int) (w + padding)) || (y >= (int) (h + padding))) {
            return false;

        }
        return true;
    }

    bool isFree(const double sx, const double sy, const double ex, const double ey) const {
        bresenham.setGrid(map_, std::floor(sx + padding), std::floor(sy + padding), std::floor(ex + padding),std::floor(ey + padding));

        double theta = std::atan2(ey-sy,ex-sx);

        unsigned x,y;
        while(bresenham.next()) {
            bresenham.coordinates(x,y);
            if(!isFree(x-padding,y-padding,theta)) {
                return false;
            }
//            if(bresenham.isOccupied()) {
//                return false;
//            }
        }

        return true;
    }

    bool isFreeOrUnknown(const double sx, const double sy, const double ex, const double ey) const {
        bresenham.setGrid(map_, std::floor(sx + padding), std::floor(sy + padding), std::floor(ex + padding),std::floor(ey + padding));

        double theta = std::atan2(ey-sy,ex-sx);

        unsigned x,y;
        while(bresenham.next()) {
            bresenham.coordinates(x,y);
            if(!isNoInformation(x-padding,y-padding,theta) && !isFree(x-padding,y-padding,theta)) {
                return false;
            }
        }

        return true;
    }

    bool isFree(int x, int y, double theta) const {
        if((x < 0) || (y < 0) || (x >= (int) (w)) || (y >= (int) (h))) {
            return false;
        }
        return map_->isFree(x,y,theta);
    }
    bool isNoInformation(int x, int y, double theta) const {
        if((x < 0) || (y < 0) || (x >= (int) (w)) || (y >= (int) (h))) {
            return true;
        }
        return map_->isNoInformation(x,y,theta);
    }

    bool isOccupied(const NodeType* reference) const {
        return isOccupied(reference->x, reference->y, reference->theta);
    }
    bool isOccupied(int x, int y, double theta) const {
        if((x < 0) || (y < 0) || (x >= (int) (w)) || (y >= (int) (h))) {
            return false;
        }
        return map_->isOccupied(x,y,theta);
    }

public:
    unsigned w;
    unsigned h;
    unsigned theta_slots;

    int padding_chunks;
    int padding;

protected:
    const MapT* map_;
    mutable Bresenham2d bresenham;

private:
    unsigned dimension;
    unsigned chunk_size;


    std::size_t chunks_w;
    std::size_t chunks_h;

public:
    mutable std::vector<Chunk*> chunks;
};
}

#endif // MAPMANAGER_H
