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
    struct OutsideMapException : public std::exception
    {
    };

public:
    typedef MapType MapT;
    typedef NodeT NodeType;

    template <class PointT>
    struct NodeHolder {
        typedef PointT NodeType;
    };

    Manager()
        : w(0), h(0), theta_slots(1), map_(NULL)
    {}

    virtual ~Manager() {
    }

    const MapT* getMap() {
        assert(map_ != NULL);
        return map_;
    }

    uint8_t getValue(const double sx, const double sy) {
        return map_->getValue(sx, sy);
    }

    void setMap(const MapT* map) {
        map_ = map;

        bool replace = w != map->getWidth() || h != map_->getHeight();

        w = map->getWidth();
        h = map->getHeight();

        initMap(replace);
    }

    double getResolution() {
        return map_->getResolution();
    }

    virtual void initMap(bool replace) = 0;

    bool isFree(const NodeType* reference) {
        //return map_->isFree(reference->x, reference->y);
        return isFree(reference->x, reference->y, reference->theta);
    }
    bool isFree(int x, int y, double theta) {
        return map_->isFree(x,y,theta);
    }


    bool isOccupied(const NodeType* reference) {
        return isOccupied(reference->x, reference->y, reference->theta);
    }
    bool isOccupied(int x, int y, double theta) {
        return map_->isOccupied(x,y,theta);
    }

    bool isFree(const double sx, const double sy, const double ex, const double ey) {
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

    bool isFreeOrUnknown(const double sx, const double sy, const double ex, const double ey) {
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

    bool contains(const int x, const int y) {
        return map_->isInMap(x, y);
    }

    bool contains(const double x, const double y) {
        return map_->isInMap((int) std::floor(x), (int) std::floor(y));
    }

    virtual bool hasNode(const int x, const int y, unsigned t, bool forward) {
        return contains(x,y);
    }

public:
    unsigned w;
    unsigned h;
    unsigned theta_slots;

protected:
    const MapT* map_;
    Bresenham2d bresenham;

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

    inline unsigned index(unsigned x, unsigned y, unsigned t = 1, bool f = true) {
        return y * w + x;
    }

    template <class PointT>
    NodeType* lookup(const PointT& reference) {
        unsigned y = reference.y;
        unsigned x = reference.x;
        if((x < 0) || (y < 0) || (x >= w) || (y >= h)) {
            throw typename ManagerT::OutsideMapException();
        }
        return &data[index(x,y)];
    }

    NodeType* lookup(const int x, const int y) {
        if((x < 0) || (y < 0) || (x >= w) || (y >= h)) {
            throw typename ManagerT::OutsideMapException();
        }
        return &data[index(x,y)];
    }
    NodeType* lookup(const double x, const double y, double ignored = 0) {
        if((x < 0) || (y < 0)) {
            throw typename ManagerT::OutsideMapException();
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


    inline unsigned index(unsigned x, unsigned y, unsigned t) {
        if((x < 0) || (y < 0) || (x >= w) || (y >= h) || (t < 0) || (t >= theta_slots)) {
            throw typename ManagerT::OutsideMapException();
        }
        return y * w + x + t*w*h;
    }
    inline unsigned angle2index(double theta) {
        return (MathHelper::AngleClamp(theta) + M_PI) / (2 * M_PI) * (theta_slots - 1);
    }

    template <class PointT>
    NodeType* lookup(const PointT& reference) {
        unsigned y = reference.y;
        unsigned x = reference.x;
        unsigned t = angle2index(reference.theta);
        return &data[index(x,y,t)];
    }

    NodeType* lookup(const int x, const int y, double theta) {
        if((x < 0) || (y < 0) || (x >= w) || (y >= h)) {
            throw typename ManagerT::OutsideMapException();
        }
        return &data[index(x,y,angle2index(theta))];
    }
    NodeType* lookup(const double x, const double y, double theta) {
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
    NodeType* lookup(const PointT& reference) {
        unsigned y = reference.y;
        unsigned x = reference.x;
        unsigned t = angle2index(reference.theta);
        return &data[index(x,y,t, reference.forward)];
    }
    NodeType* lookup(const Pose2d& reference) {
        unsigned y = reference.y;
        unsigned x = reference.x;
        unsigned t = angle2index(reference.theta);
        return &data[index(x,y,t, true)];
    }

    NodeType* lookup(const int x, const int y, double theta, bool forward) {
        int t = angle2index(theta);
        if((x < 0) || (y < 0) || (x >= (int) w) || (y >= (int) h) || (t < 0) || (t >= (int) theta_slots)) {
            throw typename ManagerT::OutsideMapException();
        }
        return &data[index(x,y,t, forward)];
    }
    NodeType* lookup(const double x, const double y, double theta, bool forward) {
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
class DynamicDirectionalStateSpaceManager :
    public Manager<NodeT, GridMap2d>
{
public:
    typedef Manager<NodeT, GridMap2d> ManagerT;
    typedef typename ManagerT::NodeType NodeType;

    using ManagerT::w;
    using ManagerT::h;
    using ManagerT::theta_slots;

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

        inline unsigned chunk_index(int mx, int my, int t=0, bool forward=true) {
            int cx = mx - ox;
            int cy = my - oy;
            if((cx < 0) || (cy < 0) || (cx >= (int) chunk_size) || (cy >= (int) chunk_size)) {
                throw typename ManagerT::OutsideMapException();
            }

            return cy * chunk_size + cx + t*chunk_size*chunk_size + (forward ? 0 : chunk_dimension);
        }

        inline NodeType* lookup(int mx, int my, int t=0, bool forward=true) {
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
    {}

    virtual ~DynamicDirectionalStateSpaceManager() {
        clearChunks();
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

    void initMap(bool replace) {
        chunk_size = 32;
        theta_slots = 32;

        chunks_w = w / chunk_size + 1;
        chunks_h = h / chunk_size + 1;

        dimension = w * h * theta_slots;


        clearChunks();

        chunks.resize(chunks_w * chunks_h);
        memset(chunks.data(), 0, chunks.size() * sizeof(Chunk*));
    }


    inline unsigned angle2index(double theta) {
        return (MathHelper::AngleClamp(theta) + M_PI) / (2 * M_PI) * (theta_slots - 1);
    }

    template <class PointT>
    NodeType* lookup(const PointT& reference) {
        return lookup(reference.x,reference.y,reference.theta, reference.forward);
    }
    NodeType* lookup(const Pose2d& reference) {
        return lookup(reference.x,reference.y,reference.theta, true);
    }

    NodeType* lookup(const int x, const int y, double theta, bool forward) {
        return lookup(x,y,angle2index(theta), forward);
    }
    NodeType* lookup(const double x, const double y, double theta, bool forward) {
        return lookup((int) std::floor(x), (int) std::floor(y), theta, forward);
    }
    NodeType* lookup(const int x, const int y, unsigned t, bool forward) {
        if((x < 0) || (y < 0) || (x >= (int) w) || (y >= (int) h) || (t < 0) || (t >= (int) theta_slots)) {
            throw typename ManagerT::OutsideMapException();
        }
        int cx = (x / chunk_size);
        int cy = (y / chunk_size);

        Chunk* chunk = chunks.at(cy * chunks_w + cx);

        if(!chunk) {
            std::cerr << "allocating new chunk " << cx << ", " << cy << " of size " << chunk_size << " x " << chunk_size << std::endl;
            chunk = new Chunk(cx * chunk_size, cy * chunk_size, chunk_size, theta_slots);
            chunks.at(cy * chunks_w + cx) = chunk;
        }

        return chunk->lookup(x,y,t, forward);
    }


    bool hasNode(const int x, const int y, unsigned t, bool forward) {
        if((x < 0) || (y < 0) || (x >= (int) w) || (y >= (int) h) || (t < 0) || (t >= (int) theta_slots)) {
            throw typename ManagerT::OutsideMapException();
        }
        int cx = (x / chunk_size);
        int cy = (y / chunk_size);

        Chunk* chunk = chunks.at(cy * chunks_w + cx);
        return chunk != nullptr;
    }

private:
    unsigned dimension;
    unsigned chunk_size;

    std::size_t chunks_w;
    std::size_t chunks_h;

public:
    std::vector<Chunk*> chunks;
};
}

#endif // MAPMANAGER_H
