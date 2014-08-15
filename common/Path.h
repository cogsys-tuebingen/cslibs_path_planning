#ifndef PATH_H
#define PATH_H

/// COMPONENT
#include "Point2d.h"

/// SYSTEM
#include <vector>

namespace lib_path {

template <class T>
class GenericPath : public std::vector<T>
{
public:
    typedef T NodeT;

    void operator += (GenericPath<T>& rhs)
    {
        std::vector<T>::insert(this->end(), rhs.begin(), rhs.end());
    }

};

typedef GenericPath<Pose2d> Path;

}

#endif // PATH_H
