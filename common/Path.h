#ifndef PATH_H
#define PATH_H

/// COMPONENT
#include "Point2d.h"

/// SYSTEM
#include <vector>

namespace lib_path {

typedef std::vector<Pose2d> Path;

template <typename Node>
inline void operator += (std::vector<Node>& lhs, const std::vector<Node>& rhs)
{
    lhs.insert(lhs.end(), rhs.begin(), rhs.end());
}

}

#endif // PATH_H
