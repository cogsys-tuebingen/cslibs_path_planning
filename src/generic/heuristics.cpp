/// HEADER
#include <cslibs_path_planning/generic/NodeManager.hpp>
#include <cslibs_path_planning/generic/Heuristics.hpp>

using namespace lib_path;

double* HeuristicHolonomicObstacles::costs;
unsigned HeuristicHolonomicObstacles::w;
unsigned HeuristicHolonomicObstacles::h;
int HeuristicHolonomicObstacles::padding;
