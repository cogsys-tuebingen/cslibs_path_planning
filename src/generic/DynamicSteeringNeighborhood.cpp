/// HEADER
#include <cslibs_path_planning/generic/DynamicSteeringNeighborhood.h>

using namespace lib_path;


double DynamicSteeringNeighborhood::resolution = 1;
bool DynamicSteeringNeighborhood::reversed = false;
double DynamicSteeringNeighborhood::distance_step = 0.4;
double DynamicSteeringNeighborhood::distance_step_pixel = 1;

int DynamicSteeringNeighborhood::MAX_STEER_ANGLE = 60;
double DynamicSteeringNeighborhood::STEER_DELTA = 15;
unsigned int DynamicSteeringNeighborhood::steer_steps = 2;
double DynamicSteeringNeighborhood::LA = 1.20;

bool DynamicSteeringNeighborhood::allow_forward = true;
bool DynamicSteeringNeighborhood::allow_backward = false;

double DynamicSteeringNeighborhood::goal_dist_threshold = 0.05;
double DynamicSteeringNeighborhood::goal_angle_threshold = (180 / 180.0 * M_PI) / 8;
