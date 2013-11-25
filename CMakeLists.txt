cmake_minimum_required(VERSION 2.8.3)
project(utils_clustering)

find_package(catkin REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES utils_generic
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
set (LibGeneric
    Intermission.hpp
    Utils.hpp
)
add_library(utils_generic SHARED ${LibGeneric})
SET_TARGET_PROPERTIES(utils_generic PROPERTIES LINKER_LANGUAGE CXX)
