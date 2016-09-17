cmake_minimum_required(VERSION 2.8.3)
project(utils_path)

## Enforce that we use C++11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  add_definitions(-std=c++11)
elseif(COMPILER_SUPPORTS_CXX0X)
  add_definitions(-std=c++0x)
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS utils_general utils_generic cmake_modules)

find_package(Boost COMPONENTS program_options system filesystem thread REQUIRED)
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES utils_path
    CATKIN_DEPENDS utils_generic utils_general
)

find_package(OpenCV)
set(YAML_LIBRARIES -L/usr/local/lib/ yaml-cpp)

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

set(SOURCES
    src/ReedsShepp/CurveGenerator.cpp
    src/ReedsShepp/Curve.cpp
    src/ReedsShepp/CurveSegment.cpp
    src/ReedsShepp/CircleSegment.cpp
    src/ReedsShepp/LineSegment.cpp
    src/ReedsShepp/GeometryHelper.cpp
    src/common/MapMath.cpp
    src/common/Point2d.cpp
    src/common/Pose2d.cpp
    src/common/Path.cpp
    src/common/GridMap2d.cpp
    src/common/SimpleGridMap2d.cpp
    src/common/RotatedGridMap2d.cpp
    src/common/CollisionGridMap2d.cpp
    src/common/Bresenham2d.cpp
    src/a_star/AStar.cpp
    src/a_star/Node.cpp
    src/a_star/OpenList.cpp
    src/geometry/circle.cpp
    src/geometry/intersector.cpp
    src/geometry/line.cpp
    src/geometry/shape.cpp
    src/geometry/tangentor.cpp
    src/generic/heuristics.cpp
    src/sampling/GoalRegion.cpp
    src/sampling/RingGoalRegion.cpp
    #src/sampling/PoseListGoalRegion.cpp
    src/sampling/CentroidRadiusGoalRegion.cpp
    src/sampling/SamplingPlanner.cpp
)
if(OpenCV_FOUND)
    set(SOURCES ${SOURCES}
        src/common/CollisionGridMap2d.cpp
    )
endif(OpenCV_FOUND)

#
# LIST EVERYTHING IN QT
#
file(GLOB_RECURSE ALL_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp include/*.h)


add_library(utils_path  ${SOURCES} ${ALL_HEADERS})

if(OpenCV_FOUND)
    target_link_libraries(utils_path ${OpenCV_LIBRARIES})
endif(OpenCV_FOUND)



target_link_libraries(utils_path ${catkin_LIBRARIES})

#target_link_libraries(ReedsShepp    LibUtil)

#install(TARGETS ReedsShepp LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

if(NOT OpenCV_FOUND)
    message( "OpenCV NOT found, not building test program" )

else(NOT OpenCV_FOUND)


    add_executable (CostPrecomputation
                             src/ReedsShepp/CostPrecomputation.cpp
                             src/ReedsShepp/CurveRenderer.cpp)
    target_link_libraries (CostPrecomputation
        utils_path
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
    )


    add_executable (ReedsSheppCurveTest
                             src/ReedsShepp/CurveTest.cpp
                             src/ReedsShepp/CurveRenderer.cpp)
    target_link_libraries (ReedsSheppCurveTest
        utils_path
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
    )
    add_executable (Path4wsSimulator
                             src/common/path4ws_simulator.cpp
                             src/common/path4ws_sim.cpp
    )
    target_link_libraries (Path4wsSimulator
        utils_path
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
    )
    add_executable (Evaluator
                             src/evaluation/evaluator.cpp)
    target_link_libraries (Evaluator
        pthread
        utils_path
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        ${YAML_LIBRARIES})

    add_executable (Geomtest
                             src/geometry/test_geometry.cpp)
    target_link_libraries (Geomtest
        utils_path
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        ${YAML_LIBRARIES})



 #   install(TARGETS ReedsSheppCurveTest RUNTIME DESTINATION bin)
endif(NOT OpenCV_FOUND)

#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS_BAK})

#add_executable(unit_tests ReedsShepp/UnitTest.cpp)

#target_link_libraries (unit_tests
#    LibUtil
#    ReedsShepp
#)
