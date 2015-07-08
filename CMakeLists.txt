cmake_minimum_required(VERSION 2.8.3)
project(utils_path)

## Enforce that we use C++11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS utils_general utils_generic cmake_modules)

find_package(Boost COMPONENTS program_options system filesystem thread REQUIRED)
catkin_package(
    INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/..
    LIBRARIES utils_path
    CATKIN_DEPENDS utils_generic utils_general
)

find_package(OpenCV)
set(YAML_LIBRARIES -L/usr/local/lib/ yaml-cpp)

set(CMAKE_BUILD_TYPE Release)

find_package(Eigen QUIET)
if(NOT ${Eigen_FOUND})
  find_package(Eigen3 REQUIRED)
endif()
include_directories(
  ${EIGEN_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

set(SOURCES ReedsShepp/CurveGenerator.cpp
    ReedsShepp/Curve.cpp
    ReedsShepp/CurveSegment.cpp
    ReedsShepp/CircleSegment.cpp
    ReedsShepp/LineSegment.cpp
    ReedsShepp/GeometryHelper.cpp
    common/MapMath.cpp
    common/Point2d.cpp
    common/Pose2d.cpp
    common/Path.cpp
    common/GridMap2d.cpp
    common/SimpleGridMap2d.cpp
    common/CollisionGridMap2d.cpp
    common/Bresenham2d.cpp
    a_star/AStar.cpp
    a_star/Node.cpp
    a_star/OpenList.cpp
    generic/Common.hpp
    generic/SearchAlgorithm.hpp
    generic/Algorithms.hpp
    generic/DistanceTransformation.hpp
    generic/Heuristics.hpp
    generic/DirectNeighborhood.hpp
    generic/NonHolonomicNeighborhood.hpp
    generic/MapManager.hpp
    generic/NodeManager.hpp
    generic/ReedsSheppExpansion.hpp
    geometry/circle.cpp
    geometry/intersector.cpp
    geometry/line.cpp
    geometry/tangentor.cpp
    evaluation/Renderer.hpp
    evaluation/Util.hpp
    evaluation/MapGenerators.hpp
    sampling/GoalRegion.cpp
    sampling/RingGoalRegion.cpp
    #sampling/PoseListGoalRegion.cpp
    sampling/CentroidRadiusGoalRegion.cpp
    sampling/SamplingPlanner.cpp
)
if(OpenCV_FOUND)
    set(SOURCES ${SOURCES}
        common/CollisionGridMap2d.cpp
    )
endif(OpenCV_FOUND)

add_library(utils_path  ${SOURCES})

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
                             ReedsShepp/CostPrecomputation.cpp
                             ReedsShepp/CurveRenderer.cpp)
    target_link_libraries (CostPrecomputation
        utils_path
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
    )


    add_executable (ReedsSheppCurveTest
                             ReedsShepp/CurveTest.cpp
                             ReedsShepp/CurveRenderer.cpp)
    target_link_libraries (ReedsSheppCurveTest
        utils_path
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
    )
    add_executable (Path4wsSimulator
                             common/path4ws_simulator.cpp
                             common/path4ws_sim.cpp
    )
    target_link_libraries (Path4wsSimulator
        utils_path
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
    )
    add_executable (Evaluator
                             evaluation/evaluator.cpp)
    target_link_libraries (Evaluator
        utils_path
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        ${YAML_LIBRARIES})

    add_executable (Geomtest
                             geometry/test_geometry.cpp)
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
