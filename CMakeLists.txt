cmake_minimum_required(VERSION 2.8.3)
project(cslibs_path_planning)

add_definitions(-W -Wall -Wno-unused-parameter -fno-strict-aliasing -Wno-unused-function -Wno-deprecated-register)

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

find_package(catkin REQUIRED COMPONENTS cslibs_navigation_utilities)

find_package(Boost COMPONENTS program_options system filesystem thread REQUIRED)
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS cslibs_navigation_utilities
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
    src/generic/DynamicSteeringNeighborhood.cpp
    src/sampling/GoalRegion.cpp
    src/sampling/RingGoalRegion.cpp
    src/sampling/CentroidRadiusGoalRegion.cpp
    src/sampling/SamplingPlanner.cpp
    )
if()
    set(SOURCES ${SOURCES}
        src/common/CollisionGridMap2d.cpp
        )
endif()


#
# LIST EVERYTHING IN QT
#
file(GLOB_RECURSE ALL_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp include/*.h)
add_custom_target(${PROJECT_NAME}_show_all_headers_in_qtc SOURCES ${ALL_HEADERS})


add_library(${PROJECT_NAME}  ${SOURCES})

if(OpenCV_FOUND)
    target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
endif()



target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#target_link_libraries(ReedsShepp    LibUtil)

#install(TARGETS ReedsShepp LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

#if(NOT OpenCV_FOUND)
#    message( "OpenCV NOT found, not building test program" )

#else()
#    add_executable (CostPrecomputation
#        src/ReedsShepp/CostPrecomputation.cpp
#        src/ReedsShepp/CurveRenderer.cpp)
#    target_link_libraries (CostPrecomputation
#        ${PROJECT_NAME}
#        ${catkin_LIBRARIES}
#        ${OpenCV_LIBS}
#        ${Boost_LIBRARIES}
#        )


#    add_executable (ReedsSheppCurveTest
#        src/ReedsShepp/CurveTest.cpp
#        src/ReedsShepp/CurveRenderer.cpp)
#    target_link_libraries (ReedsSheppCurveTest
#        ${PROJECT_NAME}
#        ${catkin_LIBRARIES}
#        ${OpenCV_LIBS}
#        )
#    add_executable (Path4wsSimulator
#        src/common/path4ws_simulator.cpp
#        src/common/path4ws_sim.cpp
#        )
#    target_link_libraries (Path4wsSimulator
#        ${PROJECT_NAME}
#        ${catkin_LIBRARIES}
#        ${OpenCV_LIBS}
#        )
#    add_executable (Evaluator
#        src/evaluation/evaluator.cpp)
#    target_link_libraries (Evaluator
#        pthread
#        ${PROJECT_NAME}
#        ${catkin_LIBRARIES}
#        ${Boost_LIBRARIES}
#        ${OpenCV_LIBS}
#        ${YAML_LIBRARIES})

#    add_executable (Geomtest
#        src/geometry/test_geometry.cpp)
#    target_link_libraries (Geomtest
#        ${PROJECT_NAME}
#        ${catkin_LIBRARIES}
#        ${Boost_LIBRARIES}
#        ${OpenCV_LIBS}
#        ${YAML_LIBRARIES})

#    install(TARGETS CostPrecomputation
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#    install(TARGETS ReedsSheppCurveTest
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#    install(TARGETS Path4wsSimulator
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#    install(TARGETS Evaluator
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#    install(TARGETS Geomtest
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#endif()

#add_executable(unit_tests ReedsShepp/UnitTest.cpp)

#target_link_libraries (unit_tests
#    LibUtil
#    ReedsShepp
#)




#
# INSTALL
#

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
