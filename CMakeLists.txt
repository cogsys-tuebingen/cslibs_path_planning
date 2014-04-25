cmake_minimum_required(VERSION 2.8.3)
project(utils_generic)

find_package(catkin REQUIRED)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
set (LibGeneric
    include/utils_generic/Intermission.hpp
    include/utils_generic/Utils.hpp
)
add_custom_target(utils_generic SOURCES ${LibGeneric})



install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

