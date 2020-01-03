include( CTest )

# tests that are not yet distributed across the project
file(GLOB_RECURSE tests_SRC
    "*.cpp"
)

catkin_add_gtest(${PROJECT_NAME}_test
   ${tests_SRC}
)
# set_tests_properties(${PROJECT_NAME}_test PROPERTIES TIMEOUT 60)
target_link_libraries(${PROJECT_NAME}_test
    ${catkin_LIBRARIES}
    gtest gtest_main)