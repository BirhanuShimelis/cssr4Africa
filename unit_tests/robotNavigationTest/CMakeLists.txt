
# # Add the executables
add_executable        (robotNavigationTest src/robotNavigationTestApplication.cpp src/robotNavigationTestImplementation.cpp )
set_target_properties (robotNavigationTest PROPERTIES OUTPUT_NAME robotNavigationTest PREFIX "")
target_link_libraries (robotNavigationTest ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
target_include_directories(robotNavigationTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
add_dependencies(robotNavigationTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # Add the executables for the stub and driver
# add_executable        (exampleComponentStub src/exampleComponentStub.cpp)
# set_target_properties (exampleComponentStub PROPERTIES OUTPUT_NAME exampleComponentStub PREFIX "")
# target_link_libraries (exampleComponentStub ${catkin_LIBRARIES})
# target_include_directories(exampleComponentStub PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
# add_dependencies(exampleComponentStub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # Add the executables for the stub and driver
add_executable        (robotNavigationDriver src/robotNavigationDriver.cpp)
set_target_properties (robotNavigationDriver PROPERTIES OUTPUT_NAME robotNavigationDriver PREFIX "")
target_link_libraries (robotNavigationDriver ${catkin_LIBRARIES})
target_include_directories(robotNavigationDriver PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
add_dependencies(robotNavigationDriver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})