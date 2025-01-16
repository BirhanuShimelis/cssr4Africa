
# # Add the executables
# add_executable        (exampleComponentTest src/exampleComponentTestApplication.cpp src/exampleComponentTestImplementation.cpp)
# set_target_properties (exampleComponentTest PROPERTIES OUTPUT_NAME exampleComponentTest PREFIX "")
# target_link_libraries (exampleComponentTest ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
# target_include_directories(exampleComponentTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
# add_dependencies(exampleComponentTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # Add the executables for the stub and driver
# add_executable        (exampleComponentStub src/exampleComponentStub.cpp)
# set_target_properties (exampleComponentStub PROPERTIES OUTPUT_NAME exampleComponentStub PREFIX "")
# target_link_libraries (exampleComponentStub ${catkin_LIBRARIES})
# target_include_directories(exampleComponentStub PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
# add_dependencies(exampleComponentStub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # Add the executables for the stub and driver
# add_executable        (exampleComponentDriver src/exampleComponentDriver.cpp)
# set_target_properties (exampleComponentDriver PROPERTIES OUTPUT_NAME exampleComponentDriver PREFIX "")
# target_link_libraries (exampleComponentDriver ${catkin_LIBRARIES})
# target_include_directories(exampleComponentDriver PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
# add_dependencies(exampleComponentDriver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})