#################################
add_executable(pcd_viz pcd_viz.cpp)
target_link_libraries(pcd_viz ${PCL_LIBRARIES} ${GFLAGS_LIBRARIES})
set_target_properties(pcd_viz PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")

