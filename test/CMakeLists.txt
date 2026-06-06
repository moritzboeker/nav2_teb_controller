ament_add_gtest(test_edge_time_optimal
  unit/g2o_types/test_edge_time_optimal.cpp
)

target_include_directories(test_edge_time_optimal PRIVATE
  ${CMAKE_SOURCE_DIR}/include
)

ament_target_dependencies(test_edge_time_optimal
  rclcpp
)

target_link_libraries(test_edge_time_optimal
  ${PROJECT_NAME}
)