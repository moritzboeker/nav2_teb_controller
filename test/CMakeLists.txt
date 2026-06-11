# ── test_edge_time_optimal ────────────────────────────────────────────────────
ament_add_gtest(test_edge_time_optimal
  unit/g2o_types/test_edge_time_optimal.cpp
)
target_include_directories(test_edge_time_optimal PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(test_edge_time_optimal rclcpp)
target_link_libraries(test_edge_time_optimal ${PROJECT_NAME})

# ── test_edge_path_smoothness ─────────────────────────────────────────────────
ament_add_gtest(test_edge_path_smoothness
  unit/g2o_types/test_edge_path_smoothness.cpp
)
target_include_directories(test_edge_path_smoothness PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(test_edge_path_smoothness rclcpp)
target_link_libraries(test_edge_path_smoothness ${PROJECT_NAME})

# ── test_edge_shortest_path ───────────────────────────────────────────────────
ament_add_gtest(test_edge_shortest_path
  unit/g2o_types/test_edge_shortest_path.cpp
)
target_include_directories(test_edge_shortest_path PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(test_edge_shortest_path rclcpp)
target_link_libraries(test_edge_shortest_path ${PROJECT_NAME})