cmake_minimum_required(VERSION 3.14)
project(lane_width_calculator)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_library(lane_width_calculator_node SHARED
  src/node.cpp
)

rclcpp_components_register_node(lane_width_calculator_node
  PLUGIN "lane_width_calculator::CalculatorNode"
  EXECUTABLE lane_width_calculator
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lane_width_calculator_node
    test/test_lane_width_calculator_node.cpp
  )
  target_link_libraries(test_lane_width_calculator_node
    lane_width_calculator_node
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
