cmake_minimum_required(VERSION 3.8)
project(mgs1600gy_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# mgs1600gy_node ====================================================
ament_auto_add_library(${PROJECT_NAME} SHARED src/mgs1600gy_node.cpp)
rclcpp_components_register_node(
  ${PROJECT_NAME} PLUGIN "mgs1600gy_node::Mgs1600gyNode" EXECUTABLE
  ${PROJECT_NAME}_exec
)
# End mgs1600gy_node ================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
