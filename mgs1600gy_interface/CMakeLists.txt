cmake_minimum_required(VERSION 3.8)
project(mgs1600gy_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OpenCV REQUIRED)

# mgs1600gy interface ===============================================
ament_auto_add_library(
  ${PROJECT_NAME}
  STATIC
    ./src/port_handler.cpp
    ./src/packet_pool.cpp
    ./src/command_handler.cpp
    ./src/command_handler/prettier.cpp
    ./src/command_handler/rx_configuration.cpp
    ./src/command_handler/rx_realtime.cpp
    ./src/command_handler/tx_buffer.cpp
    ./src/command_handler/tx_configuration.cpp
    ./src/command_handler/tx_maintenance.cpp
    ./src/command_handler/tx_realtime.cpp
)
ament_target_dependencies(${PROJECT_NAME} OpenCV)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
# End mgs1600gy interface ===========================================

# show data executable ==============================================
ament_auto_add_executable(show_data ./examples/show_data.cpp)
target_link_libraries(show_data ${PROJECT_NAME})
# End show data executable ==========================================

# show image executabe ==============================================
ament_auto_add_executable(show_image ./examples/show_image.cpp)
target_link_libraries(show_image ${PROJECT_NAME})
# End show image executable =========================================


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_converter test/src/test_converter.cpp)
  target_link_libraries(test_converter ${PROJECT_NAME})

  ament_add_gtest(test_rx_realtime test/src/test_rx_realtime.cpp)
  target_link_libraries(test_rx_realtime ${PROJECT_NAME})

  ament_add_gtest(test_rx_configuration test/src/test_rx_configuration.cpp)
  target_link_libraries(test_rx_configuration ${PROJECT_NAME})

  ament_add_gtest(test_tx_configuration test/src/test_tx_configuration.cpp)
  target_link_libraries(test_tx_configuration ${PROJECT_NAME})

  ament_add_gtest(test_tx_maintenance test/src/test_tx_maintenance.cpp)
  target_link_libraries(test_tx_maintenance ${PROJECT_NAME})

  ament_add_gtest(test_tx_buffer test/src/test_tx_buffer.cpp)
  target_link_libraries(test_tx_buffer ${PROJECT_NAME})

  ament_add_gtest(test_tx_realtime test/src/test_tx_realtime.cpp)
  target_link_libraries(test_tx_realtime ${PROJECT_NAME})

  ament_add_gtest(test_parser test/src/test_parser.cpp)
  target_link_libraries(test_parser ${PROJECT_NAME})

  # ===================================================================
  ament_add_gtest(
    test_packet_pool
    test/src/test_packet_pool.cpp)
  target_include_directories(test_packet_pool PRIVATE "include")
  target_link_libraries(test_packet_pool ${PROJECT_NAME})
endif()

ament_auto_package()
