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
    ./src/commander/configuration_commander.cpp
    ./src/commander/maintenance_commander.cpp
    ./src/commander/realtime_commander.cpp
    ./src/mgs1600gy_interface.cpp
    ./src/packet_handler.cpp
    ./src/packet_pool.cpp
    ./src/port_handler.cpp
    ./src/utils.cpp
)
ament_target_dependencies(${PROJECT_NAME} OpenCV)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
ament_export_dependencies(OpenCV)
# End mgs1600gy interface ===========================================


# Examples ==========================================================
set(EXAMPLES
  show_data
  show_image
  show_imu)
foreach(TARGET ${EXAMPLES})
  ament_auto_add_executable(${TARGET} ./examples/${TARGET}.cpp)
  target_link_libraries(${TARGET} ${PROJECT_NAME})
endforeach()
# End examples ======================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gtest(
    test_packet_pool
    test/src/test_packet_pool.cpp)
  target_include_directories(test_packet_pool PRIVATE "include")
  target_link_libraries(test_packet_pool ${PROJECT_NAME})

  set(
    COMMANDER_TEST_CASES
      test_realtime_commander
      test_maintenance_commander)
  foreach(TARGET ${COMMANDER_TEST_CASES})
    ament_add_gmock(
      ${TARGET}
      test/src/commander/${TARGET}.cpp)
    target_link_libraries(${TARGET} ${PROJECT_NAME})
  endforeach()
endif()

ament_auto_package()
