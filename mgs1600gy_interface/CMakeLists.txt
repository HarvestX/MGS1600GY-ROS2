cmake_minimum_required(VERSION 3.8)
project(mgs1600gy_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OpenCV REQUIRED)

# Mgs1600gy interface ===============================================
file(GLOB src_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.cpp)
file(GLOB commander_src_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/commander/*.cpp)
set(TARGET packet_handler)
set(MY_LIB_NAME ${PROJECT_NAME}_${TARGET})
ament_auto_add_library(${MY_LIB_NAME} SHARED ${src_files} ${commander_src_files})
ament_target_dependencies(${MY_LIB_NAME} OpenCV)

# Examples ==========================================================
set(EXAMPLES
  show_data
  show_image
  show_imu)

foreach(TARGET ${EXAMPLES})
  ament_auto_add_executable(${TARGET} ./examples/${TARGET}.cpp)
  target_link_libraries(${TARGET} ${MY_LIB_NAME})
endforeach()

# Testing ===========================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)

  set(TARGET test_packet_pool)
  ament_add_gtest(${TARGET} test/src/${TARGET}.cpp)
  target_include_directories(${TARGET} PRIVATE "include")
  target_link_libraries(${TARGET} ${MY_LIB_NAME})

  set(COMMANDER_TEST_CASES
    test_realtime_commander
    test_maintenance_commander)

  foreach(TARGET ${COMMANDER_TEST_CASES})
    ament_add_gmock(
      ${TARGET}
      test/src/commander/${TARGET}.cpp)
    target_link_libraries(${TARGET} ${MY_LIB_NAME})
  endforeach()
endif()

ament_export_dependencies(OpenCV)
ament_auto_package()
