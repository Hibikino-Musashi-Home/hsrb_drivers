cmake_minimum_required(VERSION 3.8)
project(hsrb_cgos_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/hsrb_cgos_hw.cpp
  src/${PROJECT_NAME}/hsrb_cgos_lib.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(hardware_interface hsrb_cgos_driver.xml)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gmock(test_hsrb_cgos_gpio test/hsrb_cgos_gpio_test.cpp)
  target_link_libraries(test_hsrb_cgos_gpio ${PROJECT_NAME})
  ament_target_dependencies(test_hsrb_cgos_gpio rclcpp)

  target_include_directories(test_hsrb_cgos_gpio PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_add_gmock(test_hsrb_cgos_lib test/hsrb_cgos_lib_test.cpp)
  target_link_libraries(test_hsrb_cgos_lib ${PROJECT_NAME})
  ament_target_dependencies(test_hsrb_cgos_lib rclcpp)

  target_include_directories(test_hsrb_cgos_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp hardware_interface pluginlib)

ament_package()