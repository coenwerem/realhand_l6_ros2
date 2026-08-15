cmake_minimum_required(VERSION 3.16)
project(realhand_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

find_package(ament_cmake REQUIRED)
foreach(dep IN LISTS THIS_PACKAGE_DEPENDS)
  find_package(${dep} REQUIRED)
endforeach()

# Pure codec and model table, no ROS dependency, unit tested on its own.
add_library(realhand_protocol SHARED
  src/hand_models.cpp
  src/protocol.cpp
)
target_include_directories(realhand_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_library(${PROJECT_NAME} SHARED
  src/realhand_system.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC realhand_protocol)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_DEPENDS})

pluginlib_export_plugin_description_file(hardware_interface realhand_hardware.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS realhand_protocol ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol realhand_protocol)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPENDS})
ament_package()
