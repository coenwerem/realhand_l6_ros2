cmake_minimum_required(VERSION 3.16)
project(realhand_contact_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_DEPENDS
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  std_msgs
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(dep IN LISTS THIS_PACKAGE_DEPENDS)
  find_package(${dep} REQUIRED)
endforeach()

generate_parameter_library(contact_gated_controller_parameters
  src/contact_gated_controller_parameters.yaml
)

add_library(${PROJECT_NAME} SHARED
  src/contact_gated_controller.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC contact_gated_controller_parameters)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_DEPENDS})

pluginlib_export_plugin_description_file(controller_interface realhand_contact_controller.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} contact_gated_controller_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_contact_gated_controller test/test_contact_gated_controller.cpp)
  target_link_libraries(test_contact_gated_controller ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPENDS})
ament_package()
