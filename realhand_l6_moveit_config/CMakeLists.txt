cmake_minimum_required(VERSION 3.16)
project(realhand_l6_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz srdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_pytest_test(test_srdf test/test_srdf.py)
  add_launch_test(test/test_plan_to_pinch.py TIMEOUT 180)
endif()

ament_package()
