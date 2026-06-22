cmake_minimum_required(VERSION 3.8)
project(amr2ax_nav2)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch behavior_trees maps
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS 
    scripts/clicked_point_to_goal_update.py
    scripts/send_dynamic_follow_goal.py
    scripts/mission_executor.py
    scripts/mission_executor_v2.py
    scripts/dispatcher.py
    scripts/station.py
    scripts/station_manager.py
    scripts/inject.py
    scripts/dashboard.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
