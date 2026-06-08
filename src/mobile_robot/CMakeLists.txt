cmake_minimum_required(VERSION 3.8)
project(mobile_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch model config maps docs scripts rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  FILES CURRICULUM_MAPS.md ROBOT_MODEL_YAHBOOM_RPLIDAR_C1.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS
  mobile_robot/rl/deploy_sac.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  mobile_robot/sac_navigation/center_route_requester_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME center_route_requester_node
)

install(PROGRAMS
  mobile_robot/sac_navigation/local_target_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME local_target_node
)

install(PROGRAMS
  mobile_robot/sac_navigation/sac_motion_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sac_motion_node
)

install(PROGRAMS
  mobile_robot/sac_navigation/obstacle_memory_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME obstacle_memory_node
)

install(PROGRAMS
  mobile_robot/sac_navigation/sac_safety_filter_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sac_safety_filter_node
)

install(PROGRAMS
  mobile_robot/sac_navigation/sac_validation_logger.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sac_validation_logger
)

install(PROGRAMS
  mobile_robot/localization/return_to_start_goal_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME return_to_start_goal_node
)

install(PROGRAMS
  mobile_robot/localization/arena_center_estimator_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME arena_center_estimator_node
)

install(PROGRAMS
  mobile_robot/localization/global_amcl_autorotate_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME global_amcl_autorotate_node
)

install(PROGRAMS
  mobile_robot/localization/goal_relative_calculator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME goal_relative_calculator
)

install(PROGRAMS
  mobile_robot/localization/center_success_marker_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME center_success_marker_node
)

install(PROGRAMS
  mobile_robot/action_logger_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME action_logger_node
)

install(PROGRAMS
  mobile_robot/experiment_logger.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME experiment_logger
)

install(PROGRAMS
  scripts/analyze_experiment_log.py
  scripts/battery_endurance_monitor.py
  scripts/live_trial_monitor.py
  scripts/live_trial_plotter.py
  scripts/run_endurance_spin_test.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
