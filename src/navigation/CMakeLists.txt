cmake_minimum_required(VERSION 3.8)
project(navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install launch files and config
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install the gps_goal_sender as a runnable Python node
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  navigation/gps_goal_sender.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gps_goal_sender
)

install(PROGRAMS
  navigation/dock_goal_sender.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dock_goal_sender
)

install(PROGRAMS
  scripts/compute_dock_pose.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME compute_dock_pose
)

ament_package()
