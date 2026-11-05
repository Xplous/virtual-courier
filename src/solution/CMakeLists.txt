cmake_minimum_required(VERSION 3.8)
project(solution)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/cell_explorer_node.py
  scripts/costmap_cloud_node.py
  scripts/courier_node.py
  scripts/station_detector.py
  scripts/docking_controller.py
  scripts/cargo_loader.py
  scripts/unload_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES scripts/maze_grid_node.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
