cmake_minimum_required(VERSION 3.8)
project(solution)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/cell_explorer_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES scripts/maze_grid_node.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
