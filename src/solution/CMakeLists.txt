cmake_minimum_required(VERSION 3.8)
project(solution)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ---- C++ узлы (пример; добавляйте свои по аналогии)
add_executable(example_cpp src/example_cpp.cpp)
ament_target_dependencies(example_cpp rclcpp sensor_msgs geometry_msgs)
install(TARGETS example_cpp DESTINATION lib/${PROJECT_NAME})

# ---- Python узлы (каждый новый скрипт добавьте в этот список и сделайте исполняемым)
install(PROGRAMS
  scripts/courier_node.py
  scripts/explorer_node.py
  scripts/station_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
