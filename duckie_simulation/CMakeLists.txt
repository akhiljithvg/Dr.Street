cmake_minimum_required(VERSION 3.8)
project(duckie_simulation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package (for imports)
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts as executables
install(PROGRAMS
  duckie_simulation/sim_motor_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sim_motor_bridge
)

install(PROGRAMS
  duckie_simulation/sim_perception_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sim_perception_node
)

# Install data directories
install(DIRECTORY description/ DESTINATION share/${PROJECT_NAME}/description)
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
