cmake_minimum_required(VERSION 3.8)
project(rover_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

install(DIRECTORY
  launch
  meshes
  urdf
  robots
  config
  DESTINATION share/${PROJECT_NAME}
)

# Si querés compilar un .xacro a .urdf en build y usarlo directamente:
# (opcional, útil para testing o generación previa)


ament_package()
