cmake_minimum_required(VERSION 3.8)
project(services_quiz_srv)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Debug line so we can see what CMake reads
message(STATUS "Using manifest: ${CMAKE_CURRENT_SOURCE_DIR}/package.xml")

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Turn.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
