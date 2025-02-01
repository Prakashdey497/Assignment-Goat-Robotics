cmake_minimum_required(VERSION 3.8)
project(amr_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(amr_custom_msg REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/problem_one.py
  ${PROJECT_NAME}/problem_two.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
