cmake_minimum_required(VERSION 3.8)
project(we_r2_arm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Example.msg"
  "srv/Example.srv"
  "srv/EulerToQuaternion.srv"
  "srv/QuaternionToEuler.srv"
  "action/Example.action"
  "action/MoveJointGroup.action"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
