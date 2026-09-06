cmake_minimum_required(VERSION 3.8)
project(car_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成自定义消息与服务
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/AckermannCommand.msg"
  "msg/MotorFeedback.msg"
  "msg/UavStatus.msg"
  "msg/UavCommand.msg"
  "msg/LeadscrewCommand.msg"
  "msg/LeadscrewStatus.msg"
  "srv/SetGoal.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
