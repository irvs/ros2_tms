cmake_minimum_required(VERSION 3.5)
project(tms_msg_ts)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(tms_msg_ts
  "msg/Arg.msg"
  "srv/TmsSaFindObjects.srv"
  "srv/TsReq.srv"
  "srv/TsStateControl.srv"
  "srv/TsDoTask.srv"
  "srv/TaskTextRecognize.srv"
  "action/TsDoSubtask.action"
  "action/TsDoTask.action"
  "action/TsReq.action"
  DEPENDENCIES builtin_interfaces
)

ament_package()