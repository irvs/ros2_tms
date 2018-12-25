cmake_minimum_required(VERSION 3.5)
project(tms_ss_vicon)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(tms_msg_ss REQUIRED) 
find_package(tms_msg_db REQUIRED)
find_package(visualization_msgs REQUIRED)


include_directories(
  include
)
#ament_export_include_directories(include)
ament_export_include_directories(${PROJECT_SOURCE_DIR}/viconSDK)

link_directories(${LINK_DIRECTORIES} ${PROJECT_SOURCE_DIR}/viconSDK)

ament_export_dependencies(ament_cmake)

ament_export_dependencies(vicon_stream)
ament_export_dependencies(tms_msg_ss_generate_messages_cpp)
ament_export_dependencies(tms_msg_db_generate_messages_cpp)

ament_export_include_directories(include)

#add_dependencies(vicon_stream tms_msg_ss_generate_messages_cpp tms_msg_db_generate_messages_cpp)

add_executable(vicon_stream src/vicon_streamtest.cpp)
#add_executable(vicon_stream src/Test.cpp)
ament_target_dependencies(vicon_stream
    rclcpp
    rcutils
    std_msgs
    visualization_msgs

    tms_msg_ss
    tms_msg_db
)

target_link_libraries(vicon_stream
  ViconDataStreamSDK_CPP
  ${rclcpp_LIBRARIES}
  ${rmw_implementation_LIBRARIES}
  ${std_msgs_LIBRARIES}
)
#target_link_libraries(test_executor ${PROJECT_NAME})

install(TARGETS vicon_stream
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
