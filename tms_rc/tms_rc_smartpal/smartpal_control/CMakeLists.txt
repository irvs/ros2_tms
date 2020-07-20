cmake_minimum_required(VERSION 3.5)
project(tms_rc_smartpal_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tms_msg_rc REQUIRED)
find_package(tms_msg_db REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


# set(smartpalIDLPath ~/catkin_ws/src/ros_tms/tms_rc/tms_rc_smartpal/smartpal_control/src/idl/sp5/)

# include_directories(
#   include
#   SYSTEM
#   ${smartpalIDLPath}
#   ${catkin_INCLUDE_DIRS}
# )
set(smartpalIDLPath ~/colcon_ws/src/ros_tms/tms_rc/tms_rc_smartpal/smartpal_control/src/idl/sp5/)

link_directories(${smartpalIDLPath})
# include_directories(include ${smartpalIDLPath})
# add_library(smartpal5_control SHARED
#   src/idl/sp5/Common.hh
# )

add_executable(smartpal5_control
   
    src/smartpal5_control.cpp 
   
    src/sp5_client.cpp    
    src/sp5_cmd_mobility.cpp
    src/sp5_cmd_arm.cpp
    src/sp5_cmd_gripper.cpp
    src/sp5_cmd_lumbar.cpp

    src/idl/sp5/CommonSK.cc

    src/idl/sp5/MobilitySK.cc

    src/idl/sp5/ArmUnitSK.cc
    
    src/idl/sp5/GripperUnitSK.cc
    src/idl/sp5/GripperExtAxesSK.cc

    src/idl/sp5/LumbarUnitSK.cc
    src/idl/sp5/LumbarExtAxesSK.cc
)
ament_target_dependencies(smartpal5_control rclcpp std_msgs sensor_msgs tms_msg_db tf2_ros tms_msg_rc moveit_msgs)
ament_export_libraries(smartpal5_control)

target_link_libraries(smartpal5_control omniORB4 omnithread)


install(TARGETS
  smartpal5_control
  DESTINATION lib/${PROJECT_NAME})


# install(TARGETS
#   smartpal5_control
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)




# add_dependencies(smartpal5_control tms_msg_rc_gencpp)

# target_link_libraries(smartpal5_control ${catkin_LIBRARIES} omniORB4 omnithread)

ament_package()
