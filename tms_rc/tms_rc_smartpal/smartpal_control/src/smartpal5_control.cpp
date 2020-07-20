///-----------------------------------------------------------------------------
/// @FileName smartpal5_control.cpp /// @Date 2014.06.18 / 2013.06.02
/// @author Yoonseok Pyo (passionvirus@gmail.com)
///-----------------------------------------------------------------------------
//------------------------------------------------------------------------------
#include <chrono>
#include <memory>
#include <stdio.h>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"


#include "std_msgs/msg/string.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>

#include "sensor_msgs/msg/joint_state.hpp"
#include <tms_msg_db/msg/tmsdb_stamped.hpp>
#include <tms_msg_db/srv/tmsdb_get_data.hpp>
#include <tms_msg_rc/srv/turtlebot_control.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>


#include <geometry_msgs/msg/pose_stamped.hpp>


#include <vector>
#include "sp5_client.h"

// using namespace std::chrono_literals;

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


tf2_ros::TransformListener *listener;

Client *smartpal = new Client;


rclcpp::Publisher<tms_msg_db::msg::TmsdbStamped>::SharedPtr pose_publisher;
rclcpp::Client<tms_msg_db::srv::TmsdbGetData>::SharedPtr get_data_client;

std::shared_ptr<rclcpp::Node> node;

int getRobotCurrentPos(double *x, double *y, double *th, double *waistL, double *waistH, double *jointR,
                       double *gripperR, double *jointL, double *gripperL)
{
  int8_t ret1, ret2, ret3, ret4, ret5, ret6;
  ret1 = smartpal->vehicleGetPos(x, y, th);
  ret2 = smartpal->armGetPos(ArmR, 0, jointR);
  ret3 = smartpal->armGetPos(ArmL, 0, jointL);
  ret4 = smartpal->gripperGetPos(ArmR, gripperR);
  ret5 = smartpal->gripperGetPos(ArmL, gripperL);
  ret6 = smartpal->lumbaGetPos(waistL, waistH);

  if ((ret1 + ret2 + ret3 + ret4 + ret5 + ret6) <= 0)
  {
    printf("Failed to get robot position.\n");
    return 0;
  }
  else
    return 1;
}
bool is_grasp = false;
int grasping_object_id = 0;
//------------------------------------------------------------------------------

class armInfo
{
  public:
  int move;
  double j_L[7];         // 0
  double gripper_left;   // 1
  double j_R[7];         // 2
  double gripper_right;  // 3
};
std::vector<armInfo> trajectory;

const int sid_ = 100000;

int g_oid;
double g_ox;
double g_oy;
double g_oz;
double g_orr;
double g_orp;
double g_ory;

double x;
double y;
double th;
double gripperR;
double gripperL;
double waistL;
double waistH;
double jointR[7];
double jointL[7];

int32_t id_robot = 2003;                // SmartPal5_2 ID
int32_t id_odometry_and_joints = 3003;  // Sensor ID
int32_t id_place = 5002;                // Place ID


//------------------------------------------------------------------------------
using getRobotData = tms_msg_db::srv::TmsdbGetData;
using getServer = tms_msg_rc::srv::TurtlebotControl;


void armcallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  cout  << "armcallback" << endl;
    armInfo *ai;
  if (msg->name[0] == "l_gripper_thumb_joint")
  {
    ai->gripper_left = msg->position[0];
    ai->move = 1;
  }
  else if (msg->name[0] == "l_arm_j1_joint")
  {
    for (int i = 0; i < 7; i++)
    {
      ai->j_L[i] = msg->position[i];
    }
    ai->move = 0;
  }
  trajectory.push_back(*ai);
}
void ObjectDataUpdate(const moveit_msgs::msg::PlanningScene::SharedPtr msg)
{
  cout  << "ObjectDataUpdate" << endl;
  if (msg->robot_state.attached_collision_objects.size() != 0)
  {  // grasped object
    int object_id = atoi(msg->robot_state.attached_collision_objects[0].object.id.c_str());

    g_oid = object_id;

    geometry_msgs::msg::PoseStamped pose, pose2;
    pose.header.frame_id = "/l_end_effector_link";
    pose.pose.position.x = msg->robot_state.attached_collision_objects[0].object.primitive_poses[0].position.x;
    pose.pose.position.y = msg->robot_state.attached_collision_objects[0].object.primitive_poses[0].position.y;
    pose.pose.position.z = msg->robot_state.attached_collision_objects[0].object.primitive_poses[0].position.z;

    pose.pose.orientation.x = msg->robot_state.attached_collision_objects[0].object.primitive_poses[0].orientation.x;
    pose.pose.orientation.y = msg->robot_state.attached_collision_objects[0].object.primitive_poses[0].orientation.y;
    pose.pose.orientation.z = msg->robot_state.attached_collision_objects[0].object.primitive_poses[0].orientation.z;
    pose.pose.orientation.w = msg->robot_state.attached_collision_objects[0].object.primitive_poses[0].orientation.w;


    cout  << "client 1" << endl;
    
    g_ox = pose2.pose.position.x;
    g_oy = pose2.pose.position.y;
    g_oz = pose2.pose.position.z;
    tf2::Quaternion q2(pose2.pose.orientation.x, pose2.pose.orientation.y, pose2.pose.orientation.z,
                      pose2.pose.orientation.w);
    tf2::Matrix3x3 m2(q2);
    m2.getRPY(g_orr, g_orp, g_ory);
    while (!get_data_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
        return;
      }
      RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
    }
    auto srv = std::make_shared<tms_msg_db::srv::TmsdbGetData::Request>();
    tms_msg_db::srv::TmsdbGetData::Response res;

    srv->tmsdb.id = object_id + sid_;
    auto result_future = get_data_client->async_send_request(srv);
    auto status = result_future.wait_for(std::chrono::seconds(3));
    if(status == std::future_status::ready)
    {
      // g_oz -= srv.response.tmsdb[0].offset_z;
      auto res = result_future.get();
      rclcpp::Time now = rclcpp::Clock().now();  // GMT +9

      tms_msg_db::msg::TmsdbStamped db_msg;
      tms_msg_db::msg::Tmsdb current_pos_data;

      db_msg.header.frame_id = "/world";
      db_msg.header.stamp = now;

      // current_pos_data.time = boost::posix_time::to_iso_extended_string(now.toBoost());
      current_pos_data.id = g_oid;
      current_pos_data.name = res->tmsdb[0].name;
      current_pos_data.type = res->tmsdb[0].type;
      current_pos_data.x = g_ox;
      current_pos_data.y = g_oy;
      current_pos_data.z = g_oz;
      current_pos_data.rr = g_orr;
      current_pos_data.rp = g_orp;
      current_pos_data.ry = g_ory;
      current_pos_data.offset_x = res->tmsdb[0].offset_x;
      current_pos_data.offset_y = res->tmsdb[0].offset_y;
      current_pos_data.offset_z = res->tmsdb[0].offset_z;
      current_pos_data.place = 2003;
      current_pos_data.sensor = 3003;
      current_pos_data.probability = 1.0;
      current_pos_data.state = 2;

      db_msg.tmsdb.push_back(current_pos_data);
      pose_publisher->publish(db_msg);

      is_grasp = true;
      grasping_object_id = g_oid;
    }
    else
    {
      printf("failed to get data");
    }

  }
  if (is_grasp == true && msg->world.collision_objects.size() != 0 &&
      msg->world.collision_objects[0].primitive_poses.size() != 0)
  {  // released object
    is_grasp = false;
    grasping_object_id = 0;

    int object_id = atoi(msg->world.collision_objects[0].id.c_str());

    g_oid = object_id;
    g_ox = msg->world.collision_objects[0].primitive_poses[0].position.x;
    g_oy = msg->world.collision_objects[0].primitive_poses[0].position.y;
    g_oz = msg->world.collision_objects[0].primitive_poses[0].position.z;
    cout  << "client 2" << endl;
    
    while (!get_data_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
        return;
      }
      RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
    }
    auto srv = std::make_shared<tms_msg_db::srv::TmsdbGetData::Request>();
    tms_msg_db::srv::TmsdbGetData::Response res;
    // srv.tmsdb.id = object_id + sid_;
    auto result_future = get_data_client->async_send_request(srv);
    auto status = result_future.wait_for(std::chrono::seconds(3));
    if(status == std::future_status::ready)
    {
      // g_oz -= srv.response.tmsdb[0].offset_z;
      auto res = result_future.get();
      rclcpp::Time now = rclcpp::Clock().now();  // GMT +9
      // boost::posix_time::ptime current = boost::posix_time::microsec_clock::local_time();
      tms_msg_db::msg::TmsdbStamped db_msg;
      tms_msg_db::msg::Tmsdb current_pos_data;

      db_msg.header.frame_id = "/world";
      db_msg.header.stamp = now;

      // current_pos_data.time = now->current_pos_data.time;
      current_pos_data.id = g_oid;
      current_pos_data.name = res->tmsdb[0].name;
      current_pos_data.type = res->tmsdb[0].type;
      current_pos_data.x = g_ox;
      current_pos_data.y = g_oy;
      current_pos_data.z = g_oz;
      current_pos_data.rr = g_orr;
      current_pos_data.rp = g_orp;
      current_pos_data.ry = g_ory;
      current_pos_data.offset_x = res->tmsdb[0].offset_x;
      current_pos_data.offset_y = res->tmsdb[0].offset_y;
      current_pos_data.offset_z = res->tmsdb[0].offset_z;
      current_pos_data.place = 5002;
      current_pos_data.sensor = 3003;
      current_pos_data.probability = 1.0;
      current_pos_data.state = 1;

      db_msg.tmsdb.push_back(current_pos_data);
      pose_publisher->publish(db_msg);
    }
    else
    {
      printf("failed to get data");
    }
  }
}
void clin_()
{
    cout  << "client 0" << endl;
    while (!get_data_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
        return;
      }
      RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
    }
    auto dbreq = std::make_shared<getRobotData::Request>();
    dbreq->tmsdb.id = 2003;      // SmartPal5_2 ID
    dbreq->tmsdb.sensor = 3001;  // Vicon ID
    auto result_future = get_data_client->async_send_request(dbreq);
    auto status = result_future.wait_for(std::chrono::seconds(3));
    if(status == std::future_status::ready)
    // if (get_data_client.call(getRobotData))
    {
      printf("Get info of object ID: %d\n", dbreq->tmsdb.id);
    }
    else if (status == std::future_status::timeout) 
    {
      printf("Failed to call service getRobotData ID: %d\n", dbreq->tmsdb.id);

    }
    auto result = result_future.get();

}

void robotControl(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<tms_msg_rc::srv::TurtlebotControl::Request> req, const std::shared_ptr<tms_msg_rc::srv::TurtlebotControl::Response> res)
{
  (void)request_header;
  // tms_msg_db::srv::TmsdbGetData getRobotData;
  

  cout  << "robotcontrol" << endl;
  // auto dbreq = std::make_shared<tms_msg_db::srv::TmsdbGetData::Request>();
  // RCLCPP_INFO(g_node->get_logger(),"request: %" PRId64 " + %" PRId64 " + %" PRId64, req->unit, req->cmd, req->arg[0]);
  
  switch (req->unit)
  {
    //--------------------------------------------------------------------------
    case 0:  // all
      switch (req->cmd)
      {
        case 0:  // clearAlarm
          smartpal->vehicleClearAlarm();
          smartpal->armClearAlarm(ArmR);
          smartpal->armClearAlarm(ArmL);
          smartpal->gripperClearAlarm(GripperR);
          smartpal->gripperClearAlarm(GripperL);
          smartpal->lumbaClearAlarm();
          res->result = SUCCESS;  // temp
          break;
        case 1:  // setPower
          smartpal->vehicleSetPower(req->arg[0]);
          smartpal->armSetPower(ArmR, req->arg[0]);
          smartpal->armSetPower(ArmL, req->arg[0]);
          smartpal->gripperSetPower(GripperR, req->arg[0]);
          smartpal->gripperSetPower(GripperL, req->arg[0]);
          smartpal->lumbaSetPower(req->arg[0]);
          res->result = SUCCESS;  // temp
          break;
        case 2:  // setServo
          smartpal->vehicleSetServo(req->arg[0]);
          smartpal->armSetServo(ArmR, req->arg[0]);
          smartpal->armSetServo(ArmL, req->arg[0]);
          smartpal->gripperSetServo(GripperR, req->arg[0]);
          smartpal->gripperSetServo(GripperL, req->arg[0]);
          smartpal->lumbaSetServo(req->arg[0]);
          res->result = SUCCESS;  // temp
          break;
        case 3:  // pause
          smartpal->vehiclePause();
          smartpal->armPause(ArmR);
          smartpal->armPause(ArmL);
          smartpal->gripperPause(GripperR);
          smartpal->gripperPause(GripperL);
          smartpal->lumbaPause();
          res->result = SUCCESS;  // temp
          break;
        case 4:  // resume
          smartpal->vehicleResume();
          smartpal->armResume(ArmR);
          smartpal->armResume(ArmL);
          smartpal->gripperResume(GripperR);
          smartpal->gripperResume(GripperL);
          smartpal->lumbaResume();
          res->result = SUCCESS;  // temp
          break;
        case 5:  // abort
          smartpal->armAbort(ArmR);
          smartpal->armAbort(ArmL);
          smartpal->gripperAbort(GripperR);
          smartpal->gripperAbort(GripperL);
          smartpal->lumbaAbort();
          res->result = SUCCESS;  // temp
          break;
        case 6:  // stop
          smartpal->vehicleStop();
          smartpal->armStop(ArmR);
          smartpal->armStop(ArmL);
          smartpal->gripperStop(GripperR);
          smartpal->gripperStop(GripperL);
          smartpal->lumbaStop();
          res->result = SUCCESS;  // temp
          break;
        case 7:                                      // getViconData
          {
            clin_();
            tms_msg_db::srv::TmsdbGetData::Response dbres;
            if (!dbres.tmsdb.empty())
            {
              if (dbres.tmsdb[0].x != 0 && dbres.tmsdb[0].y != 0)
              {
                res->result = smartpal->vehicleSetPos(dbres.tmsdb[0].x, dbres.tmsdb[0].y,
                                                    dbres.tmsdb[0].ry);
              }
            }
            break;
          }
        case 8:  // move trajectory
        {
          res->result = SUCCESS;
          if (!trajectory.empty())
          {
            for (int t=0; t<trajectory.size(); t++)
            {
              if (trajectory.at(t).move == 0)
              {
                printf("trajectory[%d]:armL [%f,%f,%f,%f,%f,%f,%f]", t, trajectory.at(t).j_L[0],
                        trajectory.at(t).j_L[1], trajectory.at(t).j_L[2], trajectory.at(t).j_L[3],
                        trajectory.at(t).j_L[4], trajectory.at(t).j_L[5], trajectory.at(t).j_L[6]);
              }
              else if (trajectory.at(t).move == 1)
              {
                printf("trajectory[%d]:gripperL %f", t, trajectory.at(t).gripper_left);
              }
            }
            for (int t=0; t<trajectory.size(); t++)
            {
              printf("trajectory[%d]", t);
              if (trajectory.at(t).move == 0)
              {  // armL
                double arg[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.175};
                for (int i = 0; i < 7; i++)
                {
                  arg[i] = trajectory.at(t).j_L[i];
                }
                res->result = smartpal->armMoveJointAbs(ArmL, &arg[0], arg[7]);
                // returnのタイミングをロボットの状態がReadyになるまで待機
                rclcpp::Time begin = rclcpp::Clock().now();
                while (1)
                {
                  int state = smartpal->armGetState(ArmL);
                  if (state == Busy)
                  {
                    printf("armL state : Busy\n");
                    rclcpp::Rate(1.0).sleep();
                  }
                  else if (state == Ready)
                  {
                    printf("Succeed to manipulation action(armL).state : Ready\n");
                    res->result = SUCCESS;
                    break;
                  }
                  else
                  {
                    printf("Failed to get collect armL status:%d\n", state);
                    rclcpp::Rate(1.0).sleep();
                    smartpal->armClearAlarm(ArmL);
                  }
                  rclcpp::Time now = rclcpp::Clock().now();
                  // Time out
                  if ((now - begin).nanoseconds() >= 60.0)
                  {
                    printf("TIMEOUT!(armL)\n");
                    res->result = FAILURE;
                    break;
                  }
                }
              }
              else if (trajectory.at(t).move == 1)
              {  // gripperL
                double arg[3] = {0.0, 0.175, 0.175};
                arg[0] = trajectory.at(t).gripper_left;
                res->result = smartpal->gripperMoveAbs(ArmL, arg[0], arg[1], arg[2]);
                if (res->result != SUCCESS)
                  printf("Failed gripperMove(armL)\n");
                while (1)
                {
                  if (smartpal->gripperGetState(ArmL))
                  {
                    printf("gripperL state : moving");
                    rclcpp::Rate(1.0).sleep();
                  }
                  else
                  {
                    printf("gripperL state : ready");
                    break;
                  }
                }
              }

              if (res->result != SUCCESS)
                break;
              else
                printf("succeed");
            }
            trajectory.clear();
          }
          break;
        }
        break;
        default:
          {
            res->result = SRV_CMD_ERR;
            break;
          }
      }
      break;
      
    //--------------------------------------------------------------------------
    case 1:  // vehicle
      switch (req->cmd)
      {
        case 0:
          res->result = smartpal->vehicleClearAlarm();
          break;
        case 1:
          res->result = smartpal->vehicleSetPower(req->arg[0]);
          break;
        case 2:
          res->result = smartpal->vehicleSetServo(req->arg[0]);
          break;
        case 3:
          res->result = smartpal->vehiclePause();
          break;
        case 4:
          res->result = smartpal->vehicleResume();
          break;
        case 6:
          res->result = smartpal->vehicleStop();
          break;
        case 7:
          res->result = smartpal->vehicleGetState();
          break;
        case 8:
          res->val.resize(3);
          res->result = smartpal->vehicleGetPos(&res->val[0], &res->val[1], &res->val[2]);
          break;
        case 9:
          res->result = smartpal->vehicleSetPos(req->arg[0], req->arg[1], req->arg[2]);
          break;
        case 10:
          res->result = smartpal->vehicleSetVel(req->arg[0], req->arg[1]);
          break;
        case 11:
          res->result = smartpal->vehicleSetAcc(req->arg[0], req->arg[1]);
          break;
        case 15:
        {
          res->result = smartpal->vehicleMoveLinearAbs(req->arg[0], req->arg[1], req->arg[2]);
          // returnのタイミングをロボットの状態がReadyになるまで待機
          rclcpp::Time begin = rclcpp::Clock().now();
          while (1)
          {
            int state = smartpal->vehicleGetState();
            if (state == VehicleBusy)
            {
              printf("vehicle state : Busy\n");
              rclcpp::Rate(1.0).sleep();
            }
            else if (state == VehicleReady)
            {
              printf("Succeed to move vehicle.state : Ready\n");
              res->result = SUCCESS;
              break;
            }
            else
            {
              printf("Failed to get collect vehicle status:%d\n", state);
            }

            rclcpp::Time now = rclcpp::Clock().now();
            // Time out
            if ((now - begin).nanoseconds() >= 60.0)
            {
              printf("TIMEOUT!(vehicle)\n");
              res->result = FAILURE;
              break;
            }
          }
          break;
        }
        case 16:
        {
          res->result = smartpal->vehicleMoveLinearRel(req->arg[0], req->arg[1], req->arg[2]);
          // returnのタイミングをロボットの状態がReadyになるまで待機
          rclcpp::Time begin = rclcpp::Clock().now();
          while (1)
          {
            int state = smartpal->vehicleGetState();
            if (state == VehicleBusy)
            {
              printf("vehicle state : Busy\n");
              rclcpp::Rate(1.0).sleep();
            }
            else if (state == VehicleReady)
            {
              printf("Succeed to move vehicle.state : Ready\n");
              res->result = SUCCESS;
              break;
            }
            else
            {
              printf("Failed to get collect vehicle status:%d\n", state);
            }

            rclcpp::Time now = rclcpp::Clock().now();
            // Time out
            if ((now - begin).nanoseconds() >= 60.0)
            {
              printf("TIMEOUT!(vehicle)\n");
              res->result = FAILURE;
              break;
            }
          }
          break;
        }
        case 17:
          res->result = smartpal->vehicleMoveCruiseAbs(req->arg[0], req->arg[1]);
          break;
        case 18:
          res->result = smartpal->vehicleMoveCruiseRel(req->arg[0], req->arg[1]);
          break;
        case 19:
          res->result = smartpal->vehicleMoveContinuousRel(req->arg[0], req->arg[1], req->arg[2]);
          break;
        case 20:
          res->result = smartpal->vehicleMoveCircularRel(req->arg[0], req->arg[1], req->arg[2]);
          break;
        case 25:
          res->result = smartpal->vehicleSetJogTimeout(req->arg[0]);
          break;
        case 26:
          res->result = smartpal->vehicleMoveJog(req->arg[0], req->arg[1], req->arg[2]);
          break;
        default:
          res->result = SRV_CMD_ERR;
          break;
      }
      break;
    //--------------------------------------------------------------------------
    case 2:  // ArmR
      switch (req->cmd)
      {
        case 0:
          res->result = smartpal->armClearAlarm(ArmR);
          break;
        case 1:
          res->result = smartpal->armSetPower(ArmR, req->arg[0]);
          break;
        case 2:
          res->result = smartpal->armSetServo(ArmR, req->arg[0]);
          break;
        case 3:
          res->result = smartpal->armPause(ArmR);
          break;
        case 4:
          res->result = smartpal->armResume(ArmR);
          break;
        case 5:
          res->result = smartpal->armAbort(ArmR);
          break;
        case 6:
          res->result = smartpal->armStop(ArmR);
          break;
        case 7:
          res->result = smartpal->armGetState(ArmR);
          break;
        case 8:
          res->val.resize(7);
          res->result = smartpal->armGetPos(ArmR, req->arg[0], &res->val[0]);
          break;
        case 9:
          res->result = smartpal->armGetActiveAlarm(ArmR, req->arg[0], &res->val[0]);
          break;
        case 10:
          res->result = smartpal->armSetJointAcc(ArmR, req->arg[0]);
          break;
        case 11:
          res->result = smartpal->armSetLinearAcc(ArmR, req->arg[0], req->arg[1]);
          break;
        case 12:
          res->result = smartpal->armIsPowerOn(ArmR);
          break;
        case 13:
          res->result = smartpal->armIsServoOn(ArmR);
          break;
        case 15:
        {
          res->result = smartpal->armMoveJointAbs(ArmR, &req->arg[0], req->arg[7]);
          // returnのタイミングをロボットの状態がReadyになるまで待機
          rclcpp::Time begin = rclcpp::Clock().now();
          while (1)
          {
            int state = smartpal->armGetState(ArmR);
            if (state == Busy)
            {
              printf("armR state : Busy\n");
              rclcpp::Rate(1.0).sleep();
            }
            else if (state == Ready)
            {
              printf("Succeed to manipulation action(armR).state : Ready\n");
              res->result = SUCCESS;
              break;
            }
            else
            {
              printf("Failed to get collect armR status:%d\n", state);
            }
            rclcpp::Time now = rclcpp::Clock().now();
            // Time out
            if ((now - begin).nanoseconds() >= 60.0)
            {
              printf("TIMEOUT!(armR)\n");
              res->result = FAILURE;
              break;
            }
          }
          break;
        }
        case 16:
          res->result = smartpal->armMoveJointRel(ArmR, &req->arg[0], req->arg[7]);
          break;
        case 17:
          res->result = smartpal->armMoveLinearAbs(ArmR, req->arg[0], &req->arg[1], req->arg[7], req->arg[8], req->arg[9]);
          break;
        case 18:
          res->result = smartpal->armMoveLinearRel(ArmR, req->arg[0], &req->arg[1], req->arg[7], req->arg[8], req->arg[9]);
          break;
        default:
          res->result = SRV_CMD_ERR;
          break;
      }
      break;
    //--------------------------------------------------------------------------
    case 3:  // ArmL
      switch (req->cmd)
      {
        case 0:
          res->result = smartpal->armClearAlarm(ArmL);
          break;
        case 1:
          res->result = smartpal->armSetPower(ArmL, req->arg[0]);
          break;
        case 2:
          res->result = smartpal->armSetServo(ArmL, req->arg[0]);
          break;
        case 3:
          res->result = smartpal->armPause(ArmL);
          break;
        case 4:
          res->result = smartpal->armResume(ArmL);
          break;
        case 5:
          res->result = smartpal->armAbort(ArmL);
          break;
        case 6:
          res->result = smartpal->armStop(ArmL);
          break;
        case 7:
          res->result = smartpal->armGetState(ArmL);
          break;
        case 8:
          res->val.resize(7);
          res->result = smartpal->armGetPos(ArmL, req->arg[0], &res->val[0]);
          break;
        case 9:
          res->result = smartpal->armGetActiveAlarm(ArmL, req->arg[0], &res->val[0]);
          break;
        case 10:
          res->result = smartpal->armSetJointAcc(ArmL, req->arg[0]);
          break;
        case 11:
          res->result = smartpal->armSetLinearAcc(ArmL, req->arg[0], req->arg[1]);
          break;
        case 12:
          res->result = smartpal->armIsPowerOn(ArmL);
          break;
        case 13:
          res->result = smartpal->armIsServoOn(ArmL);
          break;
        case 15:
        {
          res->result = smartpal->armMoveJointAbs(ArmL, &req->arg[0], req->arg[7]);
          // returnのタイミングをロボットの状態がReadyになるまで待機
          rclcpp::Time begin = rclcpp::Clock().now();
          while (1)
          {
            int state = smartpal->armGetState(ArmL);
            if (state == Busy)
            {
              printf("armL state : Busy\n");
              rclcpp::Rate(3.0).sleep();
            }
            else if (state == Ready)
            {
              printf("Succeed to manipulation action(armL).state : Ready\n");
              res->result = SUCCESS;
              break;
            }
            else
            {
              printf("Failed to get collect armL status:%d\n", state);
            }
            rclcpp::Time now = rclcpp::Clock().now();
            // Time out
            if ((now - begin).nanoseconds() >= 60.0)
            {
              printf("TIMEOUT!(armL)\n");
              res->result = FAILURE;
              break;
            }
          }
          break;
        }
        case 16:
          res->result = smartpal->armMoveJointRel(ArmL, &req->arg[0], req->arg[7]);
          break;
        case 17:
          res->result = smartpal->armMoveLinearAbs(ArmL, req->arg[0], &req->arg[1], req->arg[7], req->arg[8], req->arg[9]);
          break;
        case 18:
          res->result = smartpal->armMoveLinearRel(ArmL, req->arg[0], &req->arg[1], req->arg[7], req->arg[8], req->arg[9]);
          break;
        default:
          res->result = SRV_CMD_ERR;
          break;
      }
      break;
    //--------------------------------------------------------------------------
    case 4:  // GripperR
      switch (req->cmd)
      {
        case 0:
          res->result = smartpal->gripperClearAlarm(ArmR);
          break;
        case 1:
          res->result = smartpal->gripperSetPower(ArmR, req->arg[0]);
          break;
        case 2:
          res->result = smartpal->gripperSetServo(ArmR, req->arg[0]);
          break;
        case 3:
          res->result = smartpal->gripperPause(ArmR);
          break;
        case 4:
          res->result = smartpal->gripperResume(ArmR);
          break;
        case 5:
          res->result = smartpal->gripperAbort(ArmR);
          break;
        case 6:
          res->result = smartpal->gripperStop(ArmR);
          break;
        case 7:
          res->result = smartpal->gripperGetState(ArmR);
          break;
        case 8:
          res->val.resize(1);
          res->result = smartpal->gripperGetPos(ArmR, &res->val[0]);
          break;
        case 15:
          res->result = smartpal->gripperMoveAbs(ArmR, req->arg[0], req->arg[1], req->arg[2]);
          break;
        default:
          res->result = SRV_CMD_ERR;
          break;
      }
      break;
    //--------------------------------------------------------------------------
    case 5:  // GripperL
      switch (req->cmd)
      {
        case 0:
          res->result = smartpal->gripperClearAlarm(ArmL);
          break;
        case 1:
          res->result = smartpal->gripperSetPower(ArmL, req->arg[0]);
          break;
        case 2:
          res->result = smartpal->gripperSetServo(ArmL, req->arg[0]);
          break;
        case 3:
          res->result = smartpal->gripperPause(ArmL);
          break;
        case 4:
          res->result = smartpal->gripperResume(ArmL);
          break;
        case 5:
          res->result = smartpal->gripperAbort(ArmL);
          break;
        case 6:
          res->result = smartpal->gripperStop(ArmL);
          break;
        case 7:
          res->result = smartpal->gripperGetState(ArmL);
          break;
        case 8:
          res->val.resize(1);
          res->result = smartpal->gripperGetPos(ArmL, &res->val[0]);
          break;
        case 15:
          res->result = smartpal->gripperMoveAbs(ArmL, req->arg[0], req->arg[1], req->arg[2]);
          break;
        default:
          res->result = SRV_CMD_ERR;
          break;
      }
      break;
    //--------------------------------------------------------------------------
    case 6:  // Waist
      switch (req->cmd)
      {
        case 0:
          res->result = smartpal->lumbaClearAlarm();
          break;
        case 1:
          res->result = smartpal->lumbaSetPower(req->arg[0]);
          break;
        case 2:
          res->result = smartpal->lumbaSetServo(req->arg[0]);
          break;
        case 3:
          res->result = smartpal->lumbaPause();
          break;
        case 4:
          res->result = smartpal->lumbaResume();
          break;
        case 5:
          res->result = smartpal->lumbaAbort();
          break;
        case 6:
          res->result = smartpal->lumbaStop();
          break;
        case 7:
          res->result = smartpal->lumbaGetState();
          break;
        case 8:
          res->val.resize(2);
          res->result = smartpal->lumbaGetPos(&res->val[0], &res->val[1]);
          break;
        case 15:
        {
          res->result = smartpal->lumbaMoveCooperative(req->arg[0], req->arg[1], req->arg[2]);
          // returnのタイミングをロボットの状態がReadyになるまで待機
          rclcpp::Time begin = rclcpp::Clock().now();
          while (1)
          {
            int state = smartpal->lumbaGetState();
            if (state == Busy)
            {
              printf("waist state : Busy\n");
              rclcpp::Rate(3.0).sleep();
            }
            else if (state == Ready)
            {
              printf("Succeed to move waist.state : Ready\n");
              res->result = SUCCESS;
              break;
            }
            else
            {
              printf("Failed to get collect waist status:%d\n", state);
            }
            rclcpp::Time now = rclcpp::Clock().now();
            // Time out
            if ((now - begin).nanoseconds() >= 60.0)
            {
              printf("TIMEOUT!(waist)\n");
              res->result = FAILURE;
              break;
            }
          }
          break;
        }
        case 16:
          res->result = smartpal->lumbaMove(req->arg[0], req->arg[1], req->arg[2], req->arg[3]);
          break;
        case 17:
          res->result = smartpal->lumbaMoveLowerAxis(req->arg[0], req->arg[1], req->arg[2]);
          break;
        case 18:
          res->result = smartpal->lumbaMoveUpperAxis(req->arg[0], req->arg[1], req->arg[2]);
          break;
        default:
          res->result = SRV_CMD_ERR;
          break;
      }
      break;
    //--------------------------------------------------------------------------
    case 7:  // CC
      res->result = SRV_UNIT_ERR;
      break;
      break;
    //--------------------------------------------------------------------------
    default:
      res->result = SRV_UNIT_ERR;
      break; 
  }
  return;
}

class SmartpalControl : public rclcpp::Node
{
public:
  SmartpalControl()
  : Node("smartpal_control_node")
  {
    initialize();
    // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    pose_publisher = this->create_publisher<tms_msg_db::msg::TmsdbStamped>("tms_db_data", 10);

    timer_ = this->create_wall_timer(
    500ms, std::bind(&SmartpalControl::timer_callback, this));
  
  }
  ~SmartpalControl()
  {    
    rclcpp::WallRate loop_rate(500ms);


  }
private:
  void initialize()
  {
    // test------------------------------------------------------------
    // auto message = std_msgs::msg::String();
    // message.data = "Hello, world! " + std::to_string(count_++);
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    // publisher_->publish(message);

    //--------------------------------------------------------------------------
    // smartpal initialize
    auto current_pos_data = tms_msg_db::msg::Tmsdb();
    uint8_t cnt = 0;
    cout  << "1" << endl;
    char ch;
    while (!smartpal->Initialize())
    {
      cnt++;

      printf("CORBA client object initialization has been failed.\n");
      printf("Preparing Retry ...\n");
      printf("Retry : anykey without q / Quit:q : ");
      scanf("%c", &ch);
      getchar();
      if (ch == 'q'){
        exit(1);
      }
      printf("Retry initializing...\n");

      
        // delay 3sec
      rclcpp::Rate(3.0).sleep();
      if (cnt == 5)
      {
        smartpal->Shutdown();
      }
      
    }
    printf("CORBA client object initialization has been completed.\n\n");

    //--------------------------------------------------------------------------
    // clear alarm

    smartpal->vehicleClearAlarm();
    smartpal->armClearAlarm(ArmR);
    smartpal->armClearAlarm(ArmL);
    smartpal->gripperClearAlarm(GripperR);
    smartpal->gripperClearAlarm(GripperL);
    smartpal->lumbaClearAlarm();

    rclcpp::Rate(2.0).sleep();

    // power on
    smartpal->lumbaSetPower(ON);
    smartpal->armSetPower(ArmR, ON);
    smartpal->armSetPower(ArmL, ON);
    smartpal->vehicleSetPower(ON);

    rclcpp::Rate(2.0).sleep();

    // servo on
    smartpal->vehicleSetServo(ON);
    smartpal->gripperSetServo(GripperR, ON);
    smartpal->gripperSetServo(GripperL, ON);
    smartpal->lumbaSetServo(ON);

    smartpal->armSetServo(ArmR, ON);
    smartpal->armSetServo(ArmL, ON);

    smartpal->armGetSoftLimit(ArmL);

    rclcpp::Time tNow;
    rclcpp::Rate(1.0).sleep();  // 10Hz frequency (0.1 sec)

  }
  void timer_callback()
  {
    rclcpp::Time now = rclcpp::Clock().now();
    if (getRobotCurrentPos(&x, &y, &th, &waistL, &waistH, jointR, &gripperR, jointL, &gripperL) == 1)
    {
      tms_msg_db::msg::TmsdbStamped db_msg;
      tms_msg_db::msg::Tmsdb current_pos_data;
      rclcpp::WallRate loop_rate(500ms);
      db_msg.header.frame_id = "/world";
      db_msg.header.stamp = now;

      // current_pos_data.time = rclcpp::Clock().now();
      current_pos_data.id = id_robot;
      current_pos_data.x = x;
      current_pos_data.y = y;
      current_pos_data.z = 0.0;
      current_pos_data.rr = 0.0;
      current_pos_data.rp = 0.0;
      current_pos_data.ry = th;
      current_pos_data.place = id_place;
      current_pos_data.sensor = id_odometry_and_joints;
      current_pos_data.state = 1;
      std::stringstream ss;
      ss << waistH << ";" << waistL << ";";
      for (int i = 0; i < 7; i++)
      {
        ss << jointR[i] << ";";
      }
      ss << gripperR << ";";
      for (int i = 0; i < 7; i++)
      {
        ss << jointL[i] << ";";
      }
      ss << gripperL;

      // joint information of smartpal5
      // lumba_low, lumba_high, jR[0]...[6], gripper_right, jL[0]...[6], gripper_left
      current_pos_data.joint = ss.str();
      std::stringstream ss2;
      ss2 << "grasping=" << grasping_object_id;
      current_pos_data.note = ss2.str();
      db_msg.tmsdb.push_back(current_pos_data);
      pose_publisher->publish(db_msg);
      // loop_rate.sleep();
    }
    else
    {
      printf("Failed to getRobotCurrentPos\n");
    }  
  }
  //   }
  rclcpp::TimerBase::SharedPtr timer_;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::WallRate loop_rate(500ms);
  auto service_node = rclcpp::Node::make_shared("smartpal_server");
  auto publisher = rclcpp::Node::make_shared("pose_pub_node");
  auto node = rclcpp::Node::make_shared("smartpal_node");
  auto Subscription = rclcpp::Node::make_shared("Subscription");
  auto timer_node = std::make_shared<SmartpalControl>();


  auto pose_publisher = publisher->create_publisher<std_msgs::msg::String>("topic", 10);
  auto get_data_client = node->create_client<getRobotData>("smartpal_client");
  auto server = service_node->create_service<getServer>("smartpal_server1", robotControl);
  auto arm_data_sub = Subscription->create_subscription<sensor_msgs::msg::JointState>("/move_group/fake_controller_joint_states", 10, armcallback);
  auto object_data_sub = Subscription->create_subscription<moveit_msgs::msg::PlanningScene>("/move_group/monitored_planning_scene", 10, ObjectDataUpdate);

  
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  executor->add_node(service_node);
  executor->add_node(Subscription);

  executor->add_node(timer_node);

  executor->spin();

  rclcpp::shutdown();
  service_node = nullptr;
    //--------------------------------------------------------------------------
    // servo off
  smartpal->vehicleSetServo(OFF);
  smartpal->armSetServo(ArmR, OFF);
  smartpal->armSetServo(ArmL, OFF);
  smartpal->gripperSetServo(GripperR, OFF);
  smartpal->gripperSetServo(GripperL, OFF);
  smartpal->lumbaSetServo(OFF);
  loop_rate.sleep();

    // power off
  smartpal->vehicleSetPower(OFF);
  smartpal->armSetPower(ArmR, OFF);
  smartpal->armSetPower(ArmL, OFF);
  smartpal->lumbaSetPower(OFF);
  smartpal->Shutdown();
  delete listener;


  
  return 0;
}
