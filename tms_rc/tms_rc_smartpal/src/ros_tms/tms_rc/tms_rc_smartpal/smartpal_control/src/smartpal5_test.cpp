#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/Twist.h>
#include <vector>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <tms_msg_rc/msg/smartpal_control.h>
#include <tms_msg_db/msg/TmsdbStamped.h>
#include <tms_msg_db/msg/TmsdbGetData.h>
#include <sensor_msgs/msg/JointState.h>
#include <moveit_msgs/msg/PlanningScene.h>


#include <tf2_ros/transform_listener.h>
// #include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>

//------------------------------------------------------------------------------
#include "sp5_client.h"

#define rad2deg(x) ((x) * (180.0) / M_PI)
#define deg2rad(x) ((x)*M_PI / 180.0)

#define UNIT_ALL 0
#define UNIT_VEHICLE 1
#define UNIT_ARM_R 2
#define UNIT_ARM_L 3
#define UNIT_GRIPPER_R 4
#define UNIT_GRIPPER_L 5
#define UNIT_LUMBA 6
#define UNIT_CC 7

#define CMD_CLEARALARM 0
#define CMD_SETPOWER 1
#define CMD_SETSERVO 2
#define CMD_PAUSE 3
#define CMD_RESUME 4
#define CMD_ABORT 5
#define CMD_STOP 6
#define CMD_GETSTATE 7
#define CMD_GETPOSE 8
#define CMD_MOVE_ABS 15
#define CMD_MOVE_REL 16

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

int argc;
char **argv;

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

bool action(tms_msg_rp::action::Request &req, tms_msg_rp::action::Response &res)
{
  tms_msg_rc::msg::smartpal_control sp_control_srv;

  //----------------------------------------------------------------------
  sp_control_srv.request.unit = UNIT_ARM_R;
  sp_control_srv.request.cmd = CMD_MOVE_ABS;
  sp_control_srv.request.arg.resize(8);
  sp_control_srv.request.arg[0] = 0;
  sp_control_srv.request.arg[1] = -10;
  sp_control_srv.request.arg[2] = 0;
  sp_control_srv.request.arg[3] = 0;
  sp_control_srv.request.arg[4] = 0;
  sp_control_srv.request.arg[5] = 0;
  sp_control_srv.request.arg[6] = 0;
  sp_control_srv.request.arg[7] = 10;

  if (sp5_control_client.call(sp_control_srv))
    RCLCPP_INFO(this->get_logger(),"result: %d", sp_control_srv.response.result);
  else
    RCLCPP_ERROR("Failed to call service sp5_control");

  sp_control_srv.request.unit = UNIT_ARM_L;
  sp_control_srv.request.cmd = CMD_MOVE_ABS;
  sp_control_srv.request.arg.resize(8);
  sp_control_srv.request.arg[0] = 0;
  sp_control_srv.request.arg[1] = -10;
  sp_control_srv.request.arg[2] = 0;
  sp_control_srv.request.arg[3] = 0;
  sp_control_srv.request.arg[4] = 0;
  sp_control_srv.request.arg[5] = 0;
  sp_control_srv.request.arg[6] = 0;
  sp_control_srv.request.arg[7] = 10;

  if (sp5_control_client.call(sp_control_srv))
    RCLCPP_INFO(this->get_logger(),"result: %d", sp_control_srv.response.result);
  else
    RCLCPP_ERROR("Failed to call service sp5_control");

  cout << "PRM start" << endl;
  if (!PlanBase::instance()->targetArmFinger)
  {
    MessageView::mainInstance()->putln("error: Set Robot");
  }
  cout << "check setRobot" << endl;

  cnoid::Link *rl = PlanBase::instance()->targetArmFinger->bodyItemRobot->body()->rootLink();
  cout << "make linkt" << endl;

  MessageView::mainInstance()->putln("run SmartpalAction!");

  PlanBase *pb = PlanBase::instance();
  // setrobot
  pb->setTrajectoryPlanDOF();
  // setobject

  for (int i = 0; i < 18; i++)
  {
    pb->bodyItemRobot()->body()->joint(i)->q() = 0;
  }
  pb->bodyItemRobot()->body()->joint(3)->q() = deg2rad(-10.0);
  pb->bodyItemRobot()->body()->joint(11)->q() = deg2rad(10.0);

  pb->initial();
  pb->graspMotionSeq.clear();

  pb->setGraspingState(PlanBase::NOT_GRASPING);
  pb->setObjectContactState(PlanBase::ON_ENVIRONMENT);
  pb->graspMotionSeq.push_back(pb->getMotionState());

  bool success = GraspController::instance()->loadAndSelectGraspPattern();
  if (!success)
  {
    cout << "Error: Cannot find grasping posure" << endl;
  }

  MotionState graspMotionState = pb->getMotionState();

  Vector3 Pp_(pb->palm()->p());
  Matrix3 Rp_(pb->palm()->R());

  //==== Approach Point
  pb->setMotionState(graspMotionState);
  pb->arm()->IK_arm(Rp_ * pb->arm()->approachOffset + Pp_ + Vector3(0, 0, 0.05), Rp_);
  pb->setGraspingState(PlanBase::UNDER_GRASPING);
  for (int i = 0; i < pb->nFing(); i++)
  {
    for (int j = 0; j < pb->fingers(i)->fing_path->numJoints(); j++)
    {
      pb->fingers(i)->fing_path->joint(j)->q() = pb->fingers(i)->fingerOpenPose[j];
    }
  }
  pb->graspMotionSeq.push_back(pb->getMotionState());

  //==== Grasp Point and Hand open
  pb->setMotionState(graspMotionState);
  pb->setGraspingState(PlanBase::UNDER_GRASPING);
  for (int i = 0; i < pb->nFing(); i++)
  {
    for (int j = 0; j < pb->fingers(i)->fing_path->numJoints(); j++)
    {
      pb->fingers(i)->fing_path->joint(j)->q() = pb->fingers(i)->fingerOpenPose[j];
    }
  }
  pb->graspMotionSeq.push_back(pb->getMotionState());

  //==== hand close
  pb->setMotionState(graspMotionState);
  pb->setGraspingState(PlanBase::GRASPING);
  pb->graspMotionSeq.push_back(pb->getMotionState());

  //==== lift up
  pb->arm()->IK_arm(Vector3(Vector3(0, 0, 0.05) + Pp_), Rp_);

  pb->setObjectContactState(PlanBase::OFF_ENVIRONMENT);
  pb->graspMotionSeq.push_back(pb->getMotionState());

  //==== end point
  vector< double > closefinger;
  for (int i = 0; i < pb->nFing(); i++)
  {
    for (int j = 0; j < pb->fingers(i)->fing_path->numJoints(); j++)
    {
      closefinger.push_back(pb->fingers(i)->fing_path->joint(j)->q());
    }
  }

  for (int i = 0; i < 18; i++)
  {
    pb->bodyItemRobot()->body()->joint(i)->q() = 0;
  }
  pb->bodyItemRobot()->body()->joint(3)->q() = deg2rad(-10.0);
  pb->bodyItemRobot()->body()->joint(11)->q() = deg2rad(10.0);

  int cnt = 0;
  for (int i = 0; i < pb->nFing(); i++)
  {
    for (int j = 0; j < pb->fingers(i)->fing_path->numJoints(); j++)
    {
      pb->fingers(i)->fing_path->joint(j)->q() = closefinger[cnt++];
    }
  }
  pb->graspMotionSeq.push_back(pb->getMotionState());

  TrajectoryPlanner tp;
  success = tp.doTrajectoryPlanning();

  if (!success)
  {
    cout << "Error: Cannot find motion path" << endl;
  }

  res.result = (float)success;

  double jonit[18];

  for (int i = 0; i < tp.motionSeq.size(); i++)
  {
    for (int j = 0; j < tp.motionSeq[i].jointSeq.size(); j++)
    {
      jonit[j] = rad2deg(tp.motionSeq[i].jointSeq(j));
      cout << "jonit[" << j << "] : " << jonit[j] << endl;
    }

    sp_control_srv.request.unit = UNIT_ARM_R;
    sp_control_srv.request.cmd = CMD_MOVE_ABS;
    sp_control_srv.request.arg.resize(8);
    sp_control_srv.request.arg[0] = jonit[2];
    sp_control_srv.request.arg[1] = jonit[3];
    sp_control_srv.request.arg[2] = jonit[4];
    sp_control_srv.request.arg[3] = jonit[5];
    sp_control_srv.request.arg[4] = jonit[6];
    sp_control_srv.request.arg[5] = jonit[7];
    sp_control_srv.request.arg[6] = jonit[8];
    sp_control_srv.request.arg[7] = 10;

    if (sp5_control_client.call(sp_control_srv))
      RCLCPP_INFO(this->get_logger(),"result: %d", sp_control_srv.response.result);
    else
      RCLCPP_ERROR(this->get_logger(),"Failed to call service sp5_control");

    //----------------------------------------------------------------------------
    sp_control_srv.request.unit = UNIT_LUMBA;
    sp_control_srv.request.cmd = 16;
    sp_control_srv.request.arg.resize(4);
    sp_control_srv.request.arg[0] = jonit[0];
    sp_control_srv.request.arg[1] = jonit[1];
    sp_control_srv.request.arg[2] = 10;
    sp_control_srv.request.arg[3] = 10;

    if (sp5_control_client.call(sp_control_srv))
      RCLCPP_INFO(this->get_logger(),"result: %d", sp_control_srv.response.result);
    else
      RCLCPP_ERROR(this->get_logger(),"Failed to call service sp5_control");

    //----------------------------------------------------------------------------
    sp_control_srv.request.unit = UNIT_GRIPPER_R;
    sp_control_srv.request.cmd = CMD_MOVE_ABS;
    sp_control_srv.request.arg.resize(3);
    sp_control_srv.request.arg[0] = jonit[9];
    sp_control_srv.request.arg[1] = 10;
    sp_control_srv.request.arg[2] = 10;

    if (sp5_control_client.call(sp_control_srv))
      RCLCPP_INFO(this->get_logger(),"result: %d", sp_control_srv.response.result);
    else
      RCLCPP_ERROR(this->get_logger(),"Failed to call service sp5_control");

    sleep(3);
  }

  return true;
}


class smartpal5 : public rclcpp::Node
{
public:
  smartpal5()
  : Node("smartpal_node")
  {
    
    // node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

    get_data_client = this->create_client<tms_msg_db::msg::TmsdbGetData>("/tms_db_reader");
    arm_data_sub = this->create_subscription<std_msgs::msg::String>("/move_group/fake_controller_joint_states", 10, std::bind(&smartpal5::armCallback, this, _1));
    object_data_sub = this->create_subscription<std_msgs::msg::String>("/move_group/monitored_planning_scene", 10, std::bind(&smartpal5::ObjectDataUpdate, this, _1));
    publisher_ = this->create_publisher<tms_msg_db::msg::TmsdbStamped>("tms_db_data", 10);

    // tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_handle_);

  }

private:
  rclcpp::Client<tms_msg_db::msg::TmsdbGetData>::SharedPtr get_data_client;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_data_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr object_data_sub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  
  // rclcpp::Node::SharedPtr node_handle_;

};

void armCallback(const sensor_msgs::msg::JointState &msg)
  {
    armInfo ai;
    if (msg.name[0] == "l_gripper_thumb_joint")
    {
      ai.gripper_left = msg.position[0];
      ai.move = 1;
    }
    else if (msg.name[0] == "l_arm_j1_joint")
    {
      for (int i = 0; i < 7; i++)
      {
        ai.j_L[i] = msg.position[i];
      }
      ai.move = 0;
    }
    trajectory.push_back(ai);
  }

void ObjectDataUpdate(const moveit_msgs::msg::PlanningScene &msg)
  {
    if (msg.robot_state.attached_collision_objects.size() != 0)
    {  // grasped object
      int object_id = atoi(msg.robot_state.attached_collision_objects[0].object.id.c_str());

      g_oid = object_id;

      geometry_msgs::msg::PoseStamped pose, pose2;
      pose.header.frame_id = "/l_end_effector_link";
      pose.pose.position.x = msg.robot_state.attached_collision_objects[0].object.primitive_poses[0].position.x;
      pose.pose.position.y = msg.robot_state.attached_collision_objects[0].object.primitive_poses[0].position.y;
      pose.pose.position.z = msg.robot_state.attached_collision_objects[0].object.primitive_poses[0].position.z;

      pose.pose.orientation.x = msg.robot_state.attached_collision_objects[0].object.primitive_poses[0].orientation.x;
      pose.pose.orientation.y = msg.robot_state.attached_collision_objects[0].object.primitive_poses[0].orientation.y;
      pose.pose.orientation.z = msg.robot_state.attached_collision_objects[0].object.primitive_poses[0].orientation.z;
      pose.pose.orientation.w = msg.robot_state.attached_collision_objects[0].object.primitive_poses[0].orientation.w;

      listener->transformPose("/world_link", pose, pose2);

      g_ox = pose2.pose.position.x;
      g_oy = pose2.pose.position.y;
      g_oz = pose2.pose.position.z;
      tf2_ros::Quaternion q2(pose2.pose.orientation.x, pose2.pose.orientation.y, pose2.pose.orientation.z,
                        pose2.pose.orientation.w);
      tf2::Matrix3x3 m2(q2);
      m2.getRPY(g_orr, g_orp, g_ory);

      tms_msg_db::msg::TmsdbGetData srv;
      srv.request.tmsdb.id = object_id + sid_;

      if (get_data_client.call(srv))
      {
        // g_oz -= srv.response.tmsdb[0].offset_z;

        rclcpp::Time now = ros_clock_.now();  // GMT +9

        tms_msg_db::msg::TmsdbStamped db_msg;
        tms_msg_db::msg::Tmsdb current_pos_data;

        db_msg.header.frame_id = "/world";
        db_msg.header.stamp = now;

        current_pos_data.time = boost::posix_time::to_iso_extended_string(now.toBoost());
        current_pos_data.id = g_oid;
        current_pos_data.name = srv.response.tmsdb[0].name;
        current_pos_data.type = srv.response.tmsdb[0].type;
        current_pos_data.x = g_ox;
        current_pos_data.y = g_oy;
        current_pos_data.z = g_oz;
        current_pos_data.rr = g_orr;
        current_pos_data.rp = g_orp;
        current_pos_data.ry = g_ory;
        current_pos_data.offset_x = srv.response.tmsdb[0].offset_x;
        current_pos_data.offset_y = srv.response.tmsdb[0].offset_y;
        current_pos_data.offset_z = srv.response.tmsdb[0].offset_z;
        current_pos_data.place = 2003;
        current_pos_data.sensor = 3003;
        current_pos_data.probability = 1.0;
        current_pos_data.state = 2;

        db_msg.tmsdb.push_back(current_pos_data);
        pose_publisher.publish(db_msg);

        is_grasp = true;
        grasping_object_id = g_oid;
      }
      else
      {
       RCLCPP_ERROR(this->get_logger(),"failed to get data");
      }
    }
    if (is_grasp == true && msg.world.collision_objects.size() != 0 &&
        msg.world.collision_objects[0].primitive_poses.size() != 0)
    {  // released object
      is_grasp = false;
      grasping_object_id = 0;

      int object_id = atoi(msg.world.collision_objects[0].id.c_str());

      g_oid = object_id;
      g_ox = msg.world.collision_objects[0].primitive_poses[0].position.x;
      g_oy = msg.world.collision_objects[0].primitive_poses[0].position.y;
      g_oz = msg.world.collision_objects[0].primitive_poses[0].position.z;

      tms_msg_db::msg::TmsdbGetData srv;
      srv.request.tmsdb.id = object_id + sid_;

      if (get_data_client.call(srv))
      {
        // g_oz -= srv.response.tmsdb[0].offset_z;

        rclcpp::Time now = ros_clock_.now();  // GMT +9

        tms_msg_db::msg::TmsdbStamped db_msg;
        tms_msg_db::msg::Tmsdb current_pos_data;

        db_msg.header.frame_id = "/world";
        db_msg.header.stamp = now;

        current_pos_data.time = boost::posix_time::to_iso_extended_string(now.toBoost());
        current_pos_data.id = g_oid;
        current_pos_data.name = srv.response.tmsdb[0].name;
        current_pos_data.type = srv.response.tmsdb[0].type;
        current_pos_data.x = g_ox;
        current_pos_data.y = g_oy;
        current_pos_data.z = g_oz;
        current_pos_data.rr = g_orr;
        current_pos_data.rp = g_orp;
        current_pos_data.ry = g_ory;
        current_pos_data.offset_x = srv.response.tmsdb[0].offset_x;
        current_pos_data.offset_y = srv.response.tmsdb[0].offset_y;
        current_pos_data.offset_z = srv.response.tmsdb[0].offset_z;
        current_pos_data.place = 5002;
        current_pos_data.sensor = 3003;
        current_pos_data.probability = 1.0;
        current_pos_data.state = 1;

        db_msg.tmsdb.push_back(current_pos_data);
        pose_publisher.publish(db_msg);
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(),"failed to get data");
      }
    }
  }


int main(int argc, char * argv[])
{
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<smartpal5>());
    rclcpp::shutdown();

    // nh.setParam("/2003_is_real", true);


    listener = new tf::TransformListener;

    int32_t id_robot = 2003;                // SmartPal5_2 ID
    int32_t id_odometry_and_joints = 3003;  // Sensor ID
    int32_t id_place = 5002;                // Place ID

    //--------------------------------------------------------------------------
    // smartpal initialize
    uint8_t cnt = 0;
    while (!smartpal->Initialize())
    {
      cnt++;

      printf("CORBA client object initialization has been failed.\n");
      printf("Retry initializing...\n");

      sleep(3);  // delay 3sec

      if (cnt == 5)
      {
        smartpal->Shutdown();
        return (0);
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

    sleep(2);

    // power on
    smartpal->lumbaSetPower(ON);
    smartpal->armSetPower(ArmR, ON);
    smartpal->armSetPower(ArmL, ON);
    smartpal->vehicleSetPower(ON);

    sleep(2);

    // servo on
    smartpal->vehicleSetServo(ON);
    smartpal->gripperSetServo(GripperR, ON);
    smartpal->gripperSetServo(GripperL, ON);
    smartpal->lumbaSetServo(ON);

    smartpal->armSetServo(ArmR, ON);
    smartpal->armSetServo(ArmL, ON);

    smartpal->armGetSoftLimit(ArmL);

    rclcpp::Clock tNow;
    rclcpp::rate rate(10);  // 10Hz frequency (0.1 sec)

    double x;
    double y;
    double th;
    double gripperR;
    double gripperL;
    double waistL;
    double waistH;
    double jointR[7];
    double jointL[7];

    //--------------------------------------------------------------------------
    while (rclcpp::ok())
    {
      //----------------------------------------------------------------------
      rclcpp::Time now = ros_clock_.now();  // GMT +9

      tms_msg_db::msg::TmsdbStamped db_msg;
      tms_msg_db::msg::Tmsdb current_pos_data;

      if (getRobotCurrentPos(&x, &y, &th, &waistL, &waistH, jointR, &gripperR, jointL, &gripperL) == 1)
      {
        db_msg.header.frame_id = "/world";
        db_msg.header.stamp = now;

        current_pos_data.time = boost::posix_time::to_iso_extended_string(now.toBoost());
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

        pose_publisher.publish(db_msg);
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(),"Failed to getRobotCurrentPos\n");
      }

      //----------------------------------------------------------------------
      rclcpp::spinOnce();
      rate.sleep();

      //----------------------------------------------------------------------
    }

    //--------------------------------------------------------------------------
    // servo off
    smartpal->vehicleSetServo(OFF);
    smartpal->armSetServo(ArmR, OFF);
    smartpal->armSetServo(ArmL, OFF);
    smartpal->gripperSetServo(GripperR, OFF);
    smartpal->gripperSetServo(GripperL, OFF);
    smartpal->lumbaSetServo(OFF);

    sleep(2);

    // power off
    smartpal->vehicleSetPower(OFF);
    smartpal->armSetPower(ArmR, OFF);
    smartpal->armSetPower(ArmL, OFF);
    smartpal->lumbaSetPower(OFF);

    smartpal->Shutdown();

    delete listener;


    return 0;
}
