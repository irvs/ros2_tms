#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


#include <vicon_stream/client.h>

#include <tms_msg_db/msg/tmsdb_stamped.h>
#include <tms_msg_db/msg/tmsdb.h>
#include <tms_msg_ss/msg/vicon_data.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/algorithm/string.hpp>
#include <visualization_msgs/msg/marker.h>

#include <iostream>
#include <fstream>
#include <cassert>
#include <ctime>
#include <time.h>
#include <string>
#include <chrono>

using std::string;
// using namespace ViconDataStreamSDK;
using namespace ViconDataStreamSDK::CPP;
using namespace boost;
using namespace std;
using namespace std::chrono_literals;

// export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/lib/x86_64-linux-gnu/"

string Adapt(const Direction::Enum i_Direction)
{
  switch (i_Direction)
  {
    case Direction::Forward:
      return "Forward";
    case Direction::Backward:
      return "Backward";
    case Direction::Left:
      return "Left";
    case Direction::Right:
      return "Right";
    case Direction::Up:
      return "Up";
    case Direction::Down:
      return "Down";
    default:
      return "Unknown";
  }
}
std::string Adapt(const bool i_Value)
{
  return i_Value ? "True" : "False";
}



class ViconStream : public rclcpp::Node
{ 

private:
  // Sensor ID
  int32_t idSensor;
  // Place ID
  int32_t idPlace;
  // // Parameters:
  string stream_mode;
  string host_name;

  string frame_id;

  double update_time;
  bool isDebug;
  // Access client for ViconSDK
  // ViconDataStreamSDK::CPP::Client MyClient;
  // Output_IsConnected Output = MyClient.IsConnected();]
  ViconDataStreamSDK::CPP::Client MyClient;


public:
  //ViconStream(idSensor, idPlace, stream_mode, host_name, frame_id, update_time, isDebug)
  ViconStream()
  : Node("Vicon")
  , count_(0)
  , idPlace(5001)
  , stream_mode("ClientPull")
  , frame_id("/world")
  , update_time(0.01)
  , host_name("192.168.4.151:801")
  , isDebug(true)
    {
      // auto node = std::make_shared<rclcpp::Node>("debug");
      // node->get_parameter_or("debug", isDebug, isDebug);
    //nh_priv.param("debug", isDebug, isDebug);
    // Publishers
      // auto db_pub = std::make_shared<ViconStream>("tms_db_data");
    //  dbpub = this->create_publisher<std_msgs::msg::String>("~/tms_db_data");
    //db_pub = nh.advertise< tms_msg_db::TmsdbStamped >("tms_db_data", 1);

    // TimerEvent
      init_vicon();
      updatetimer = this->create_wall_timer(100ms, std::bind(&ViconStream::updateCallback, this));
    }


// Output_IsConnected Output = MyClient.IsConnected()
// // Output.Connected == false
// MyClient.Connect( "localhost" );
// Output_IsConnected Output = MyClient.IsConnected()
// // Output.Connected == true
// // (assuming localhost is serving)

private:
  bool init_vicon()
  {
    std::cout << "Connecting to Vicon DataStream SDK at " << host_name << " ..." << std::endl;

    while (!MyClient.IsConnected().Connected)
    {
      MyClient.Connect(host_name);
      sleep(1);
    }
    if(MyClient.IsConnected().Connected){
      std::cout << "Connected" << std::endl;
    }
    Output_SetStreamMode result;

    if (stream_mode == "ClientPull")
    {
      result = MyClient.SetStreamMode(StreamMode::ClientPull);
    }
    else
    {
      return false;
    }

    if (result.Result != Result::Success)
    {
      std::cout << "Set stream mode call failed -- shutting down" <<std::endl;
    }

    MyClient.SetAxisMapping(Direction::Forward, Direction::Left, Direction::Up);  // 'Z-up'

    Output_GetAxisMapping _Output_GetAxisMapping = MyClient.GetAxisMapping();
    std::cout <<"Axis Mapping: X-" << Adapt(_Output_GetAxisMapping.XAxis) << " Y-"
                                       << Adapt(_Output_GetAxisMapping.YAxis) << " Z-"
                                       << Adapt(_Output_GetAxisMapping.ZAxis) <<std::endl;
    Output_GetVersion _Output_GetVersion = MyClient.GetVersion();
    std::cout << "Version: " << _Output_GetVersion.Major << "." << _Output_GetVersion.Minor << "."
                                << _Output_GetVersion.Point << std::endl;
    return true;
  }


 void updateCallback()
  //void updateCallback(const ros::TimerEvent& e)
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    dbpub->publish(message);
    // Enable some different data types
    MyClient.EnableSegmentData();
    MyClient.EnableMarkerData();
    MyClient.EnableUnlabeledMarkerData();
    MyClient.EnableDeviceData();

   if (isDebug)
    {
      std::cout << "Segment Data Enabled: " << Adapt(MyClient.IsSegmentDataEnabled().Enabled) << std::endl;
      std::cout << "Marker Data Enabled: " << Adapt(MyClient.IsMarkerDataEnabled().Enabled) << std::endl;
      std::cout << "Unlabeled Marker Data Enabled: " << Adapt(MyClient.IsUnlabeledMarkerDataEnabled().Enabled)
                << std::endl;
      std::cout << "Device Data Enabled: " << Adapt(MyClient.IsDeviceDataEnabled().Enabled) << std::endl;
    }

    if (isDebug)
      std::cout << "Waiting for new frame...";
    while (MyClient.GetFrame().Result != Result::Success)
    {
      std::cout << "." << std::endl;
    }

    // Get the frame number
    Output_GetFrameNumber _Output_GetFrameNumber = MyClient.GetFrameNumber();
    if (isDebug)
      std::cout << "Frame Number: " << _Output_GetFrameNumber.FrameNumber << std::endl;

    // Count the number of subjects
    unsigned int SubjectCount = MyClient.GetSubjectCount().SubjectCount;
    if (isDebug)
      std::cout << "Subjects (" << SubjectCount << "):" << std::endl;

    ///////////////////////////////////////////////////////////////////
    for (unsigned int SubjectIndex = 0; SubjectIndex < SubjectCount; ++SubjectIndex)
    {
      if (isDebug)
        std::cout << "  Subject #" << SubjectIndex << std::endl;

      // Get the subject name
      std::string SubjectName = MyClient.GetSubjectName(SubjectIndex).SubjectName;
      if (isDebug)
        std::cout << "    Name: " << SubjectName << std::endl;

      // Get the root segment
      std::string RootSegment = MyClient.GetSubjectRootSegmentName(SubjectName).SegmentName;
      if (isDebug)
        std::cout << "    Root Segment: " << RootSegment << std::endl;

      // Count the number of segments
      unsigned int SegmentCount = MyClient.GetSegmentCount(SubjectName).SegmentCount;
      if (isDebug)
        std::cout << "    Segments (" << SegmentCount << "):" << std::endl;

      for (unsigned int SegmentIndex = 0; SegmentIndex < SegmentCount; ++SegmentIndex)
      {
        if (isDebug)
          std::cout << "      Segment #" << SegmentIndex << std::endl;

        // Get the segment name
        std::string SegmentName = MyClient.GetSegmentName(SubjectName, SegmentIndex).SegmentName;
        if (isDebug)
          std::cout << "        Name: " << SegmentName << std::endl;

        // Get the global segment translation
        Output_GetSegmentGlobalTranslation _Output_GetSegmentGlobalTranslation =
            MyClient.GetSegmentGlobalTranslation(SubjectName, SegmentName);
        if (isDebug)
          std::cout << "        Global Translation: (" << _Output_GetSegmentGlobalTranslation.Translation[0] << ", "
                    << _Output_GetSegmentGlobalTranslation.Translation[1] << ", "
                    << _Output_GetSegmentGlobalTranslation.Translation[2] << ") "  << std::endl;

        // Get the global segment rotation in quaternion co-ordinates
        Output_GetSegmentGlobalRotationQuaternion _Output_GetSegmentGlobalRotationQuaternion =
            MyClient.GetSegmentGlobalRotationQuaternion(SubjectName, SegmentName);
        if (isDebug)
          std::cout << "        Global Rotation Quaternion: (" << _Output_GetSegmentGlobalRotationQuaternion.Rotation[0]
                    << ", " << _Output_GetSegmentGlobalRotationQuaternion.Rotation[1] << ", "
                    << _Output_GetSegmentGlobalRotationQuaternion.Rotation[2] << ", "
                    << _Output_GetSegmentGlobalRotationQuaternion.Rotation[3] << ") " << std::endl;

        // Get the global segment rotation in EulerXYZ co-ordinates
        Output_GetSegmentGlobalRotationEulerXYZ _Output_GetSegmentGlobalRotationEulerXYZ =
            MyClient.GetSegmentGlobalRotationEulerXYZ(SubjectName, SegmentName);
        if (isDebug)
          std::cout << "        Global Rotation EulerXYZ: (" << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[0]
                    << ", " << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[1] << ", "
                    << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[2] << ") " << std::endl;
      }
    }


  }
  // Publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dbpub;
  // Timer
  rclcpp::TimerBase::SharedPtr updatetimer;
  size_t count_;  
};

int main(int argc, char * argv[])
{
  
  rclcpp::init(argc, argv);
  // ViconStream vs;
  rclcpp::spin(std::make_shared<ViconStream>());
  rclcpp::shutdown();

  return 0;
}
