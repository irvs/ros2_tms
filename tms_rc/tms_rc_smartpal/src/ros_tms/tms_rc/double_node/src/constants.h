/*******************************************************************************
* Copyright 2019 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Darby Lim */

#include <chrono>
using namespace std::chrono_literals;

namespace doublenode
{
constexpr char SensorStateTopic[] = "sensor_state";
constexpr char JointStateTopic[] = "joint_states";
constexpr char OdomTopic[] = "odom";
constexpr char ScanHalfTopic[] = "scan_half";
constexpr char ScanTopic[] = "scan";
constexpr char ImuTopic[] = "imu";
constexpr char TimeTopic[] = "time_sync";

constexpr auto JointStatePublishPeriodMillis = 33ms;
constexpr auto ScanPublishPeriodMillis = 200ms;
constexpr auto OdometryPublishPeriodMillis = 33ms;
constexpr auto TimeSyncPublishPeriodMillis = 1000ms;

constexpr double WheelRadius = 0.129f;
}
