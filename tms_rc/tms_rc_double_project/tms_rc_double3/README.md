# Double 3 controller by ROS/ROS2 using SDK and WebRTC

This code is for controlling Double 3 by ROS/ROS2 using [rosbridge_suite](http://wiki.ros.org/rosbridge_suite) or [ros2-web-bridge](https://github.com/RobotWebTools/ros2-web-bridge), [Double 3](https://www.doublerobotics.com) SDK, and the native WebRTC implementation.

This consists of two pages, the local controller (robot.html) on Double 3 and the remote controller on your web browser (driver.html) such as Firefox.

Signaling server (WSS server) is required. You can use your Glitch project server.

## Getting Started

- **GLITCH** Signaling server (ex. your Glitch project server)
- **DOUBLE3** IP address of Double 3
- **WEBSERVER** IP address of local webserver
- **ROSBRIGDE** IP address of rosbridge server

I recommend **WEBSERVER** and **ROSBRIGDE** are the same Ubuntu server on WSL in Windows 10.

1. [Remix this on Glitch](https://glitch.com/edit/#!/remix/somber-persistent-line). 
1. Install ROS or ROS2 in **ROSBRIGDE**.
    - **ROS** [ROS Melodic](http://wiki.ros.org/melodic)
    - **ROS2** [ROS 2 Dashing Diademata](https://index.ros.org/doc/ros2/)
1. Install rosbridge in **ROSBRIGDE**.
    - **ROS** [rosbridge_suite](http://wiki.ros.org/rosbridge_suite)
    - **ROS2** [Nodejs](https://nodejs.org) and [ros2-web-bridge](https://github.com/RobotWebTools/ros2-web-bridge),
1. Install webserver in **WEBSERVER**.
    - **ROS** [roswww](http://wiki.ros.org/roswww ) or python2/python 3
    - **ROS2** python2/python 3
1. Download "public" folder to **WEBSERVER**.
1. Edit “public/robot.html” and “public/index.js” in **WEBSERVER**.
    1. Set **ROSBRIGDE** in robot.html
        ```html
		<td>
		<input type="text" id="wsaddress" size="20" maxlength="40" value="ws://[ROSBRIDGE]:9090">
		</td>
        ```
    1. Set **GLITCH** in index.js
        ```java
        var socket = null;
        function connectWebsocket() {
            //socket = new WebSocket("wss://" + window.location.hostname);
            socket = new WebSocket("wss://[GLITCH]");
        ```
1. Start webserver  
    **roswww** (ROS only)
    ```
    cd ~/catkin_ws/src
    catkin_create_pkg d3 std_msgs rospy roscpp
    cd d3
    mkdir www
    cd www
    cp -r [public directory] .
    catkin_make
    roslaunch roswww roswww.launch
    ```
    **python2**
    ```
    cd [public directory]
    python -m SimpleHTTPServer 8085
    ```
    **python3**
    ```
    cd [public directory]
    python3 -m http.server 8085
    ```
1. Start rosbridge server  
    **ROS**
    ```
    roslaunch rosbridge_server rosbridge_websocket.launch
    ```
    **ROS2**
    ```
    node ~/ros2-web-bridge/bin/rosbridge.js
    ```
1. Access Double 3 Monitor (http://**DOUBLE3**:8080/) from your web browser.   
    Set 'http://**WEBSERVER**:8085/d3/public/robot.html' (roswww) or 'http://**WEBSERVER**:8085/robot.html' (python2/3) in "Standby GUI" in Double 3 Monitor and press "GO" button.
1. Access **GLITCH** from your web browser by pressing "Show" and "In a New Window" on the top of your Glitch project page.   
1. Press "connect" botton on Double screen.   
1. Press "Call" botton on your browser. You may have to press "Call" bottun several times.   

## Control examples

**ROS**
- rostopic pub /kickstand std_msgs/Bool '{data: true}' --once
- rostopic pub /pole std_msgs/Int32 '{data: 0}' --once
- rostopic pub /enableNavigation std_msgs/Bool '{data: true}' --once
- rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}, angular: {z: 0.3}}' --once
- rostopic pub /cmd_pos geometry_msgs/Twist '{linear: {x: 0.1, y: 0}}' --once
- rostopic pub /enableNavigation std_msgs/Bool '{data: true}' --once
- rostopic pub /resetOrigin std_msgs/Bool '{data: true}' --once
- rosrun turtlebot3_teleop turtlebot3_teleop_key
- rostopic echo /pos

**ROS 2**
- ros2 topic pub /kickstand std_msgs/Bool '{data: true}' --once
- ros2 topic pub /pole std_msgs/Int32 '{data: 0}' --once
- ros2 topic pub /enableNavigation std_msgs/Bool '{data: true}' --once
- ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}, angular: {z: 0.3}}' --once
- ros2 topic pub /cmd_pos geometry_msgs/Twist '{linear: {x: 0.1, y: 0}}' --once
- ros2 topic pub /enableNavigation std_msgs/Bool '{data: true}' --once
- ros2 topic pub /resetOrigin std_msgs/Bool '{data: true}' --once
- ros2 run key_teleop key_teleop /key_vel:=/cmd_vel
- ros2 topic echo /pos