import rclpy
import math

from rclpy.node import Node
from std_msgs.msg import String
from tms_msg_ss.msg import ViconData
from tms_msg_rc.srv import TurtlebotControl
from tms_msg_db.msg import Tmsdb, TmsdbStamped
from tms_msg_db.srv import TmsdbGetData
from rclpy.callback_groups import ReentrantCallbackGroup




from geometry_msgs.msg import Point, PoseStamped, Quaternion, TransformStamped

# Doubleの初期位置を保持するための大域変数
x = 0.0
y = 0.0
th = 0.0

#現在地を更新するための大域変数(for odom)
pos_x = 0.0
pos_y = 0.0
ori_th = 0.0

#現在地を更新するための大域変数(for vicon)
v_pos_x = 0.0
v_pos_y = 0.0
v_ori_th = 0.0

class ViconControl(Node):


    def __init__(self):
        global x, y, th, pos_x, pos_y, ori_th, v_pos_x, v_pos_y, v_ori_th
        super().__init__('ViconControl')

        self.cb_group = ReentrantCallbackGroup()
        self.srv = self.create_service(TurtlebotControl, 'tms_rc_double', self.callback, callback_group=self.cb_group)
        self.db_client = self.create_client(TmsdbGetData,"/tms_db_reader")
        self.subscription = self.create_subscription(ViconData,"vicon_stream/output",self.vicon_sysCallback,10, callback_group=self.cb_group)



       
        th = 0
        timer_period = 1  # seconds
        
        self.calltimer = self.create_timer(0.01, self.calltimercallback, callback_group=self.cb_group)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    async def calltimercallback(self):
        self.calltimer.cancel()

        req = TmsdbGetData.Request()
        req.tmsdb.id = 2012  
        req.tmsdb.sensor = 3001
        result = await self.db_client.call_async(req)
        var_pos_x = result.tmsdb[0].x
        var_pos_y = result.tmsdb[0].y
        ori_th = result.tmsdb[0].ry

        roll = result.tmsdb[0].rr
        pitch = result.tmsdb[0].rp
        yaw = result.tmsdb[0].ry


    async def timer_callback(self):
        global x, y, th, pos_x, pos_y, ori_th, v_pos_x, v_pos_y, v_ori_th

        req = TmsdbGetData.Request()
        req.tmsdb.id = 2012  
        req.tmsdb.sensor = 3001
        result = await self.db_client.call_async(req)
        var_pos_x = result.tmsdb[0].x
        var_pos_y = result.tmsdb[0].y
        ori_th = result.tmsdb[0].ry

        roll = result.tmsdb[0].rr
        pitch = result.tmsdb[0].rp
        yaw = result.tmsdb[0].ry
    
    def vicon_sysCallback(self, msg):
        global x, y, th, pos_x, pos_y, ori_th, v_pos_x, v_pos_y, v_ori_th
        txt = msg.subjectName 
        v_name = txt.split("#")
        if len(v_name) == 2:
            id = int(v_name[0])
            name = v_name[1]
        
    
        if id == 2012:
            v_pos_x = msg.translation.x
            v_pos_y = msg.translation.y
            v_pos_z = msg.translation.z
            v_rotation = msg.rotation
            var_pos_x = v_pos_x
            var_pos_y = v_pos_y
        pub_tf()
    
    async def callback(self, request, response):
        global x, y, th, pos_x, pos_y, ori_th, v_pos_x, v_pos_y, v_ori_th
        if request.cmd == 0 :
            if len(request.arg) != 3 or request.arg[0] < 0 or request.arg[0] > 8000 or request.arg[1] < 0 or request.arg[1] > 4500 or request.arg[2] < -180 or request.arg[2] > 180 :
                self.get_logger().info("case0 : An illegal arguments' type.\n") 
                response.result = 0
                return response

            current_x = v_pos_x
            current_y = v_pos_y
            current_th = v_ori_th
            req = TmsdbGetData.Request()
            req.tmsdb.id = 2012
            req.tmsdb.sensor = sensor

            result = await self.db_client.call_async(req)
            current_x = result.tmsdb[0].x
            current_y = result.tmsdb[0].y

            goal_distance = distance(current_x, current_y, request.arg[0], request.arg[1])
            goal_theta = request.arg[2] - current_th

            if (goal_theta > 180.0) :
                goal_theta = goal_theta - 360.0
            elif (goal_theta < -180.0) :
                goal_theta = goal_theta + 360.0

            if (goal_distance >= 5 and goal_theta < 2) :
                command = 1
            elif (goal_distance >= 5 and goal_theta >= 2) :
                command = 2

            control_base(command, goal_distance, goal_theta)
            response.result = 1
        
        elif request.cmd == 1 :
            if len(request.arg) != 3 or request.arg[0] < 0 or request.arg[0] > 8000 or request.arg[1] < 0 or request.arg[1] > 4500 or request.arg[2] < -180 or request.arg[2] > 180 :
                self.get_logger().info("case0 : An illegal arguments' type.\n") 
                response.result = 0
                while goal_theta > 180.0 :
                    goal_theta = goal_theta - 360.0
                while goal_theta < -180.0 :
                    goal_theta = goal_theta + 360.0


                return response

            current_x = v_pos_x
            current_y = v_pos_y
            current_th = v_ori_th
            req = TmsdbGetData.Request()
            req.tmsdb.id = 2012
            req.tmsdb.sensor = sensor

            result = await self.db_client.call_async(req)
            current_x = result.tmsdb[0].x
            current_y = result.tmsdb[0].y

            goal_distance = distance(current_x, current_y, request.arg[0], request.arg[1])
            goal_theta = request.arg[2] - current_th

            while goal_theta > 180.0 :
                goal_theta = goal_theta - 360.0
            while goal_theta < -180.0 :
                goal_theta = goal_theta + 360.0

            if len(request.arg) != 2 or request.arg[0] < 0 or request.arg[0] > 9179 or request.arg[1] < -180 or request.arg[1] > 180 :
                self.get_logger().info("case0 : An illegal arguments' type.\n") 
                response.result = 0
                return response


            if (goal_distance >= 5 and goal_theta < 2) :
                command = 1
            elif (goal_distance >= 5 and goal_theta >= 2) :
                command = 2
            print(f"command:{command}: goal_dis={request.arg[0]}, goal_arg={request.arg[1]}")

    
            control_base(command, goal_distance, goal_theta)
            response.result = 1
        else :
            self.get_logger().info("case0 : An illegal arguments' type.\n") 
            response.result = 0
            return response
        pub_tf()
        return response
    
    
    def distance(self, x0, y0, x1, y1):
        return math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0))

    def pub_tf(self) :
        global x, y, th, pos_x, pos_y, ori_th, v_pos_x, v_pos_y, v_ori_th
        broadcaster = tf2_ros.TransformBroadcaster()

        robot_quat = Quaternion() 
        q = quaternion_from_euler(0,0,math.radians(ori_th))
        robot_quat.x = q[0]
        robot_quat.y = q[1]
        robot_quat.z = q[2]
        robot_quat.w = q[3]

        ts = TransformStamped() 
        ts.header.frame_id = "map"
        ts.child_frame_id = "base_footprint"
        ts.header.stamp = self.get_clock().now()
        ts.transform.translation.x = var_pos_x / 1000
        ts.transform.translation.y = var_pos_y /1000 
        ts.transform.translation.z = 0
        ts.transform.rotation = robot_quat
        #ifdef VICON
        ts.transform.rotation = v_rotation
        #endif
        broadcaster.sendTransform(ts)

    async def control_base(self, command, goal_dis, goal_ang):

        global x, y, th, pos_x, pos_y, ori_th, v_pos_x, v_pos_y, v_ori_th
        ini_pos_x = v_pos_x
        ini_pos_y = v_pos_y
        ini_ori_th = v_ori_th

        var_pos_x = v_pos_x
        var_pos_y = v_pos_y
        var_ori_th = v_ori_th

        req = TmsdbGetData.Request()
        req.tmsdb.id = 2012  
        req.tmsdb.sensor = 3001

        result = await self.db_client.call_async(req)
        ini_pos_x = req.response.tmsdb[0].x    
        ini_pos_y = req.response.tmsdb[0].y

        var_pos_x = result.tmsdb[0].x
        var_pos_y = result.tmsdb[0].y
        
                
        if command == 0 or command == 2 :
            if goal_ang > 0:
                while True:
                    if ini_ori_th + goal_ang > 180 :
                        if (ini_ori_th <= var_ori_th and var_ori_th <= 180) or (-180 <= var_ori_th and var_ori_th < ini_ori_th + goal_ang - 360) :
                            var_ori_th = v_ori_th
                        else :
                            break

                    else :
                        if math.fabs(var_ori_th - ini_ori_th) < math.fabs(goal_ang):
                            var_ori_th = v_ori_th
                        else :
                            break
            elif goal_ang < 0:
                while True:
                    if ini_ori_th + goal_ang < -180 :
                        if (-180 <= var_ori_th and var_ori_th <= ini_ori_th) or (360 + goal_ang + ini_ori_th < var_ori_th and var_ori_th <= 180) :
                            var_ori_th = v_ori_th
                        else :
                            break

                    else :
                        if math.fabs(var_ori_th - ini_ori_th) < math.fabs(goal_ang):
                            var_ori_th = v_ori_th
                        else :
                            break
        if (command == 1 or command == 2) :
            if goal_dis > 0:
                while True:
                    if distance(ini_pos_x, ini_pos_y, var_pos_x, var_pos_y) < goal_dis :
                        var_pos_x = v_pos_x
                        var_pos_y = v_pos_y

                        result = await self.db_client.call_async(req)
                        var_pos_x = result.tmsdb[0].x
                        var_pos_y = result.tmsdb[0].y
        
                    else :
                        break
        return true



def main(args=None):

    rclpy.init(args=args)
    
    viconcontrol = ViconControl()

    rclpy.spin(viconcontrol)

    viconcontrol.destory_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
