#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import Int64 
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
import numpy as np
# import time
ack = Int64()
a = int()
l = int()
x = String()
x.data = 'start'
out1=np.empty(0)
out2=np.empty(0)
out3=np.empty(1)
start = False
stop = False
class Node1(Node):
    def __init__(self):
        global x
        super().__init__("Simulation_node")
        print(self)
        self.path_publisher_ = self.create_subscription(Int64MultiArray, 'path_transmitter',self.find_path, 10)
        self.ack_reply= self.create_publisher(String, 'ack_reciever', 10)
        self.ack_reply.publish(x)
        self.ack_reciever_ = self.create_subscription(Int64, "ack_transmitter",self.control, 10)
        print("attempting connection with path_follower")
        self.velocity_publisher = self.create_publisher(Twist,'/demo/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Odometry,'/demo/odom',self.updatepose,10)
        self.pose= Odometry()
        self.vel_msg = Twist()
        self.goal_pose= Odometry()
        self.rate= super().create_rate(2)
    def move(self):
        vel_msg =self.vel_msg
        global out1,out2,out3
        print(out1,out2)
        for i in range(1,len(out1)):
            print("turning")
            self.goal_pose.pose.pose.position.x=out1[i]
            self.goal_pose.pose.pose.position.y=out2[i]
            
            while self.steering_angle(self.goal_pose) - self.pose.pose.pose.orientation.z >= 2:
                vel_msg.angular.z = 0.1
                self.velocity_publisher.publish(vel_msg)
                print(self.steering_angle(self.goal_pose) - self.pose.pose.pose.orientation.z)
            
            self.goal_pose.pose.pose.position.x = out1[i]
            self.goal_pose.pose.pose.position.y = out2[i]
            print(self.goal_pose.pose.pose.position.x ,self.goal_pose.pose.pose.position.y)
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.linear.x =0.09
            vel_msg.angular.z = 0.1
            print(vel_msg)
            self.velocity_publisher.publish(vel_msg)
            print(self.pose.pose.pose.orientation.z)
            print(self.pose.pose.pose.orientation.y)
            print(self.pose.pose.pose.position.x)
            print(self.pose.pose.pose.position.y)
        
#         self.rate.sleep()
    def updatepose(self,msg):
        self.pose = msg
#         self.pose.pose.pose.position.x = round(self.pose.pose.pose.position.x, 4)
#         self.pose.pose.pose.position.y = round(self.pose.pose.pose.position.y, 4)
    def euclidean_distance(self, goal_pose):
        print("distance")
        return sqrt(pow((goal_pose.pose.pose.position.x - self.pose.pose.pose.position.x), 2) +pow((goal_pose.pose.pose.position.y - self.pose.pose.pose.position.y), 2))
    def linear_vel(self, goal_pose, constant=.5):
        return constant * self.euclidean_distance(goal_pose)
    def steering_angle(self, goal_pose):
        return atan2(goal_pose.pose.pose.position.y - self.pose.pose.pose.position.y, goal_pose.pose.pose.position.x - self.pose.pose.pose.position.x)
    def angular_vel(self, goal_pose, constant=1):
        print(self.steering_angle(goal_pose))
        return constant * (self.steering_angle(goal_pose) - self.pose.pose.pose.orientation.z)
    def control(self,msg):
        global a
        global start
        global stop
        global out1,out2,out3
        if start == False or stop == False:
#           self.get_logger().info('i "%s"' %msg.data)
            a=msg.data
            if a == 5:
                print("connection established")
            elif a==0:
                print("started")
                start=True
            elif a== 4 and start == True:
                stop = True
                print("stopped")
                out3=np.vstack((out1,out2))
                print(out3)
                self.a= self.create_timer(1,self.move)
                print("timer started")
#           if ack == 0:
#             ac.data = 'r'
#             a=self.ack_reply
#             print("ack replied",ac.data)

    def find_path(self,msg):
        global data
        global a
        global out1
        global out2,out3
        global start
        global stop
        data = Int64()
        data = msg.data
#       print(data)
        if start == True and stop ==False :
            if a == 1:
                out1=np.append(out1,data)
            if a == 3:
                out2=np.append(out2,data)
def main(args=None):
    rclpy.init(args=args)
    node = Node1()
    rclpy.spin(node)
    print("end")
    rclpy.shutdown()
if __name__ == '__main__':
    main()