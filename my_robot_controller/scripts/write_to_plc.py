#!/usr/bin/env python3
import roslib; roslib.load_manifest('my_robot_controller')
import math
from math import sin, cos
import rospy
import tf
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from connection import RS485

def callback(msg):
    global myData
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo(msg)
    myData = msg


def listener():
    rospy.init_node('omni_robot')
    pub = rospy.Publisher("/odom/kinematic_raw", Odometry, queue_size=1)
    msg1= Odometry()
    rospy.Subscriber('cmd_vel', Twist, callback)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(2)
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    x_pos_=0.0
    y_pos_=0.0
    th=0.0
    global alpha
    global d
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        # Then set your wheel speeds (using wheel_left and wheel_right as examples)
        v1_dat = math.sqrt(2)/2*(-myData.linear.x + myData.linear.y) + myData.angular.z*d

        v2_dat = math.sqrt(2)/2*(-myData.linear.x - myData.linear.y) + myData.angular.z*d

        v3_dat = math.sqrt(2)/2*( myData.linear.x - myData.linear.y) + myData.angular.z*d

        v4_dat = math.sqrt(2)/2*( myData.linear.x + myData.linear.y) + myData.angular.z*d

        rs1.write('reg',100,int(v1_dat))
        rs1.write('reg',200,int(v3_dat))

        rs2.write('reg',100,int(v2_dat))
        rs2.write('reg',200,int(v4_dat))

        # Doc van toc v1 (van toc banh 1)
        v1 = rs1.read('hr',40)
        if v1 > 32768:
            v1_thuc = v1 - 65536 
        else: 
            v1_thuc = v1

            # Doc van toc v3 (van toc banh 3)
        v3 = rs1.read('hr',50)
        if v3 > 32768:
            v3_thuc = v3 - 65536 
        else: 
            v3_thuc = v3
        
        # Doc van toc v2 (van toc banh 2)
        v2 = rs2.read('hr',40)
        if v2 > 32768:
            v2_thuc = v2 - 65536 
        else: 
            v2_thuc = v2

        # Doc van toc v4 (van toc banh 4)
        v4 = rs2.read('hr',50)
        if v4 > 32768:
            v4_thuc = v4 - 65536 
        else: 
            v4_thuc = v4

        ###
        V = math.sqrt(2)/4*(- v1_thuc - v2_thuc + v3_thuc + v4_thuc) #van toc x theo truc gan tren xe
        Vn =  math.sqrt(2)/4*(v1_thuc - v2_thuc - v3_thuc + v4_thuc) #van toc y theo truc gan tren xe
        omega = 1/(4*d)*(v1_thuc + v2_thuc + v3_thuc + v4_thuc) # van toc goc
        Vx = math.cos(alpha)*V - math.sin(alpha)*Vn #van toc x theo truc gan tren goc toa do
        Vy = math.sin(alpha)*V + math.cos(alpha)*Vn #van toc y theo truc gan tren goc toa do

        dt = (current_time - last_time).to_sec()
        delta_x = (Vx * cos(th) - Vy * sin(th)) * dt
        delta_y = (Vx * sin(th) + Vy * cos(th)) * dt
        delta_th = omega * dt

        x_pos_ += delta_x
        y_pos_ += delta_y
        th += delta_th # goc lech giua truc toa do cua moi truong va truc toa do cua xe, 0.02s la thoi gian lay mau)

        quat = tf.transformations.quaternion_from_euler(0, 0, th)
        odom_trans = TransformStamped()
        odom_trans.header.stamp = rospy.Time.now()
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_link"
        odom_trans.transform.translation.x = x_pos_
        odom_trans.transform.translation.y = y_pos_
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = quat
        br.sendTransform((0, 0, 0.0), quat, odom_trans.header.stamp,
                         odom_trans.child_frame_id, odom_trans.header.frame_id)

        msg1.header.frame_id = "odom"
        msg1.child_frame_id = "base_link"
        msg1.header.stamp = rospy.Time.now()
        msg1.twist.twist.linear.x = Vx
        msg1.twist.twist.linear.y = Vy
        msg1.twist.twist.linear.z = 0
        msg1.twist.twist.angular.x = 0
        msg1.twist.twist.angular.y = 0
        msg1.twist.twist.angular.z = omega
        pub.publish(msg1)
        last_time = current_time
        rate.sleep()
        
if __name__ == '__main__':
    rs1 = RS485(port=1)
    rs2 = RS485(port=2)
    myData = Twist()
    alpha = 0 # goc lech giua truc toa do cua moi truong va truc toa do cua xe 
    d = 21 # 21 cm
    listener()
    
    