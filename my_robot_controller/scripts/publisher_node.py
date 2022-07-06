#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


def talk_to_me():
    rospy.init_node('publisher_node')
    rospy.loginfo("Node has been started.")
    msg = Odometry()
    pub = rospy.Publisher("/odom/kinematic_raw", Odometry, queue_size=1)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():

        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        odom_trans = TransformStamped()
        odom_trans.header.stamp = rospy.Time.now()
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_link"
        odom_trans.transform.translation.x = 0
        odom_trans.transform.translation.y = 0
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = quat
        br.sendTransform((0, 0, 0.0), quat, odom_trans.header.stamp,
                         odom_trans.child_frame_id, odom_trans.header.frame_id)

        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.header.stamp = rospy.Time.now()
        msg.twist.twist.linear.x = 0
        msg.twist.twist.linear.y = 0
        msg.twist.twist.linear.z = 0
        msg.twist.twist.angular.x = 0
        msg.twist.twist.angular.y = 0
        msg.twist.twist.angular.z = 0
        pub.publish(msg)
        rate.sleep()
if __name__ == '__main__':
    try:
        talk_to_me()
    except rospy.ROSInterruptException:
        pass
