#!/usr/bin/env python
import roslib; roslib.load_manifest('robot_control')
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

vel_msg = Twist()

def callback(twist):
    global vel_msg
    vel_msg = twist
    # print(rospy.get_name() + ": %s", Twist)
    # print("\n")

def listener():
    rospy.init_node("listener_lidar")
    rospy.Subscriber("/two_wheel_robot/laser/scan", LaserScan, callback)
    rospy.spin()

def talker():
    global vel_msg
    pub = rospy.Publisher("/cmd_vel", Twist)
    rospy.init_node("Control_robot")
    
    while not rospy.is_shutdown():
        pub.publish(vel_msg)
        rospy.sleep(0.02)

if __name__ == "__main__":
    listener()
    try:
        talker()
    except rospy.ROSInterruptException: pass