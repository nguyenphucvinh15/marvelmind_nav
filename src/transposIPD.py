#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from marvelmind_nav.msg import hedge_pos

def main():
    rospy.init_node ('transpos',anonymous=True)
    pub = rospy.Publisher("/rtabmap/odom", Odometry, queue_size=10)
    rate = rospy.Rate (10)    
    while not rospy.is_shutdown():
        msg = rospy.wait_for_message("/hedge_pos", hedge_pos)
        msg1 = Odometry()
        msg1.header.stamp = rospy.Time.now()
        msg1.header.frame_id= "odom"
        msg1.pose.pose.position.x = msg.x_m
        msg1.pose.pose.position.y = msg.y_m
        msg1.pose.pose.position.z = msg.z_m
        pub.publish(msg1)
        print (msg1.pose.pose.position.x)
        

    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
