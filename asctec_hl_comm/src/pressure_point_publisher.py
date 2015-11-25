#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped
from asctec_hl_comm.msg import mav_imu

def callback(data):
    #height = data.differential_height
    height = data.height
    
    rospy.loginfo(rospy.get_caller_id() + "Pressure: %f", height)
    pt = PointStamped()
    pt.header = data.header
    pt.point.z = height
    pub = rospy.Publisher('pressure_height_point', PointStamped, queue_size=1)
    pub.publish(pt)
    
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('pressure_point_publisher', anonymous=True)

    rospy.Subscriber("fcu/imu_custom", mav_imu, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
