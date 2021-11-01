#!/usr/bin/env python

import numpy
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from turtlesim.msg import Pose
import tf
from aruco_msgs.msg import MarkerArray

ox = 0
oy = 0.4
px = 0
py = 0
K  = numpy.eye(2)
seq=0

def publish_tf(data):

    global px,py   
    global seq
    seq = seq+1
    cmsg = PointStamped()
    cmsg.point.x       = ox
    cmsg.point.y       = oy
    cmsg.header.seq    = seq
    cmsg.header.stamp  = rospy.Time.now()
    cmsg.header.frame_id = "marker_584"
    publish_center.publish(cmsg)

    kmsg = PointStamped()
    kmsg.point.x       = px
    kmsg.point.y       = py
    kmsg.header.seq    = seq
    kmsg.header.stamp  = rospy.Time.now()
    kmsg.header.frame_id = "marker_584"
    publish_robot.publish(kmsg)   

    for marker in data.markers:

	if marker.id == 584:
		px = marker.pose.pose.position.x 
		pz = 0 #marker.pose.pose.position.y 
		py = marker.pose.pose.position.z 
		rx = marker.pose.pose.orientation.x 
		ry = marker.pose.pose.orientation.y 
		rz = marker.pose.pose.orientation.z 
		w  = marker.pose.pose.orientation.w 

		#rospy.logwarn("%f,%f,%f" % (px,py,pz))

		parent = data.header.frame_id
		child  = "marker_%d" % marker.id

		time = rospy.Time.now()

		cast.sendTransform( (-px,py,pz),
		                    (rx,ry,rz,w),
		                    time,
		                    child,
		                    parent)

def update_state():
    
    global px,pz
    d     = -numpy.sqrt((ox-px)**2+(oy-py)**2)
    gphi  = -numpy.pi/2
    phi   = numpy.arctan2(-py,-px)
    error = phi - gphi
    error = numpy.arctan2(numpy.sin(error),numpy.cos(error))
    
    x = numpy.array([[d],
                     [error]])
    u = -numpy.matmul(K,x)
    rospy.logwarn(u)
    action = Twist()
    action.linear.x  = min(0.5,max(-0.5,u.item(0)))
    action.angular.z = min(1,max(-1,u.item(1)))

    #rospy.logwarn("publicando")

    actuator.publish(action)




def run():

    global cast, actuator, publish_center, publish_robot
    rospy.init_node('aruco_tf', anonymous=True)
    cast = tf.TransformBroadcaster()
    rospy.Subscriber("/aruco/markers", MarkerArray, publish_tf)
    actuator = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)
    publish_center   = rospy.Publisher("center", PointStamped, queue_size=1)
    publish_robot   = rospy.Publisher("robot", PointStamped, queue_size=1)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
	update_state()
	#rospy.logwarn("loop")
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
