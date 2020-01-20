#!/usr/bin/env python
import rospy, math, random
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2
from laser_geometry import LaserProjection
from roslib import message
from sensor_msgs import point_cloud2 
def points(data):
    pub1=rospy.Publisher('point_x', Float32, queue_size=100)
    pub2=rospy.Publisher('point_y', Float32, queue_size=100)
    pub3=rospy.Publisher('point_z', Float32, queue_size=100)
    while True:
         x1=[]
         y1=[]
         
         z1=[]
         for p in point_cloud2.read_points(data,skip_nans=True,field_names=("x","y","z")):
             #print(p[0],p[1],p[2])
             pub1.publish(p[0])
             pub2.publish(p[1])
             pub3.publish(p[2])
             
         
    
        
def listener():
       rospy.init_node('points')
       sub=rospy.Subscriber('/Sensor/points', PointCloud2, points)
       rate= rospy.Rate(0.5)
       rospy.spin()
if __name__=='__main__':
   try:
      listener()
   except rospy.ROSInterruptException:
        pass
