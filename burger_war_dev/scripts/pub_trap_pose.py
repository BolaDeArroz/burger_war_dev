#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, math, tf
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import PolygonStamped, Point32, QuaternionStamped, Quaternion, TwistWithCovariance
from tf.transformations import quaternion_from_euler


def publish_obstacle_msg():
  pub = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
  #pub = rospy.Publisher('/p3dx/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
  rospy.init_node("test_obstacle_msg")

  y_0 = -3.0
  vel_x = 0.0
  vel_y = 0.3
  range_y = 6.0

  obstacle_msg = ObstacleArrayMsg() 
  obstacle_msg.header.stamp = rospy.Time.now()
  obstacle_msg.header.frame_id = "map" # CHANGE HERE: odom/map
  
  # Add polygon obstacle
  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles[0].id = 99
  # 座標系がx, y逆な気する この順番で四角：ロボットの初期正面方向がx軸, 右側がy軸マイナス方向 0.3mは結構でかいロボット全部埋まるくらい
  v1 = Point32()
  v1.x = -0.1
  v1.y = -0.5
  v2 = Point32()
  v2.x = 0.1
  v2.y = -0.5
  v3 = Point32()
  v3.x = 0.2
  v3.y = -0.7
  v4 = Point32()
  v4.x = -0.2
  v4.y = -0.7
  obstacle_msg.obstacles[0].polygon.points = [v1, v2, v3, v4]

  r = rospy.Rate(10) # 10hz
  t = 1.0
  while not rospy.is_shutdown():
    
    # Vary y component of the point obstacle
    """
    if (vel_y >= 0):
      obstacle_msg.obstacles[0].polygon.points[0].y = y_0 + (vel_y*t)%range_y
    else:
      obstacle_msg.obstacles[0].polygon.points[0].y = y_0 + (vel_y*t)%range_y - range_y
    """

    # t = t + 0.1
    
    # print('obstacle_msg *****************', obstacle_msg)
    pub.publish(obstacle_msg)
    
    r.sleep()



if __name__ == '__main__': 
  try:
    publish_obstacle_msg()
  except rospy.ROSInterruptException:
    pass

