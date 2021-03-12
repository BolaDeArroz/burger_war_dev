#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import roslib
import copy
import csv

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from burger_war_dev.msg import MyPose 
from geometry_msgs.msg import Pose,Point,Quaternion,Vector3

from image_function import Normalization, detect_field_trap
import rospy, math, tf
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import PolygonStamped, Point32, QuaternionStamped, Quaternion, TwistWithCovariance
from tf.transformations import quaternion_from_euler

class PubTrapPose():
    def __init__(self):
        # camera info
        self.horizontal_angle = 60

        self.bridge = CvBridge()
        # pub
        self.img_pub = rospy.Publisher("trap_view", Image, queue_size=1)
        self.trap_pose_pub=rospy.Publisher('trap_pose_from_camera',MyPose,queue_size=1)
        self.teb_obstacles_pub = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
        # sub
        self.image = None
        self.img_sub = rospy.Subscriber("/image_raw", Image, self.image_callback)
        self.my_pose = None
        self.my_pose_sub = rospy.Subscriber('/my_pose', MyPose, self.my_pose_callback)

        self.theta= 0
        self.ds= 0

    
    def my_pose_callback(self,data):
        self.my_pose = data
        # print('my_pose', self.my_pose)


    def image_callback(self, data):
        try:
            self.image = data
        except CvBridgeError as e:
            print("image_callback error: ", e)


    def calc_trap_pose_local_coordinate(self, size, width, x_center):
        # r = size / 2

        # area = r * r * math.pi
        # だいたい長方形なので
        area = size * 0.25*size  / 10

        ds = (-1.1492 * area + 800 + 100) / 1000
        # ds = -0.000137 * area + 0.18286
        x_center_pixel = self.x_trans_image_coordinate(x_center, width)
        radian_pixel = float(self.horizontal_angle)/float(width)
        th = (x_center_pixel * radian_pixel) * math.pi/180
        self.theta = th
        self.ds = area
        
        print('x_center_pixel', x_center_pixel)
        print('radian_pixel', radian_pixel)
        print('th', th)
        print('ds', ds)
        print('area', area)
        
        x = ds * math.cos(th)
        y = ds * math.sin(th)

        return [x, y]

    def x_trans_image_coordinate(self, origin_c,  image_width):
        trans_result = origin_c - image_width/2
        return trans_result




    def calc_trap_pose(self):
        coordinate_list = []
        try:
            # image = Normalization(self.bridge.imgmsg_to_cv2(self.image, "bgr8"))
            image = self.bridge.imgmsg_to_cv2(self.image, "bgr8")
            trap_contours = detect_field_trap(image)
            
            h, w, _ = image.shape
            if trap_contours != []:
              # print('trap_contours', trap_contours)
              c, r = cv2.minEnclosingCircle(trap_contours[0])

              # array.data = [1, c[0] - (w / 2.0), 2 * r, w, h]
              coordinate_list = self.calc_trap_pose_local_coordinate(r, w, c[0])
              
              print('radias :', r)
              print('center', c)
              print('trap: x, y', coordinate_list)
              
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        except Exception as e:
            print('calc trap pose error', e)
        
        return coordinate_list

        

    def run(self):
        r=rospy.Rate(3)
        """
        path = '/home/ctu/rhc_branch_ws/src/burger_war/position_logs.csv'
        with open(path, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'z', 'th', 'ds'])
        """
        while not rospy.is_shutdown():
            coordinate_list = self.calc_trap_pose()
            try:
                # print('my_pose',  self.my_pose.pos)
                """
                with open(path, 'a') as f:
                    writer = csv.writer(f)
                    writer.writerow([self.my_pose.pos.x, self.my_pose.pos.y, self.my_pose.pos.z, self.theta, self.ds])
                """
                if coordinate_list != []:
                    # 
                    try:
                        _pose_x = self.my_pose.pos.x + coordinate_list[0]
                        _pose_y = self.my_pose.pos.y + coordinate_list[1]
                        print('self.my_pose.pos: {}, {}'.format(self.my_pose.pos.x, self.my_pose.pos.y))
                        print('_pose_: {}, {}'.format(_pose_x, _pose_y))
                    except Exception as e:
                        print('RUN error: ', e)
                    #
                    _pose = MyPose(pos           =Point(_pose_x , _pose_y, self.my_pose.pos.z),\
                           ori_quaternion=Quaternion( 0, 0, 0,0),\
                           ori_euler     =Vector3(            0,         0, 0)          )
                    """
                    print('enemy_pose',  enemy_pose.pos)
                    print('my_pose',  self.my_pose.pos)
                    """
                    self.trap_pose_pub.publish(_pose)
                    # set obstacles
                    # 座標系逆でなんかxがマイナスな気する
                    self.teb_obstacles_pub.publish(self.publish_obstacle_msg(_pose_y, -1*_pose_x))
            except Exception as e:
                print('run error: ', e)
            
            
            r.sleep()

    def publish_obstacle_msg(self, _pose_x, _pose_y):

      obstacle_msg = ObstacleArrayMsg() 
      obstacle_msg.header.stamp = rospy.Time.now()
      obstacle_msg.header.frame_id = "odom" # CHANGE HERE: odom/map
      
      # Add polygon obstacle
      obstacle_msg.obstacles.append(ObstacleMsg())
      obstacle_msg.obstacles[0].id = 99
      # 座標系がx, y逆な気する この順番で四角：ロボットの初期正面方向がx軸, 右側がy軸マイナス方向 0.3mは結構でかいロボット全部埋まるくらい
      v1 = Point32()
      v1.x = _pose_x
      v1.y = _pose_y
      v2 = Point32()
      v2.x = 0.2+_pose_x
      v2.y = _pose_y
      v3 = Point32()
      v3.x = 0.2+_pose_x
      v3.y = -0.2+_pose_y
      v4 = Point32()
      v4.x = _pose_x
      v4.y = -0.2+_pose_y
      obstacle_msg.obstacles[0].polygon.points = [v1, v2, v3, v4]

      return obstacle_msg



if __name__ == '__main__': 
  
  rospy.init_node("pub_trap_pose")
  pub_trap_pose = PubTrapPose()
  pub_trap_pose.run()
  rospy.spin()
  """
  try:
    publish_obstacle_msg()
  except rospy.ROSInterruptException:
    pass
  """

