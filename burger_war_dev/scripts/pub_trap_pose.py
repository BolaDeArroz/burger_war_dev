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

        ds = (-1.1492 * area + 800 + 200) / 1000 * 1.15
        # ds = -0.000137 * area + 0.18286
        x_center_pixel = self.x_trans_image_coordinate(x_center, width)
        radian_pixel = float(self.horizontal_angle)/float(width)
        th = (x_center_pixel * radian_pixel) * math.pi/180
        self.theta = th
        self.ds = area
        """
        print('x_center_pixel', x_center_pixel)
        print('radian_pixel', radian_pixel)
        print('th', th)
        print('ds', ds)
        print('area', area)
        """
        return [ds, th, area]

    def x_trans_image_coordinate(self, origin_c,  image_width):
        trans_result = origin_c - image_width/2
        return trans_result




    def calc_trap_pose(self):
        coordinate_list = []
        result_pose=[]
        try:
            # image = Normalization(self.bridge.imgmsg_to_cv2(self.image, "bgr8"))
            # gazebo
            image = self.bridge.imgmsg_to_cv2(self.image, "bgr8")
            
            trap_contours = detect_field_trap(image)
            
            h, w, _ = image.shape
            if trap_contours != []:
              # print('trap_contours', trap_contours)
              c, r = cv2.minEnclosingCircle(trap_contours[0])

              # array.data = [1, c[0] - (w / 2.0), 2 * r, w, h]
              # カメラ座標系からの距離と角度算出
              coordinate_list = self.calc_trap_pose_local_coordinate(r, w, c[0])
              # 本体のmap位置とカメラからの相対位置でtrapのpose推定
              ds = coordinate_list[0]
              map_th = 0
              # my_eulは  右方向が0,上がπ/2,左がπ, (or -π)下が-π/2
              if self.my_pose.ori_euler.z >= 0:
                  map_th = math.pi - self.my_pose.ori_euler.z + math.pi/2
              else:
                  map_th =  -1*self.my_pose.ori_euler.z + math.pi/2
              th = 1*coordinate_list[1] + map_th
              """
              print('self.my_pose.ori_euler.z', self.my_pose.ori_euler.z)
              print('map_th', map_th)
              print('map th', th)
              """
              # 目の前
              add_cam_x = 1*(ds * math.cos(th))
              add_cam_y = 1*(ds * math.sin(th))
              if coordinate_list[2] > 750:
                  add_cam_x = 0.06 * math.cos(th)
                  add_cam_y = 0.06 * math.sin(th)
              
              if th > math.pi/2 and th < math.pi*3/2:
                  add_cam_y = -1*add_cam_y
                  add_cam_x = -1*add_cam_x  
              
              # 横幅
              x = -1*add_cam_x + self.my_pose.pos.y -0.0
              # 奥行き
              y = add_cam_y + -1*self.my_pose.pos.x +0.0
              result_pose = [x, y]
              """
              print('add_cam_', add_cam_x, add_cam_y)
              print('radias :', r)
              print('center', c)
              print('robot: x, y', self.my_pose.pos.x, self.my_pose.pos.y)
              print('trap: x, y', result_pose)
              """
              
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        except Exception as e:
            print('calc trap pose error', e)
        
        return result_pose

        

    def run(self):
        r=rospy.Rate(10)
        
        moving_times = 5
        x_moving_ave_list = [0]*moving_times
        y_moving_ave_list = [0]*moving_times
        moving_ave_count = 0
        x_ave = y_ave = 0

        while not rospy.is_shutdown():
            
            coordinate_list = self.calc_trap_pose()
            try:
                """
                _x = self.my_pose.pos.y -0.0
                _y = -1*self.my_pose.pos.x +0.0
                self.teb_obstacles_pub.publish(self.publish_obstacle_msg(_x, _y))
                coordinate_list = []
                # print('my_pose',  self.my_pose.pos)
                """
                if coordinate_list != []:
                    
                    x_moving_ave_list[moving_ave_count] = coordinate_list[0]
                    y_moving_ave_list[moving_ave_count] = coordinate_list[1]
                    moving_ave_count = moving_ave_count+1
                    if moving_ave_count >= moving_times:
                        moving_ave_count = 0
                    
                    # 
                    try:
                        _pose_x =  sum(x_moving_ave_list) / len(x_moving_ave_list)
                        _pose_y = sum(y_moving_ave_list) / len(y_moving_ave_list)
                    except Exception as e:
                        print('RUN error: ', e)
                    #
                    _pose = MyPose(pos           =Point(_pose_x , _pose_y, self.my_pose.pos.z),\
                           ori_quaternion=Quaternion( 0, 0, 0,0),\
                           ori_euler     =Vector3(            0,         0, 0)          )
                    
                    self.trap_pose_pub.publish(_pose)
                    # set obstacles
                    self.teb_obstacles_pub.publish(self.publish_obstacle_msg(_pose_x, _pose_y))
            except Exception as e:
                print('run error: ', e)
            
            
            r.sleep()

    def publish_obstacle_msg(self, _pose_x, _pose_y):

      obstacle_msg = ObstacleArrayMsg() 
      obstacle_msg.header.stamp = rospy.Time.now()
      obstacle_msg.header.frame_id = "map" # CHANGE HERE: odom/map
      block_dis = 0.05
      # Add polygon obstacle
      obstacle_msg.obstacles.append(ObstacleMsg())
      obstacle_msg.obstacles[0].id = 99
      # 座標系がx, y逆な気する この順番で四角：ロボットの初期正面方向がx軸, 右側がy軸マイナス方向 0.3mは結構でかいロボット全部埋まるくらい
      v1 = Point32()
      v1.x = block_dis+_pose_x
      v1.y = block_dis+_pose_y
      v2 = Point32()
      v2.x = block_dis+_pose_x
      v2.y = -1*block_dis+_pose_y
      v3 = Point32()
      v3.x = -1*block_dis+_pose_x
      v3.y = -1*block_dis+_pose_y
      v4 = Point32()
      v4.x = -1*block_dis+_pose_x
      v4.y = block_dis+_pose_y
      obstacle_msg.obstacles[0].polygon.points = [v1, v2, v3, v4]

      return obstacle_msg



if __name__ == '__main__': 
  
  rospy.init_node("pub_trap_pose")
  pub_trap_pose = PubTrapPose()
  pub_trap_pose.run()
  rospy.spin()


