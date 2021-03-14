#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import json
import rospy


from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String



class PubScore():
    def __init__(self):
        self.side = str(rospy.get_param("~side"))

        # markers name: order is decided by zyali's board
        self.markers_name_list = [
                    'Tomato_N', 'Omelette_N',
                    'Tomato_S', 'Omelette_S', 
                    'FriedShrimp_N','FriedShrimp_W', 
                    'FriedShrimp_E','FriedShrimp_S', 
                    'Pudding_N','OctopusWiener_N', 
                    'Pudding_S', 'OctopusWiener_S']
        robo_maker_list = ['RE_L', 'RE_R', 'RE_B',
                            'BL_L', 'BL_R', 'BL_B']
        # blue ならま
        if self.side == 'b':
            self.markers_name_list.reverse()
        
        self.markers_name_list = self.markers_name_list+robo_maker_list
        # sub
        self.war_state=None
        self.war_state_sub = rospy.Subscriber('/war_state', String, self.warstate_callback)
        # pub
        self.score_pub = rospy.Publisher("score", Int32MultiArray, queue_size=1)

    def warstate_callback(self, data):
        _array = Int32MultiArray()
        self.war_state = data

        result = self.check_marker(self.war_state)
        # blue なら値反転
        if self.side == 'b':
            result = map(lambda x: x *-1, result)

        _array.data = result
        self.score_pub.publish(_array)
        

    def check_marker(self, war_state):
        """
        
        """
        red_list = []
        blue_list = []
        targets_list = []
        result_list = []
        if war_state is None:
            return red_list
        json_war_state = json.loads(war_state.data)
        for key in json_war_state:
            if key == 'targets':
                targets_list = json_war_state[key]
                break
        for target in targets_list:
            for key in target:
                if key == 'player' and target[key] == 'r':
                    red_list.append(target['name'])
                elif key == 'player' and target[key] == 'b':
                    blue_list.append(target['name'])
        for index, name in enumerate(self.markers_name_list):
            if name in red_list:
                result_list.append(1)
            elif name in blue_list:
                result_list.append(-1)
            else:
                result_list.append(0)
        return result_list
        


if __name__ == "__main__":
    rospy.init_node("pub_score")
    pub_score = PubScore()
    rospy.spin()
