#!/usr/bin/env python

"""
Landing Command Script for Autonomous Mission
"""

import rospy
import math

# Import message types
from aa241x_vision.msg import tag_info, Targetpoint

formation = dict()
formation[(0, 1)] = 0.15
formation[(0, 2)] = 0.15
formation[(0, 3)] = 0.15
formation[(1, 2)] = 0.15 * math.sqrt(2)
formation[(1, 3)] = 0.15 * math.sqrt(2)
formation[(2, 3)] = 0.15 * 2
formation[(0, 17)] = 0.2
formation[(1, 17)] = 0.2 + 0.15
formation[(2, 17)] = 0.25
formation[(3, 17)] = 0.25
center_id = 0

class Landing():
    def __init__(self):
        rospy.init_node('landing_node', anonymous=True)

        # class variables
        self.tag_id = []
        self.tag_position = []
        self.tag_rotation = []
        
        self.target_id = None
        self.target_x = None
        self.target_y = None
        self.target_z = None
        self.target_yaw = None

        # publishers
        self.target_pub = rospy.Publisher("/landing/target_point", Targetpoint, queue_size=10)

        # subscribers
        rospy.Subscriber('/tag_information', tag_info, self.taginfoCallback)

    ## Callbacks
    def taginfoCallback(self, msg):
        self.tag_id = []        
        self.tag_position = dict()
        self.tag_rotation = dict()

        for i in range(len(msg.id)):
            _tag_id = msg.id[i]
            _tag_position = msg.position[i * 3: (i + 1) * 3]
            _tag_rotation = msg.rotation[i * 9: (i + 1) * 9]

            _tag_yaw = math.atan2(_tag_rotation[3], _tag_rotation[0])
            _tag_pitch = math.atan2(-_tag_rotation[6], math.sqrt(_tag_rotation[7] ** 2 + _tag_rotation[8] ** 2))
            _tag_roll = math.atan2(_tag_rotation[7], _tag_rotation[8])
            
            if abs(_tag_pitch) < 0.08 and abs(_tag_roll) < 0.08:
                self.tag_id.append(_tag_id)
                self.tag_position[_tag_id] = _tag_position
                self.tag_rotation[_tag_id] = [_tag_yaw, _tag_pitch, _tag_roll]
        
        self.tag_id.sort()

    ## Main Loop for Navigator
    def landcommand(self):
        if len(self.tag_id) != 0:
            if len(self.tag_id) > 1:
                formation_match = True
                for i in range(len(self.tag_id) - 1):
                    for j in range(i + 1, len(self.tag_id)):
                        _key = (self.tag_id[i], self.tag_id[j])
                        _position_i = self.tag_position[self.tag_id[i]]
                        _position_j = self.tag_position[self.tag_id[j]]
                        _distance_ij = math.sqrt((_position_i[0] - _position_j[0])**2 + (_position_i[1] - _position_j[1])**2 + (_position_i[2] - _position_j[2])**2)
                        if abs(_distance_ij - formation[_key]) > 0.05:
                            formation_match = False

                if formation_match:
                    if center_id in self.tag_id:
                        _selected_id = center_id
                        _position = self.tag_position[center_id]
                        _rotation = self.tag_rotation[center_id]

                    else:
                        _selected_id = self.tag_id[0]
                        _position = self.tag_position[self.tag_id[0]]
                        _rotation = self.tag_rotation[self.tag_id[0]]

            if len(self.tag_id) == 1:
                formation_match = True
                _selected_id = self.tag_id[0]
                _position = self.tag_position[self.tag_id[0]]
                _rotation = self.tag_rotation[self.tag_id[0]]

            
            if formation_match:
                self.target_id = _selected_id
                tag_yaw = _rotation[0]
                self.target_yaw = tag_yaw
                self.target_z = _position[2]
                if _selected_id == center_id:
                    self.target_x = _position[0]
                    self.target_y = _position[1]
                elif _selected_id == 1:
                    self.target_x = _position[0] - 0.25 * math.sin(tag_yaw)
                    self.target_y = _position[1] + 0.25 * math.cos(tag_yaw)
                    print("tag_yaw", tag_yaw)
                elif _selected_id == 2:
                    self.target_x = _position[0] - 0.25 * math.cos(tag_yaw)
                    self.target_y = _position[1] - 0.25 * math.sin(tag_yaw)
                elif _selected_id == 3:
                    self.target_x = _position[0] + 0.25 * math.cos(tag_yaw)
                    self.target_y = _position[1] + 0.25 * math.sin(tag_yaw)
                elif _selected_id == 17:
                    self.target_x = _position[0] + 0.54 * math.sin(tag_yaw)
                    self.target_y = _position[1] - 0.54 * math.cos(tag_yaw)


    ## Process Functions
    def publish(self):
        """ publish target point for landing """
        msg = Targetpoint()
        
        if self.target_x:
            msg.id = self.target_id
            msg.x = - self.target_y
            msg.y = - self.target_x
            msg.z = self.target_z
            msg.yaw = - self.target_yaw - math.pi/2

        self.target_pub.publish(msg)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.landcommand()
            self.publish()
            rate.sleep()


if __name__ == '__main__':
    landing = Landing()
landing.run()
