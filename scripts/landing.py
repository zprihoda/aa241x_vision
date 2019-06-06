#!/usr/bin/env python

"""
Landing Command Script for Autonomous Mission
"""

import rospy
import math

# Import message types
from aa241x_vision.msg import tag_info, Targetpoint

formation = dict()
formation[(0, 1)] = 0.25
formation[(0, 2)] = 0.25
formation[(0, 3)] = 0.25
formation[(1, 2)] = 0.25 * math.sqrt(2)
formation[(1, 3)] = 0.25 * math.sqrt(2)
formation[(2, 3)] = 0.25 * 2
formation[(0, 9)] = 0.345
formation[(1, 9)] = 0.345 + 0.25
formation[(2, 9)] = math.sqrt(0.345 ** 2 + 0.25 ** 2)
formation[(3, 9)] = math.sqrt(0.345 ** 2 + 0.25 ** 2)
formation[(0, 5)] = 1.082
formation[(1, 5)] = 1.082 + 0.25
formation[(2, 5)] = math.sqrt(1.082 ** 2 + 0.25 ** 2)
formation[(3, 5)] = math.sqrt(1.082 ** 2 + 0.25 ** 2)
formation[(9, 5)] = 1.082 - 0.345
center_id = 0


class Landing():
    def __init__(self):
        rospy.init_node('landing_node', anonymous=True)

        # class variables
        self.tag_id = []
        self.tag_position = []
        self.tag_rotation = []

        self.target_id = -1
        self.target_num = 0
        self.target_x = 0
        self.target_y = 0
        self.target_z = 0
        self.target_yaw = 0

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

            if abs(_tag_pitch) < 0.16 and abs(_tag_roll) < 0.16:
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
                        _distance_ij = math.sqrt((_position_i[0] - _position_j[0]) ** 2 + (_position_i[1] - _position_j[1]) ** 2 + (_position_i[2] - _position_j[2]) ** 2)
                        if abs(_distance_ij - formation[_key]) > 0.10:
                            formation_match = False

            if len(self.tag_id) == 1:
                formation_match = True

            if formation_match:
                self.target_num = len(self.tag_id)
                final_x = 0
                final_y = 0
                final_z = 0
                final_yaw = 0
                for i in range(len(self.tag_id)):
                    _id = self.tag_id[i]
                    _position = self.tag_position[_id]
                    _rotation = self.tag_rotation[_id]

                    _tag_yaw = _rotation[0]
                    if _id == center_id:
                        _center_x = _position[0]
                        _center_y = _position[1]

                    elif _id == 1:
                        _center_x = _position[0] - 0.25 * math.sin(_tag_yaw)
                        _center_y = _position[1] + 0.25 * math.cos(_tag_yaw)

                    elif _id == 2:
                        _center_x = _position[0] - 0.25 * math.cos(_tag_yaw)
                        _center_y = _position[1] - 0.25 * math.sin(_tag_yaw)

                    elif _id == 3:
                        _center_x = _position[0] + 0.25 * math.cos(_tag_yaw)
                        _center_y = _position[1] + 0.25 * math.sin(_tag_yaw)

                    elif _id == 9:
                        _center_x = _position[0] + 0.345 * math.sin(_tag_yaw)
                        _center_y = _position[1] - 0.345 * math.cos(_tag_yaw)

                    elif _id == 5:
                        _center_x = _position[0] + 1.082 * math.sin(_tag_yaw)
                        _center_y = _position[1] - 1.082 * math.cos(_tag_yaw)

                    _center_z = _position[2]

                    final_x = final_x + _center_x
                    final_y = final_y + _center_y
                    final_z = final_z + _center_z
                    final_yaw = final_yaw + _tag_yaw

                final_x = final_x / len(self.tag_id)
                final_y = final_y / len(self.tag_id)
                final_z = final_z / len(self.tag_id)
                final_yaw = final_yaw / len(self.tag_id)

                self.target_x = final_x
                self.target_y = final_y
                self.target_z = final_z
                self.target_yaw = final_yaw


    ## Process Functions
    def publish(self):
        """ publish target point for landing """
        msg = Targetpoint()

        if self.target_num == 0:
            msg.num = 0
            msg.x = 0
            msg.y = 0
            msg.z = 0
            msg.yaw = 0
        else:
            msg.num = self.target_num
            msg.x = - self.target_y
            msg.y = - self.target_x
            msg.z = self.target_z
            msg.yaw = - self.target_yaw - math.pi / 2

        self.target_num = 0

        self.target_pub.publish(msg)

    def run(self):
        rate = rospy.Rate(2)  # 10 Hz
        while not rospy.is_shutdown():
            self.landcommand()
            self.publish()
            rate.sleep()


if __name__ == '__main__':
    landing = Landing()
    landing.run()
