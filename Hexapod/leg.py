# -*- coding: utf-8 -*-
from math_utils import *
from Inversekinematics import *
#from Hexapod.servoControl import *

half_pi = pi / 2
MOUNT_POINT_X = RADIUS_LENGTH * sin(60 * pi / 180)
MOUNT_POINT_Y = RADIUS_LENGTH * cos(60 * pi / 180)


class BaseLeg(object):
    def __init__(self, leg_index):
        self._leg_index = leg_index
        self._leg_tag = -1
        self._tip_default_world = point3d(0, 0, 0)
        """ 
        local_conv: 世界坐标系上的表达转换到本地坐标系上的表达
        world_conv: 本地坐标系上的表达转换到世界坐标系上的表达
        """
        if self._leg_index == 'RF':  # 45 or -315 degree:
            self._rotate_angle = 60.0
            self._leg_tag = 5
            self._mount_position = point3d(MOUNT_POINT_X, MOUNT_POINT_Y, 0)
        elif self._leg_index == 'RM':  # 0 degree:
            self._rotate_angle = 0.0
            self._leg_tag = 4
            self._mount_position = point3d(0, RADIUS_LENGTH, 0)
        elif self._leg_index == 'RH':  # -45 or 315 degree:
            self._rotate_angle = 300.0
            self._leg_tag = 3
            self._mount_position = point3d(-MOUNT_POINT_X, MOUNT_POINT_Y, 0)
        elif self._leg_index == 'LF':  # -135 or 225 degree:
            self._rotate_angle = 120.0
            self._leg_tag = 0
            self._mount_position = point3d(MOUNT_POINT_X, -MOUNT_POINT_Y, 0)
        elif self._leg_index == 'LM':  # -180 or 180 degree:
            self._rotate_angle = 180.0
            self._leg_tag = 1
            self._mount_position = point3d(0, -RADIUS_LENGTH, 0)
        elif self._leg_index == 'LH':  # -225 or 135 degree:
            self._rotate_angle = 240.0
            self._leg_tag = 2
            self._mount_position = point3d(-MOUNT_POINT_X, -MOUNT_POINT_Y, 0)
        else:
            raise ValueError

    def get_leg_index(self):
        return self._leg_index

    def get_rotate_angle(self):
        return self._rotate_angle

    def get_leg_tag(self):
        return self._leg_tag

    def get_mount_point(self):
        return self._mount_position

    def _local_conv(self, point: point3d):
        return rotate(point, self._rotate_angle)

    def _world_conv(self, point: point3d):
        return rotate(point, 360.0 - self._rotate_angle)

    """coordinate system translation"""
    def translate2local(self, world_point: point3d):
        return self._local_conv(world_point - self._mount_position)

    def translate2world(self, local_point: point3d):
        return self._world_conv(local_point) + self._mount_position

    def set_default_point(self, point: point3d):
        self._tip_default_world = point

    def translate_tip_2_local(self, tip_point: point3d):
        world_point = point3d(0.0, 0.0, 0.0)
        world_point.x = self._tip_default_world.x + tip_point.x
        world_point.y = self._tip_default_world.y + tip_point.y
        world_point.z = self._tip_default_world.z - tip_point.z
        return self.translate2local(world_point)

    @staticmethod
    def inverse_kinematics(target_point: point3d):
        return Inversekinematics(target_point)

    @staticmethod
    def forward_kinematics(theta):
        return kinematics(theta)

class Leg(BaseLeg):
    def __init__(self, leg_index, default_angle_pos=None, devia=45.0):
        super().__init__(leg_index)
        self._leg_servo_id = self.get_servo_id(leg_index)
        self._tip_pos_local = point3d(0, 0, 0)
        self._tip_pos = point3d(0, 0, 0)
        self._angle = [0.0] * 3
        self.devia = devia

        if default_angle_pos is not None:
            self.set_default_point(self.translate2world(self.forward_kinematics(default_angle_pos)))

    def get_tip_data(self):
        return [self._angle, self._tip_pos, self._tip_pos_local]

    def set_tip_pos_tip(self, point: point3d):
        self._tip_pos_local = self.translate_tip_2_local(point)
        self._tip_pos = self.translate2world(self._tip_pos_local)
        self._angle = self.inverse_kinematics(self._tip_pos_local)

    def set_tip_pos_local(self, point: point3d):
        self._tip_pos_local = point
        self._tip_pos = self.translate2world(point)
        self._angle = self.inverse_kinematics(self._tip_pos_local)

    def set_tip_pos(self, point: point3d):
        self._tip_pos = point
        self._tip_pos_local = self.translate2local(self._tip_pos)

        self._angle = self.inverse_kinematics(self._tip_pos_local)

    def set_tip_pos_theta(self, theta):
        try:
            tip_pos_local = self.forward_kinematics(theta)
            tip_pos = self.translate2world(tip_pos_local)
            self._tip_pos = tip_pos
            self._tip_pos_local = tip_pos_local
        except:
            pass
        self._angle = theta

    def get_leg_index(self):
        return self._leg_index

    @staticmethod
    def get_servo_id(leg_index): #TODO:get leg servo id
        leg_servo_id = []
        if leg_index == 'LF':
            leg_servo_id = [1, 2, 3]
        elif leg_index == 'LM':
            leg_servo_id = [4, 5, 6]
        elif leg_index == 'LH':
            leg_servo_id = [10, 11, 12]
        elif leg_index == 'RF':
            leg_servo_id = [16, 17, 18]
        elif leg_index == 'RM':
            leg_servo_id = [7, 8, 9]
        elif leg_index == 'RH':
            leg_servo_id = [13, 14, 15]
        else:
            raise ValueError
        return leg_servo_id

    def get_servo_move_array(self, set_devia=True):
        ang = [[self._leg_servo_id[i], self._angle[i]] for i in (0, 1, 2)]
        if set_devia:
            ang[2][1] = ang[2][1] - self.devia
        return ang

class ShortLeg:
    def __init__(self, leg_index, devia=45.0):
        self._leg_index = leg_index
        self._leg_servo_id = self.get_servo_id(leg_index)
        self._angle = [0.0] * 2
        self.devia = devia

    def get_leg_index(self):
        return self._leg_index

    def set_pos(self, angle):
        self._angle = angle

    def get_angle(self):
        return self._angle

    @staticmethod
    def get_servo_id(leg_index):  # TODO:get leg servo id
        leg_servo_id = []
        if leg_index == 'ULF':
            leg_servo_id = [25, 26]
        elif leg_index == 'ULM':
            leg_servo_id = [27, 28]
        elif leg_index == 'ULH':
            leg_servo_id = [23, 24]
        elif leg_index == 'URF':
            leg_servo_id = [29, 30]
        elif leg_index == 'URM':
            leg_servo_id = [21, 22]
        elif leg_index == 'URH':
            leg_servo_id = [19, 20]
        else:
            raise ValueError
        return leg_servo_id

    def get_servo_move_array(self):
        ang = [[self._leg_servo_id[i], self._angle[i]] for i in (0, 1)]
        return ang

class VirtualLeg(Leg):
    def __init__(self, leg_index, ax=None):
        super().__init__(leg_index)
        self._tip_pos_local = point3d(0, 0, 0)
        self._tip_pos = point3d(0, 0, 0)
        self._angle = [0.0] * 3
        self.ax = ax

    def draw_leg_2d(self):
        if self.ax is not None:
            radPerDegree = pi / 180.0
            angle = [ang * radPerDegree for ang in self._angle]

            leg_root = [0.0, 0.0]
            joint1 = [COXA_LENGTH, 0.0]
            joint2 = [COXA_LENGTH + FEMUR_LENGTH * cos(abs(angle[1])), FEMUR_LENGTH * sin(angle[1])]
            joint3 = [self._tip_pos_local.y, self._tip_pos_local.z]
            coxa_vector = [[leg_root[0], joint1[0]], [-leg_root[1], -joint1[1]]]
            femur_vector = [[joint1[0], joint2[0]], [-joint1[1], -joint2[1]]]
            tibia_vector = [[joint2[0], joint3[0]], [-joint2[1], -joint3[1]]]
            # x, y = coxa_vector
            # print('coxa_length:'+str(sqrt((x[0]-x[1])**2+(y[0]-y[1])**2)) + '\n')
            # x, y = femur_vector
            # print('femur_length:' + str(sqrt((x[0]-x[1])**2+(y[0]-y[1])**2)) + '\n')
            # x, y = tibia_vector
            # print('tibia_length:' + str(sqrt((x[0]-x[1])**2+(y[0]-y[1])**2)) + '\n')
            self.ax.plot(coxa_vector[0], coxa_vector[1], linewidth=3, color='red')
            self.ax.plot(femur_vector[0], femur_vector[1], linewidth=3, color='blue')
            self.ax.plot(tibia_vector[0], tibia_vector[1], linewidth=3, color='black')
        else:
            raise ValueError

    def draw_leg_3d(self):
        if self.ax is not None:
            radPerDegree = pi / 180.0
            angle = [ang * radPerDegree for ang in self._angle]
            leg_root = self.get_mount_point()
            joint1 = self.translate2world(point3d(COXA_LENGTH * sin(angle[0]), COXA_LENGTH * cos(angle[0]), 0.0))
            joint2 = self.translate2world(point3d((COXA_LENGTH + FEMUR_LENGTH * cos(abs(angle[1]))) * sin(angle[0]),
                                                  (COXA_LENGTH + FEMUR_LENGTH * cos(abs(angle[1]))) * cos(angle[0]),
                                                  FEMUR_LENGTH * sin(angle[1])))
            joint3 = self._tip_pos
            coxa_vector = [[leg_root.x, joint1.x], [leg_root.y, joint1.y], [-leg_root.z, -joint1.z]]
            femur_vector = [[joint1.x, joint2.x], [joint1.y, joint2.y], [-joint1.z, -joint2.z]]
            tibia_vector = [[joint2.x, joint3.x], [joint2.y, joint3.y], [-joint2.z, -joint3.z]]
            self.ax.plot(coxa_vector[0], coxa_vector[1], coxa_vector[2], linewidth=3, color='red')
            self.ax.plot(femur_vector[0], femur_vector[1], femur_vector[2], linewidth=3, color='blue')
            self.ax.plot(tibia_vector[0], tibia_vector[1], tibia_vector[2], linewidth=3, color='black')
        else:
            raise ValueError








