from servoControl import *
from leg import *
from path import *

leg_index = ['LF', 'LM', 'LH', 'RH', 'RM', 'RF']
upper_leg_index = ['ULF', 'ULM', 'ULH', 'URH', 'URM', 'URF']
DEFAULT_POS = [0, -45.0, 120.0]
DEFAULT_POS_1 = [0, 15.0, 70.0]
DEFAULT_POS_2 = [0, 50.0, 10.0]
DEFAULT_POS_3 = [0, 15.0, 65.0]


class VirtualHexapod:
    def __init__(self, ax):
        self.leg = [VirtualLeg(index, ax) for index in leg_index]
        self.body = [
            [[MOUNT_POINT_X, 0], [MOUNT_POINT_Y, RADIUS_LENGTH], [0.0] * 2],
            [[0, -MOUNT_POINT_X], [RADIUS_LENGTH, MOUNT_POINT_Y], [0.0] * 2],
            [[MOUNT_POINT_X, 0], [-MOUNT_POINT_Y, -RADIUS_LENGTH], [0.0] * 2],
            [[0, -MOUNT_POINT_X], [-RADIUS_LENGTH, -MOUNT_POINT_Y], [0.0] * 2],
            [[-MOUNT_POINT_X, -MOUNT_POINT_X], [MOUNT_POINT_Y, -MOUNT_POINT_Y], [0.0] * 2],
            [[MOUNT_POINT_X, MOUNT_POINT_X], [MOUNT_POINT_Y, -MOUNT_POINT_Y], [0.0] * 2]
        ]
        self.ax = ax

        for leg in self.leg:
            leg.set_tip_pos_theta(DEFAULT_POS_3)
            leg.set_default_point(leg.get_tip_data()[1])
        default_leg_len = self.leg[0].get_tip_data()[2].y
        self.turn_path_left = gen_turn_path_left_table(5, default_leg_len, div_num=20)
        self.turn_path_right = gen_turn_path_right_table(5, default_leg_len, div_num=20)
        self.turn_path_len = self.turn_path_right['len']

        self.tripot_gait_path_forward = gen_forward_Ellipse_path_table(20, 0.5, div_num=6)
        self.tripot_gait_path_backward = gen_forward_path_table(20, div_num=6)
        self.tripot_gait_len = self.tripot_gait_path_forward['len']


        default_world_tip = [leg.get_tip_data()[1] for leg in self.leg]
        self.rotate_path_left = gen_rotate_left_table(10, default_world_tip)
        self.rotate_path_right = gen_rotate_right_table(10, default_world_tip)
        self.rotate_len = self.rotate_path_left['len']

        # self._gen_c_path_table(gen_wave_Ellipse_path_table(20), 500)
        # self._gen_c_path_table(gen_tetrapod_Ellipse_path_table(20), 500)
        # self._gen_c_path_table(gen_ripple_Ellipse_path_table(20), 500)
        self.wave = gen_wave_Ellipse_path_table(20)
        self.wave_len = self.wave['len']

        self.tetra = gen_tetrapod_Ellipse_path_table(20)
        self.tetra_len = self.tetra['len']

        self.ripple = gen_ripple_Ellipse_path_table(20)
        self.ripple_len = self.ripple['len']

        self.move_left = gen_leftward_path_table(10, div_num=20)
        self.move_left_len = self.move_left['len']
        self.move_mode = -1

        self.twist = gen_twist_path_table(default_world_tip)
        self.twist_len = self.twist['len']

        self.test = gen_rotate_left_table(5, default_world_tip)
        self.test_len = self.test['len']

    def action(self, action_table):
        len = action_table['len']
        if self.move_mode == -1 or self.move_mode >= len - 1:
            self.move_mode = 0
        for leg in self.leg:
            leg.set_tip_pos_tip(action_table[leg.get_leg_index()][self.move_mode])
        self.move_mode += 1

    def wave_(self):
        if self.move_mode == -1 or self.move_mode >= self.wave_len - 1:
            self.move_mode = 0
        for leg in self.leg:
            leg.set_tip_pos_tip(self.wave[leg.get_leg_index()][self.move_mode])
        self.move_mode += 1

    def tetra_(self):
        if self.move_mode == -1 or self.move_mode >= self.tetra_len - 1:
            self.move_mode = 0
        for leg in self.leg:
            leg.set_tip_pos_tip(self.tetra[leg.get_leg_index()][self.move_mode])
        self.move_mode += 1

    def ripple_(self):
        if self.move_mode == -1 or self.move_mode >= self.ripple_len - 1:
            self.move_mode = 0
        for leg in self.leg:
            leg.set_tip_pos_tip(self.ripple[leg.get_leg_index()][self.move_mode])
        self.move_mode += 1


    def twist_(self):
        if self.move_mode == -1 or self.move_mode >= self.twist_len - 1:
            self.move_mode = 0
        for leg in self.leg:
            leg.set_tip_pos(self.twist[leg.get_leg_index()][self.move_mode])
        self.move_mode += 1


    def move_left_(self):
        if self.move_mode == -1 or self.move_mode >= self.move_left_len - 1:
            self.move_mode = 0
        for leg in self.leg:
            leg.set_tip_pos_tip(self.move_left[leg.get_leg_index()][self.move_mode])
        self.move_mode += 1

    def turn_body(self, direct='left'):
        if self.move_mode == -1 or self.move_mode >= self.turn_path_len - 1:
            self.move_mode = 0
        for leg in self.leg:
            if direct == 'left':
                leg.set_tip_pos_tip(self.turn_path_left[leg.get_leg_index()][self.move_mode])
            else:
                leg.set_tip_pos_tip(self.turn_path_right[leg.get_leg_index()][self.move_mode])
        self.move_mode += 1

    def rotate_body(self, direct='right'):
        if self.move_mode == -1 or self.move_mode > self.rotate_len - 1:
            self.move_mode = 0
        for leg in self.leg:
            if direct == 'left':
                leg.set_tip_pos(self.rotate_path_left[leg.get_leg_index()][self.move_mode])
            else:
                leg.set_tip_pos(self.rotate_path_right[leg.get_leg_index()][self.move_mode])
        self.move_mode += 1

    def move_body_tripot(self, direct='forward'):
        if self.move_mode == -1 or self.move_mode >= self.tripot_gait_len - 1:
            self.move_mode = 0
        for leg in self.leg:
            if direct == 'forward':
                leg.set_tip_pos_tip(self.tripot_gait_path_forward[leg.get_leg_index()][self.move_mode])
            elif direct == 'backward':
                leg.set_tip_pos_tip(self.tripot_gait_path_backward[leg.get_leg_index()][self.move_mode])
            else:
                raise ValueError
        self.move_mode += 1

    def draw_robo(self):
        for body in self.body:
            self.ax.plot(body[0], body[1], body[2], linewidth=6, color='black')
        for leg in self.leg:
            leg.draw_leg_3d()

class Heaxapod:
    def __init__(self, mode='normal'):
        self._servo = ServoControl(mode=mode)
        self._leg = [Leg(index, default_angle_pos=DEFAULT_POS) for index in leg_index]
        self._upperLeg = [ShortLeg(index, devia=0.0) for index in upper_leg_index]
        self.set_deviation = self._servo.set_deviation
        self.test_action_table = gen_forward_path_table(10)
        self.default_len = -1
        self.default_word_point = []

    def set_default_pos(self, default_pos_angle, move=True, Time=1000):
        move_list = []
        for leg in self._leg:
            leg.set_tip_pos_theta(default_pos_angle)
            leg.set_default_point(leg.get_tip_data()[1])
            self.default_len = self._leg[0].get_tip_data()[2].y
            self.default_word_point = [leg.get_tip_data()[1] for leg in self._leg]
            move_list += leg.get_servo_move_array()
        if move:
            self._servo.set_servo_angle(move_list, Time)

    def set_Upper_pos(self, default_pos, leg_id_list='All'):
        move_list = []
        leg_list = ['LF', 'LM', 'LH', 'RF', 'RM', 'RH']
        if leg_id_list != 'All':
            leg_list = leg_id_list
        for leg in self._upperLeg:
            if leg.get_leg_index() not in leg_list:
                continue
            leg.set_pos(default_pos)
            move_list += leg.get_servo_move_array()
        self._servo.set_servo_angle(move_list, 1000)

    def set_lower_pos(self, default_pos, leg_id_list='All'):
        move_list = []
        leg_list = ['LF', 'LM', 'LH', 'RF', 'RM', 'RH']
        if leg_id_list != 'All':
            leg_list = leg_id_list
        for leg in self._leg:
            if leg.get_leg_index() not in leg_list:
                continue
            leg.set_tip_pos_theta(default_pos)
            move_list += leg.get_servo_move_array()
        self._servo.set_servo_angle(move_list, 1000)

    def load_deviation_file(self, path=None):
        if path is not None:
            self._servo.load_deviation_file(path)
        else:
            self._servo.load_deviation_file()

    def save_deviation_file(self):
        self._servo.save_deviation_file(self._servo.get_deviation_data())

    def loose_all_leg(self):
        id = [i for i in range(1, 31)]
        self._servo.loose_servo(id)

    def show_servo_status(self, show_deviation=False):
        servo_data_raw = self._servo.get_raw_servo_data()
        deviation = self._servo.get_deviation_data()
        print("Available Servo List:\n")
        for i in range(len(servo_data_raw)):
            if deviation[i][0] == -1 or deviation[i][1] == -1:
                print('ID:' + str(i+1) + '|' + 'Position:' + str(servo_data_raw[i]) + '(raw)' + '\n')
            else:
                if i+1 in (3, 6, 9, 12, 15, 18, 20, 22, 24, 26, 28, 30):
                    print('ID:' + str(i+1) + '|' + 'Position:' + str(self._leg[0].devia + 90.0 * (servo_data_raw[i] - deviation[i][0])
                                                                     / (deviation[i][1] - deviation[i][0])) + '°\n')
                else:
                    print('ID:' + str(i+1) + '|' + 'Position:' + str(90.0 * (servo_data_raw[i] - deviation[i][0])
                                                                     / (deviation[i][1] - deviation[i][0])) + '°\n')
        if show_deviation:
            self._servo.show_deviation_settings()

    def move_by_action_point_table(self, action_table, Time=500, Test=True):
        action_len = action_table['len']
        # if action_table['type'] != 'point':
        #     raise ValueError
        for action in range(action_len):
            servo_move_table = []
            for leg in self._leg:
                if Test:
                    if leg.get_leg_index() != 'LF' and leg.get_leg_index() != 'LM':
                        continue
                if action_table['type'] == 'world_frame_point':
                    leg.set_tip_pos(action_table[leg.get_leg_index()][action])
                elif action_table['type'] == 'local_frame_point':
                    leg.set_tip_pos_local(action_table[leg.get_leg_index()][action])
                elif action_table['type'] == 'tip_frame_point':
                    leg.set_tip_pos_tip(action_table[leg.get_leg_index()][action])
                elif action_table['type'] == 'angle':
                    leg.set_tip_pos_theta(action_table[leg.get_leg_index()][action])
                else:
                    raise ValueError
                id_angle = leg.get_servo_move_array()
                servo_move_table += id_angle

            if action_table['type'] == 'angle':
                for leg in self._upperLeg:
                    if Test:
                        if leg.get_leg_index() != 'LF' and leg.get_leg_index() != 'LM':
                            continue
                    leg.set_pos(action_table[leg.get_leg_index()][action])
                    servo_move_table += leg.get_servo_move_array()
            servo_move_table = servo_move_table
            CMD = self._servo.set_servo_angle(servo_move_table, Time / action_len)

    @staticmethod
    def get_leg_servo_id(index):
        if index == 'LF':
            leg_servo_id = [1, 2, 3]
        elif index == 'LM':
            leg_servo_id = [4, 5, 6]
        elif index == 'LH':
            leg_servo_id = [10, 11, 12]
        elif index == 'RF':
            leg_servo_id = [16, 17, 18]
        elif index == 'RM':
            leg_servo_id = [7, 8, 9]
        elif index == 'RH':
            leg_servo_id = [13, 14, 15]
        elif index == 'ULF':
            leg_servo_id = [25, 26]
        elif index == 'ULM':
            leg_servo_id = [27, 28]
        elif index == 'ULH':
            leg_servo_id = [23, 24]
        elif index == 'URF':
            leg_servo_id = [29, 30]
        elif index == 'URM':
            leg_servo_id = [21, 22]
        elif index == 'URH':
            leg_servo_id = [19, 20]
        else:
            raise ValueError
        return leg_servo_id

    def _gen_c_path_table_fix(self, action_table, Time=500):
        #self.set_default_pos(DEFAULT_POS_3, move=False)
        action_len = action_table['len']
        Time_slice = int(Time / action_len)
        angle_id_table = []
        for action in range(action_len):
            servo_move_table = []

            moved_leg_Id = []

            for leg in self._leg:
                if leg.get_leg_index() in action_table.keys():
                    if len(action_table[leg.get_leg_index()]) == 0:
                        continue
                    else:
                        moved_leg_Id += leg.get_servo_id(leg.get_leg_index())

                    if action_table['type'] == 'world_frame_point':
                        leg.set_tip_pos(action_table[leg.get_leg_index()][action])
                    elif action_table['type'] == 'local_frame_point':
                        leg.set_tip_pos_local(action_table[leg.get_leg_index()][action])
                    elif action_table['type'] == 'tip_frame_point':
                        leg.set_tip_pos_tip(action_table[leg.get_leg_index()][action])
                    elif action_table['type'] == 'angle':
                        try:
                            leg.set_tip_pos_theta(action_table[leg.get_leg_index()][action])
                        except:
                            continue
                    else:
                        raise ValueError
                    id_angle = leg.get_servo_move_array()
                    servo_move_table += id_angle

            if action_table['type'] == 'angle':
                for leg in self._upperLeg:
                    if leg.get_leg_index() in action_table.keys():
                        if len(action_table[leg.get_leg_index()]) == 0:
                            continue
                        else:
                            moved_leg_Id += leg.get_servo_id(leg.get_leg_index())
                        try:
                            leg.set_pos(action_table[leg.get_leg_index()][action])
                            servo_move_table += leg.get_servo_move_array()
                        except:
                            pass
            angle_id_table.append(self._servo.convert_servo_angle(servo_move_table))

        with open('movementTable.h', 'a') as f:
            f.write("void movement_"+action_table['name']+'();\n')
        with open('movementTable.c', 'a') as f:
            f.write('void movement_'+action_table['name']+'(){\n')
            for item in angle_id_table:
                f.write('moveServos(' + str(len(item)) + ', ' +
                        str(Time_slice))
                for id_angle in item:
                    if id_angle[0] in moved_leg_Id:
                        f.write(', ' + str(id_angle[0]) + ', ' + str(int(id_angle[1])))
                f.write(');\n')
                f.write('HAL_Delay(' + str(Time_slice) + ');\n')
            f.write('}\n')

    def _gen_c_path_table(self, action_table, Time=500):
        action_len = action_table['len']
        Time_slice = int(Time / action_len)
        angle_id_table = []
        for action in range(action_len):
            servo_move_table = []

            for leg in self._leg:
                if action_table['type'] == 'world_frame_point':
                    leg.set_tip_pos(action_table[leg.get_leg_index()][action])
                elif action_table['type'] == 'local_frame_point':
                    leg.set_tip_pos_local(action_table[leg.get_leg_index()][action])
                elif action_table['type'] == 'tip_frame_point':
                    leg.set_tip_pos_tip(action_table[leg.get_leg_index()][action])
                elif action_table['type'] == 'angle':
                    try:
                        leg.set_tip_pos_theta(action_table[leg.get_leg_index()][action])
                    except:
                        continue
                else:
                    raise ValueError
                id_angle = leg.get_servo_move_array()
                servo_move_table += id_angle

            if action_table['type'] == 'angle':
                for leg in self._upperLeg:
                    try:
                        leg.set_pos(action_table[leg.get_leg_index()][action])
                        servo_move_table += leg.get_servo_move_array()
                    except:
                        pass
            angle_id_table.append(self._servo.convert_servo_angle(servo_move_table))

        with open('movementTable.h', 'a') as f:
            f.write("void movement_"+action_table['name']+'();\n')
        with open('movementTable.c', 'a') as f:
            f.write('void movement_'+action_table['name']+'(){\n')
            for item in angle_id_table:
                f.write('moveServos(' + str(len(item)) + ', ' +
                        str(Time_slice))
                for id_angle in item:
                    f.write(', ' + str(id_angle[0]) + ', ' + str(int(id_angle[1])))
                f.write(');\n')
                f.write('HAL_Delay(' + str(Time_slice) + ');\n')
            f.write('}\n')

    def gen_movement_table(self, div_num=4):
        with open('movementTable.h', 'w') as f:
            f.write("#ifndef UARTTEST_MOVEMENTTABLE_H\n"
                    "#define UARTTEST_MOVEMENTTABLE_H\n"
                    "#include \"Servo.h\"\n")
        with open('movementTable.c', 'w') as f:
            f.write("#include \"Servo.h\"\n"
                    "#include \"movementTable.h\"\n")
        # self._gen_c_path_table(gen_forward_Ellipse_path_table(20, 0.5, div_num=div_num), 100)
        # self._gen_c_path_table(gen_backward_Ellipse_path_table(20, 0.5, div_num=div_num), 100)
        # self._gen_c_path_table(gen_forward_path_table(20, div_num=div_num), 100)
        # self._gen_c_path_table(gen_backward_path_table(20, div_num=div_num), 100)
        # self._gen_c_path_table(gen_rotate_left_table(5, rHex.default_word_point), 100)
        # self._gen_c_path_table(gen_rotate_right_table(5, rHex.default_word_point), 100)
        # self._gen_c_path_table(gen_turn_path_left_table(8, rHex.default_len, div_num=div_num), 100)
        # self._gen_c_path_table(gen_turn_path_right_table(8, rHex.default_len, div_num=div_num), 100)
        # self._gen_c_path_table(gen_leftward_path_table(20, div_num=8), 500)
        # self._gen_c_path_table(gen_rightward_path_table(20, div_num=8), 500)
        # self._gen_c_path_table(gen_low_down_path_table(20), 400)
        # self._gen_c_path_table(gen_lift_up_path_table(30), 400)
        # self._gen_c_path_table(gen_twist_path_table(self.default_word_point, div_num=5, angle=10), 500)
        # self._gen_c_path_table(gen_default_pos_table(DEFAULT_POS_3), 1000)
        # self._gen_c_path_table(gen_step_convert(100, 2), 4000)
        # self._gen_c_path_table(gen_wave_Ellipse_path_table(20), 500)
        # self._gen_c_path_table(gen_tetrapod_Ellipse_path_table(20), 500)
        # self._gen_c_path_table(gen_ripple_Ellipse_path_table(20), 500)
        # self._gen_c_path_table(gen_rolling_loop([60, -25], [90, -25]), 6000)
        # self._gen_c_path_table(gen_rolling_loop([40, -10], [100, -25]))
        # self._gen_c_path_table(stage_1(), 800)
        self._gen_c_path_table_fix(gen_leg_hold_up(), 1000)
        self._gen_c_path_table(gen_rotate_right_table(5, self.default_word_point), 2000)
        self._gen_c_path_table_fix(gen_rolling_loop([40, -10], [100, -25]), 300)
        self._gen_c_path_table_fix(gen_up_hold(), 1000)
        self._gen_c_path_table_fix(gen_up_crawl_hold(),1000)
        self._gen_c_path_table_fix(gen_up_half_open(),1000)
        with open('movementTable.h', 'a') as f:
            f.write("#endif //UARTTEST_MOVEMENTTABLE_H")

    def gen_deviation_c_file(self):
        devia_data = self._servo.get_deviation_data()
        with open('ServoDeviation.h', 'w') as f:
            f.write('#ifndef UARTTEST_SERVODEVIATION_H\n#define UARTTEST_SERVODEVIATION_H\n\n\n')
            for ID in range(0, 30):
                f.write('#define SERVO_' + str(ID+1) + '_MIN_'+'ANGLE  ' + str(devia_data[ID][0]) + '\n')
                f.write('#define SERVO_' + str(ID + 1) + '_MAX_' + 'ANGLE  ' + str(devia_data[ID][1]) + '\n')
            f.write('#endif\n\n')

    def convert_servo_angle(self, mo_li):
        return self._servo.convert_servo_angle(mo_li)

    def gen_movement_func_text(self, movement_list, Time=1000):
        move_li = self._servo.convert_servo_angle(movement_list)
        total_len = len(move_li)
        text = "\nmoveServos({len}, {time}".format(len=total_len, time=Time)
        for id_angle in move_li:
            text += ',{ID} ,{angle}'.format(ID=id_angle[0], angle=int(id_angle[1]))
        text += ');\n'
        print(text)


if __name__ == '__main__':
    rHex = Heaxapod(mode='Test')
    while True:#115#112#70
        rHex.set_default_pos(DEFAULT_POS_3, move=False)
        rHex.load_deviation_file()

        rHex.gen_movement_func_text([[1, 0], [2, 0], [3, 120],
                                     [4, 0],
                                     [7, 0], [8, 0], [9, 120],
                                     [10, 0], [11, 0], [12, 120]])


        setting = input("Choose Func:\n")
        if setting == '1':
            while True:
                print('[1]Low coxa MIN\n')
                print('[2]Low femur MIN\n')
                print('[3]Low tibia MIN\n')
                print('[4]Low coxa MAX\n')
                print('[5]Low femur MAX\n')
                print('[6]Low tibia MAX\n')
                print('[7]quit\n')
                print('[8]show deviation data\n')
                print('[9]save deviation data\n')
                pos_list = ['LOWER_TIBIA', 'LOWER_COXA', 'LOWER_FEMUR']
                flag = ['MIN', 'MAX']
                devia = int(input('Choose part>>\n'))
                if devia <= 3:
                    flag_pos = 0
                else:
                    flag_pos = 1
                if devia == 7:
                    break
                elif devia == 8:
                    rHex.show_servo_status()
                elif devia == 9:
                    rHex.save_deviation_file()
                elif devia == 10:
                    rHex.set_deviation('UPPER_FEMUR', 'MIN')
                elif devia == 11:
                    rHex.set_deviation('UPPER_FEMUR', 'MAX')
                elif devia == 12:
                    rHex.set_deviation('UPPER_TIBIA', 'MIN')
                elif devia == 13:
                    rHex.set_deviation('UPPER_TIBIA', 'MAX')
                else:
                    rHex.set_deviation(pos_list[devia % 3], flag[flag_pos])

        elif setting == '3':
            r = input("r is:\n")
            div = input("div num is\n")
            for _ in range(0, 5):
                rHex.move_by_action_point_table(gen_forward_path_table(floor(int(r)), div_num=int(div)), Time=100, Test=False)

        elif setting == '4':
            rHex.loose_all_leg()

        elif setting == '5':
            r = input("r is:\n")
            div = input("div num is\n")
            for _ in range(0, 5):
                rHex.move_by_action_point_table(gen_turn_path_left_table(floor(int(r)), rHex.default_len, div_num=int(div)), Time=500, Test=False)

        elif setting == '7':
            r = input("r is:\n")
            for _ in range(0, 5):
                rHex.move_by_action_point_table(gen_rotate_left_table(int(r), rHex.default_word_point))

        elif setting == '8':
            r = input("r is:\n")
            div = input("div num is\n")
            for _ in range(0, 5):
                rHex.move_by_action_point_table(gen_forward_Ellipse_path_table(int(r), 0.5, div_num=int(div)), Test=False)

        elif setting == '9':
            rHex.gen_movement_table()

        elif setting == '10':
            pos_1 = int(input("pos 1 is:\n"))
            pos_2 = int(input("pos 2 is:\n"))
            tag = input("index is:\n")
            rHex.set_Upper_pos([pos_1, pos_2], leg_id_list=tag)

        elif setting == '11':
            pos_1 = int(input("pos 1 is:\n"))
            pos_2 = int(input("pos 2 is:\n"))
            pos_3 = int(input("pos 3 is:\n"))
            tag = input("index is:\n")
            rHex.set_lower_pos([pos_1, pos_2, pos_3], leg_id_list=tag)

        elif setting == '12':
            rHex.gen_deviation_c_file()











