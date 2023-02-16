import servo as servo
import json
class ServoControl:
    def __init__(self, serialPort='COM5', mode='normal'):
        self._servo_deviation_data = [[-1, -1] for _ in range(0, 30)]
        if mode != 'test':
            self._servo = servo.SERVO_UART(serialPort)
        else:
            pass

    def loose_servo(self, id_list):
        self._servo.UNLOAD_SERVO(id_list)

    def load_deviation_file(self, path='servo_deviation.json'):
        with open(path, 'r') as f:
            data = json.load(f)
            servo_deviation_data = [[-1.0, -1.0] for _ in range(len(data))]
            for info in data:
                self._servo_deviation_data[data[info]['ID'] - 1][0] = data[info]['MIN']
                self._servo_deviation_data[data[info]['ID'] - 1][1] = data[info]['MAX']
                servo_deviation_data[data[info]['ID'] - 1][0] = data[info]['MIN']
                servo_deviation_data[data[info]['ID'] - 1][1] = data[info]['MAX']
        return servo_deviation_data

    @staticmethod
    def save_deviation_file(servo_deviation_data):
        with open('servo_deviation.json', 'w') as f:
            save_file = {}
            for i in range(0, len(servo_deviation_data)):
                save_file.update({
                'ID'+str(i+1): {
                    'ID': i + 1,
                    'MIN': servo_deviation_data[i][0],
                    'MAX': servo_deviation_data[i][1]
                }
                })
            json.dump(save_file, f)

    def get_deviation_data(self):
        return self._servo_deviation_data

    def get_raw_servo_data(self):
        ID = []
        for i in range(1, 31):
            ID.append(i)
        return self._servo.CMD_GET_SERVO_ANGLE(ID)

    def set_raw_servo_data(self, angle, Time):
        return self._servo.CMD_MOVE_SERVO(angle, Time)

    def set_deviation(self, position, flag):
        angle = self.get_raw_servo_data()
        if position == "LOWER_COXA":
            for i in range(0, 6):
                if flag == 'MIN':
                    self._servo_deviation_data[0 + i * 3][0] = angle[0 + i * 3]
                else:
                    if self._servo_deviation_data[0][0] < angle[0]:
                        self._servo_deviation_data[0 + i * 3][1] = \
                            self._servo_deviation_data[0 + i * 3][0] + angle[0] - self._servo_deviation_data[0][0]
                    else:
                        self._servo_deviation_data[0 + i * 3][1] = \
                            self._servo_deviation_data[0 + i * 3][0] - (angle[0] - self._servo_deviation_data[0][0])
        elif position == "LOWER_FEMUR":
            for i in range(0, 6):
                if flag == 'MIN':
                    self._servo_deviation_data[1 + i * 3][0] = angle[1 + i * 3]
                else:
                    self._servo_deviation_data[1 + i * 3][1] = angle[1 + i * 3]
        elif position == "LOWER_TIBIA":
            for i in range(0, 6):
                if flag == 'MIN':
                    self._servo_deviation_data[2 + i * 3][0] = angle[2 + i * 3]
                else:
                    self._servo_deviation_data[2 + i * 3][1] = angle[2 + i * 3]
        elif position == "UPPER_FEMUR":
            for i in range(0, 6):
                if flag == 'MIN':
                    self._servo_deviation_data[18 + i * 2][0] = angle[18 + i * 2]
                else:
                    self._servo_deviation_data[18 + i * 2][1] = angle[18 + i * 2]
        elif position == "UPPER_TIBIA":
            for i in range(0, 6):
                if flag == 'MIN':
                    self._servo_deviation_data[18+1 + i * 2][0] = angle[18+1 + i * 2]
                else:
                    self._servo_deviation_data[18+1 + i * 2][1] = angle[18+1 + i * 2]
        pass

    def show_deviation_settings(self):
        print("Deviation settings:\n" +
              ("ID:" + str(i+1) +
               ' MIN:' + str(self._servo_deviation_data[i][0]) +
               ' MAX:' + str(self._servo_deviation_data[i][1]) +
               '\n') for i in range(0, 30))

    def get_servo_angle(self, servo_list=None):
        raw = self.get_raw_servo_data()
        if servo_list is None:
            theta = [0.0] * 30
            for i in range(0, 30):
                theta[i] = 90 * (raw[i] - self._servo_deviation_data[i][0]) \
                        / (self._servo_deviation_data[i][1] - self._servo_deviation_data[i][0])
        else:
            theta = [[servo_id, raw[servo_id-1]] for servo_id in servo_list]
        return theta

    def set_servo_angle(self, theta, Time):
        angle = theta
        for ID_angle in angle:
            ID_angle[1] = ID_angle[1]/90.0 * (self._servo_deviation_data[ID_angle[0]-1][1] - self._servo_deviation_data[ID_angle[0]-1][0]) \
                       + self._servo_deviation_data[ID_angle[0]-1][0]
        return self.set_raw_servo_data(angle, Time)

    def convert_servo_angle(self, theta):
        angle = theta
        for ID_angle in angle:
            ID_angle[1] = ID_angle[1]/90.0 * (self._servo_deviation_data[ID_angle[0]-1][1] - self._servo_deviation_data[ID_angle[0]-1][0]) \
                       + self._servo_deviation_data[ID_angle[0]-1][0]
        return angle
