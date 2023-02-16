import serial
import time
import serial.tools.list_ports
import threading
import math
import re
FRAME_HEADER = b'\x55\x55'
CMD_GET_ANGLE = b'\x15'
CMD_SERVO_UNLOAD = b'\x14'
CMD_SERVO_MOVE = b'\x03'
TOTAL_SERVO_NUM = 30

DATA = ""  # 读取的数据
DATA_ALL = bytearray(40960)
DATA_LEN = 0
NO_END = True  # 是否读取结束
DATA_IN_FLAG = 0
START_TIME = 0
SEND = 0
GET = 0
SERVO_ANGLE = [0] * TOTAL_SERVO_NUM

class SERVO_UART:
    def __init__(self, Port='COM5', Baud=9600):
        self.serialPort, ret = self.open_serial(Port, Baud, None)

    def GET_SERVO_PORT(self):
        port_list = list(serial.tools.list_ports.comports())
        print(port_list)
        if len(port_list) == 0:
            print("无可用串口！")
        else:
            for i in range(0, len(port_list)):
                port_list[i] = re.findall("(?<=COM)[0-9]", port_list[i])[0]
                print(port_list[i])
        print(time.time())
        return port_list

    def UNLOAD_SERVO(self, servo_list):
        CMD = FRAME_HEADER
        CMD_LEN = self.UINT_2_BYTES(len(servo_list) + 3)
        ID = self.UINT_2_BYTES(len(servo_list))
        for servo in servo_list:
            ID += self.UINT_2_BYTES(servo)
        self.write_to_serial(CMD + CMD_LEN + CMD_SERVO_UNLOAD + ID)

    def CMD_GET_SERVO_ANGLE(self, servo_list):
        global DATA, NO_END, DATA_IN_FLAG, START_TIME, DATA_ALL, DATA_LEN, SERVO_ANGLE, SEND, GET
        SEND = 1
        GET = 0
        CMD = FRAME_HEADER
        CMD_LEN = self.UINT_2_BYTES(len(servo_list) + 3)
        ID = self.UINT_2_BYTES(len(servo_list))
        for servo in servo_list:
            ID += self.UINT_2_BYTES(servo)
        self.write_to_serial(CMD + CMD_LEN + CMD_GET_ANGLE + ID)
        while True:
            if SEND == 0 and GET == 1:
                return SERVO_ANGLE

    def CMD_MOVE_SERVO(self, servo_angle, Time):
        CMD = FRAME_HEADER
        CMD_LEN = self.UINT_2_BYTES(len(servo_angle) * 3 + 5)
        ID = self.UINT_2_BYTES(len(servo_angle)) + self.UINT_2_BYTES(int(Time) & 0xFF) + self.UINT_2_BYTES(int(Time) >> 8)
        for ID_angle in servo_angle:
            ID += self.UINT_2_BYTES(ID_angle[0]) + self.UINT_2_BYTES(int(ID_angle[1]) & 0xFF) + self.UINT_2_BYTES(
                (int(ID_angle[1]) >> 8) & 0xFF)
        # for i in (0, len(servo_angle)):
        #     ID += self.UINT_2_BYTES(i+1) + self.UINT_2_BYTES(servo_angle[i] & 0xFF) + self.UINT_2_BYTES(
        #         (servo_angle[i] >> 8) & 0xFF)
        self.write_to_serial(CMD + CMD_LEN + CMD_SERVO_MOVE + ID)
        return CMD + CMD_LEN + CMD_SERVO_MOVE + ID

    # 读数据的本体
    @staticmethod
    def read_data(ser):
        global DATA, NO_END, DATA_IN_FLAG, START_TIME, DATA_ALL, DATA_LEN, SERVO_ANGLE, SEND, GET
        START_TIME = time.time()
        # 循环接收数据（此为死循环，可用线程实现）.
        while NO_END:
            if ser.in_waiting:
                START_TIME = time.time()
                DATA_IN_FLAG = 1
                length = ser.in_waiting
                DATA = ser.read(length)  # 注意 ser.read了之后 in_waiting马上变成0了
                for i in range(0, length):
                    DATA_ALL[DATA_LEN + i] = DATA[i]
                DATA_LEN += length

            else:
                if DATA_IN_FLAG:
                    if time.time() - START_TIME > 0.010:  # 串口超时10ms
                        DATA_PRINT = bytearray(DATA_LEN)
                        for i in range(0, DATA_LEN):
                            DATA_PRINT[i] = DATA_ALL[i]
                        if DATA_PRINT[0] != 0x55 or DATA_PRINT[1] != 0x55:
                            print("\nUnknown command\n")
                        else:
                            CMD_TYPE = DATA_PRINT[3]
                            if CMD_TYPE == 0x15:
                                Total_num = int(DATA_PRINT[4])
                                for i in range(0, Total_num):
                                    ID = int(DATA_PRINT[5 + i * 3]) - 1
                                    ANGLE = int(DATA_PRINT[7 + i * 3] * (2 ** 8) + DATA_PRINT[6 + i * 3])
                                    SERVO_ANGLE[ID] = ANGLE
                                GET = 1
                                SEND = 0
                                print('Receive servo angle data\n')
                        START_TIME = time.time()
                        DATA_IN_FLAG = 0
                        DATA_LEN = 0

    # 打开串口
    def open_serial(self, port, bps, timeout):
        ret = False
        ser = None
        try:
            # 打开串口，并得到串口对象
            ser = serial.Serial(port, bps, timeout=timeout)
            # 判断是否成功打开
            if ser.is_open:
                ret = True
                th = threading.Thread(target=self.read_data, args=(ser,))  # 创建一个子线程去等待读数据
                th.start()
        except Exception as e:
            print("error!", e)
        return ser, ret

    # 关闭串口
    def close_serial(self, ser):
        global NOEND
        NOEND = False
        ser.close()

    # 写数据
    def write_to_serial(self, text):
        res = self.serialPort.write(text)  # 写
        return res

    # 读数据
    @staticmethod
    def read_from_serial():
        global DATA
        data = DATA
        DATA = ""  # 清空当次读取
        return data

    @staticmethod
    def UINT_2_BYTES(num, byteorder='little', signed=False):
        if num == 0:
            return b'\x00'
        return num.to_bytes(length=max(math.ceil(math.log(num, 2) / 8.0), 1), byteorder=byteorder,
                            signed=signed)

