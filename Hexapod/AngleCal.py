import streamlit as st
from Hexapod import *
import pyperclip

st.markdown("# 动作文件生成")
st.markdown("---")
vHex = Heaxapod()
vHex.set_default_pos(DEFAULT_POS_3, move=False)
vHex.load_deviation_file()
leg_set = ['LF', 'LM', 'LH', 'RF', 'RM', 'RH', "None"]
upper_leg = ['ULF', 'ULM', 'ULH', 'URF', 'URM', 'URH', "None"]
movement = []
_servo = [-1000, -1000, -1000]
_uServo = [-1000, -1000, -1000]
legControl, upper_legControl = st.columns(2)
with legControl:
    legOption = st.multiselect(
        'Choose Leg:',
        leg_set,
        "None"
    )
    _servo[0] = st.number_input("1st servo angle:", -1000)
    _servo[1] = st.number_input("2nd servo angle:", -1000)
    _servo[2] = st.number_input("3rd servo angle:", -1000)
with upper_legControl:
    UpperLegOption = st.multiselect(
        'Choose Upper Leg:',
        upper_leg,
        "None"
    )
    _uServo[0] = st.number_input("1st uservo angle:", -1000)
    _uServo[1] = st.number_input("2nd uservo angle:", -1000)
Time = st.number_input("movement Time:", 1000)

if st.button('生成动作文件'):
    move_list = []
    for leg in legOption:
        if leg == "None":
            break
        servo_list = vHex.get_leg_servo_id(leg)
        for i in range(0, len(servo_list)):
            if _servo[i] == -1000:
                continue
            move_list.append([servo_list[i], _servo[i]])

    for leg in UpperLegOption:
        if leg == "None":
            break
        servo_list = vHex.get_leg_servo_id(leg)
        for i in range(0, len(servo_list)):
            if _uServo[i] == -1000:
                continue
            move_list.append([servo_list[i], _uServo[i]])
    move_list = vHex.convert_servo_angle(move_list)
    total_len = len(move_list)

    move = "\nmoveServos({len}, {time}".format(len=total_len, time=Time)
    for id_angle in move_list:
        move += ',{ID} ,{angle}'.format(ID=id_angle[0], angle=int(id_angle[1]))
    move += ');\n'
    pyperclip.copy(move)
    st.info("已成功生成动作文件至剪切板")
else:
    pass




