from collections import deque
from Inversekinematics import *
from math_utils import *

def gen_twist_path_table(localPoint, angle=5, div_num=2):
    pointExtesion = []
    for i in range(0, div_num+1):
        pointExtesion.append([rotate(point, i*angle/div_num) for point in localPoint])
    for i in range(1, div_num+1):
        pointExtesion.append([rotate(point, angle - i*angle/div_num) for point in localPoint])
    for i in range(0, div_num+1):
        pointExtesion.append([rotate(point, -i*angle/div_num) for point in localPoint])
    for i in range(1, div_num+1):
        pointExtesion.append([rotate(point, i*angle/div_num - angle) for point in localPoint])
    return {
        'name': 'twist',
        'type': 'world_frame_point',
        'LF': [r_path[0] for r_path in pointExtesion],
        'LM': [r_path[1] for r_path in pointExtesion],
        'LH': [r_path[2] for r_path in pointExtesion],
        'RH': [r_path[3] for r_path in pointExtesion],
        'RM': [r_path[4] for r_path in pointExtesion],
        'RF': [r_path[5] for r_path in pointExtesion],
        'len': len(pointExtesion)
    }

def gen_rightward_path_table(radius, div_num=20):
    path = []
    slice_length = 2 * radius / div_num
    for i in range(0, div_num+1):
        x = 0
        y = i * slice_length - radius
        z = sqrt(radius**2 - x**2)
        path.append(point3d(x, y, z))
    for i in range(1, div_num):
        x = 0
        y = radius - i * slice_length
        z = 0
        path.append(point3d(x, y, z))
    path_r = deque(path)
    path_r.rotate(-int(div_num/2))
    path_r.reverse()
    R_group = list(path_r)
    path_l = deque(path)
    path_l.rotate(-int(div_num/2)*3)
    path_l.reverse()
    L_group = list(path_l)
    path_table = {
        'name': 'normal_rightward',
        'type': 'tip_frame_point',
        'RM': R_group,
        'LF': R_group,
        'LH': R_group,
        'LM': L_group,
        'RF': L_group,
        'RH': L_group,
        'len': len(L_group)
    }
    return path_table

def gen_leftward_path_table(radius, div_num=20):
    path = []
    slice_length = 2 * radius / div_num
    for i in range(0, div_num+1):
        x = 0
        y = i * slice_length - radius
        z = sqrt(radius**2 - x**2)
        path.append(point3d(x, y, z))
    for i in range(1, div_num):
        x = 0
        y = radius - i * slice_length
        z = 0
        path.append(point3d(x, y, z))
    path_r = deque(path)
    path_r.rotate(-int(div_num/2))
    R_group = list(path_r)
    path_l = deque(path)
    path_l.rotate(-int(div_num/2)*3)
    L_group = list(path_l)
    path_table = {
        'name': 'normal_leftward',
        'type': 'tip_frame_point',
        'RM': R_group,
        'LF': R_group,
        'LH': R_group,
        'LM': L_group,
        'RF': L_group,
        'RH': L_group,
        'len': len(R_group)
    }
    return path_table

def gen_forward_path_table_high(radius, div_num=20):
    path = []
    slice_length = 2 * radius / div_num
    for i in range(0, div_num+1):
        x = i * slice_length - radius
        y = 0
        z = sqrt(radius**2 - x**2)
        path.append(point3d(x, y, z))
    for i in range(1, div_num):
        x = radius - i * slice_length
        y = 0
        z = 0
        path.append(point3d(x, y, z))
    path_r = deque(path)
    path_r.rotate(-int(div_num/2))
    R_group = list(path_r)
    path_l = deque(path)
    path_l.rotate(-int(div_num/2)*3)
    L_group = list(path_l)
    for i in range(0, len(R_group)):
        R_group[i] = R_group[i].__sub__(point3d(0, 0, 28))
    for i in range(0, len(L_group)):
        L_group[i] = L_group[i].__sub__(point3d(0, 0, 28))
    path_table = {
        'name': 'normal_forward',
        'type': 'tip_frame_point',
        'RM': R_group,
        'LF': R_group,
        'LH': R_group,
        'LM': L_group,
        'RF': L_group,
        'RH': L_group,
        'len': len(R_group)
    }
    return path_table

def gen_forward_path_table(radius, div_num=20):
    path = []
    slice_length = 2 * radius / div_num
    for i in range(0, div_num+1):
        x = i * slice_length - radius
        y = 0
        z = sqrt(radius**2 - x**2)
        path.append(point3d(x, y, z))
    for i in range(1, div_num):
        x = radius - i * slice_length
        y = 0
        z = 0
        path.append(point3d(x, y, z))
    path_r = deque(path)
    path_r.rotate(-int(div_num/2))
    R_group = list(path_r)
    path_l = deque(path)
    path_l.rotate(-int(div_num/2)*3)
    L_group = list(path_l)
    path_table = {
        'name': 'normal_forward',
        'type': 'tip_frame_point',
        'RM': R_group,
        'LF': R_group,
        'LH': R_group,
        'LM': L_group,
        'RF': L_group,
        'RH': L_group,
        'len': len(R_group)
    }
    return path_table

def gen_tetrapod_Ellipse_path_table(radius, rate=0.5, div_num=2):
    path = []
    air_slice_length = 2 * radius / div_num
    for i in range(0, div_num+1):
        x = i * air_slice_length - radius
        y = 0
        z = sqrt((radius / (1 - (1 - abs(x/radius)) * rate))**2 - x**2)
        path.append(point3d(x, y, z))
    ground_slice_length = radius / (div_num+1)
    for i in range(1, (div_num+1)*2 + 1):
        x = radius - i * ground_slice_length
        y = 0
        z = 0
        path.append(point3d(x, y, z))
    path_table = {
        'name': 'tetrapod_forward',
        'type': 'tip_frame_point',
        'len': len(path)
    }
    path_table.update({'LF':path})
    path_table.update({'RH':path})
    queue = deque(path)
    queue.rotate(div_num)
    path_table.update({'LM':list(queue)})
    path_table.update({'RF':list(queue)})
    queue.rotate(div_num)
    path_table.update({'LH':list(queue)})
    path_table.update({'RM':list(queue)})
    return path_table

def gen_wave_Ellipse_path_table(radius, rate=0.5, div_num=2):
    path = []
    air_slice_length = 2 * radius / div_num
    for i in range(0, div_num+1):
        x = i * air_slice_length - radius
        y = 0
        z = sqrt((radius / (1 - (1 - abs(x/radius)) * rate))**2 - x**2)
        path.append(point3d(x, y, z))
    ground_slice_length = 2*radius / ((div_num+1)*5)
    for i in range(1, (div_num+1)*5 + 1):
        x = radius - i * ground_slice_length
        y = 0
        z = 0
        path.append(point3d(x, y, z))
    path_table = {
        'name': 'wave_forward',
        'type': 'tip_frame_point',
        'len': len(path)
    }
    queue = deque(path)
    path_table.update({'LF':path})
    queue.rotate(div_num)
    path_table.update({'LM':list(queue)})
    queue.rotate(div_num)
    path_table.update({'LH':list(queue)})
    queue.rotate(div_num)
    path_table.update({'RF':list(queue)})
    queue.rotate(div_num)
    path_table.update({'RM':list(queue)})
    queue.rotate(div_num)
    path_table.update({'RH':list(queue)})
    return path_table

def gen_ripple_Ellipse_path_table(radius, rate=0.5, div_num=20):
    path = []
    air_slice_length = 2 * radius / div_num
    for i in range(0, div_num + 1):
        x = i * air_slice_length - radius
        y = 0
        z = sqrt((radius / (1 - (1 - abs(x / radius)) * rate)) ** 2 - x ** 2)
        path.append(point3d(x, y, z))
    ground_slice_length = 2 * radius / ((div_num + 1) * 5)
    for i in range(1, (div_num + 1) * 5 + 1):
        x = radius - i * ground_slice_length
        y = 0
        z = 0
        path.append(point3d(x, y, z))
    path_table = {
        'name': 'ripple_forward',
        'type': 'tip_frame_point',
        'len': len(path)
    }
    queue = deque(path)
    path_table.update({'LF': path})
    queue.rotate(div_num)
    path_table.update({'RH': list(queue)})
    queue.rotate(div_num)
    path_table.update({'LM': list(queue)})
    queue.rotate(div_num)
    path_table.update({'RM': list(queue)})
    queue.rotate(div_num)
    path_table.update({'LH': list(queue)})
    queue.rotate(div_num)
    path_table.update({'RF': list(queue)})
    return path_table

def gen_forward_Ellipse_path_table(radius, rate=0.5, div_num=20):
    path = []
    slice_length = 2 * radius / div_num
    for i in range(0, div_num+1):
        x = i * slice_length - radius
        y = 0
        z = sqrt(radius**2 - x**2)
        z = sqrt((radius / (1 - (1 - abs(x/radius)) * rate))**2 - x**2)
        path.append(point3d(x, y, z))
    for i in range(1, div_num):
        x = radius - i * slice_length
        y = 0
        z = 0
        path.append(point3d(x, y, z))
    path_r = deque(path)
    path_r.rotate(-int(div_num/2))
    R_group = list(path_r)
    path_l = deque(path)
    path_l.rotate(-3*int(div_num/2))
    L_group = list(path_l)
    path_table = {
        'name': 'highlifting_forward',
        'type': 'tip_frame_point',
        'RM': R_group,
        'LF': R_group,
        'LH': R_group,
        'LM': L_group,
        'RF': L_group,
        'RH': L_group,
        'len': len(R_group)
    }
    return path_table

def gen_forward_Ellipse_path_table_high(radius, rate=0.5, div_num=20):
    path = []
    slice_length = 2 * radius / div_num
    for i in range(0, div_num+1):
        x = i * slice_length - radius
        y = 0
        z = sqrt(radius**2 - x**2)
        z = sqrt((radius / (1 - (1 - abs(x/radius)) * rate))**2 - x**2)
        path.append(point3d(x, y, z))
    for i in range(1, div_num):
        x = radius - i * slice_length
        y = 0
        z = 0
        path.append(point3d(x, y, z))
    path_r = deque(path)
    path_r.rotate(-int(div_num/2))
    R_group = list(path_r)
    path_l = deque(path)
    path_l.rotate(-3*int(div_num/2))
    L_group = list(path_l)
    for i in range(0, len(R_group)):
        R_group[i] = R_group[i].__sub__(point3d(0, 0, 28))
    for i in range(0, len(L_group)):
        L_group[i] = L_group[i].__sub__(point3d(0, 0, 28))
    path_table = {
        'name': 'highlifting_forward_high',
        'type': 'tip_frame_point',
        'RM': R_group,
        'LF': R_group,
        'LH': R_group,
        'LM': L_group,
        'RF': L_group,
        'RH': L_group,
        'len': len(R_group)
    }
    return path_table

def gen_backward_Ellipse_path_table(radius, rate, div_num=20):
    path = []
    slice_length = 2 * radius / div_num
    for i in range(0, div_num+1):
        x = i * slice_length - radius
        y = 0
        z = sqrt(radius**2 - x**2)
        z = sqrt((radius / (1 - (1 - abs(x/radius)) * rate))**2 - x**2)
        path.append(point3d(x, y, z))
    for i in range(1, div_num):
        x = radius - i * slice_length
        y = 0
        z = 0
        path.append(point3d(x, y, z))
    path_r = deque(path)
    path_r.rotate(-int(div_num/2))
    path_r.reverse()
    R_group = list(path_r)
    path_l = deque(path)
    path_l.rotate(-3*int(div_num/2))
    path_l.reverse()
    L_group = list(path_l)
    path_table = {
        'name': 'highlifting_backward',
        'type': 'tip_frame_point',
        'RM': R_group,
        'LF': R_group,
        'LH': R_group,
        'LM': L_group,
        'RF': L_group,
        'RH': L_group,
        'len': len(R_group)
    }
    return path_table

def gen_backward_path_table(radius, div_num=20):
    path = []
    slice_length = 2 * radius / div_num
    for i in range(0, div_num+1):
        x = i * slice_length - radius
        y = 0
        z = sqrt(radius**2 - x**2)
        path.append(point3d(x, y, z))
    for i in range(1, div_num):
        x = radius - i * slice_length
        y = 0
        z = 0
        path.append(point3d(x, y, z))
    path_r = deque(path)
    path_r.rotate(-int(div_num/2))
    path_r.reverse()
    R_group = list(path_r)
    path_l = deque(path)
    path_l.rotate(-int(div_num/2)*3)
    path_l.reverse()
    L_group = list(path_l)
    path_table = {
        'name': 'normal_backward',
        'type': 'tip_frame_point',
        'RM': R_group,
        'LF': R_group,
        'LH': R_group,
        'LM': L_group,
        'RF': L_group,
        'RH': L_group,
        'len': len(L_group)
    }
    return path_table

def gen_low_down_path_table(height):
    return {
        'name': 'lowdown',
         'type': 'tip_frame_point',
         'RM': [point3d(0, 0, height)],
         'LF': [point3d(0, 0, height)],
         'LH': [point3d(0, 0, height)],
         'LM': [point3d(0, 0, height)],
         'RF': [point3d(0, 0, height)],
         'RH': [point3d(0, 0, height)],
         'len': 1
    }

def gen_lift_up_path_table(height):
    return {
        'name': 'lift_up',
         'type': 'tip_frame_point',
         'RM': [point3d(0, 0, - height)],
         'LF': [point3d(0, 0, - height)],
         'LH': [point3d(0, 0, - height)],
         'LM': [point3d(0, 0, - height)],
         'RF': [point3d(0, 0, - height)],
         'RH': [point3d(0, 0, - height)],
         'len': 1
    }

def gen_rotate_right_table(rotate_angle, localPoint, div_num=6):
    # path = []
    # slice_angle = 360.0 / div_num
    # ori_point = localPoint
    # extend_point_set = []
    # for i in range(0, 6):
    #     for iter_ in range(0, 6):
    #         extend_point_set.append(rotate(ori_point[i], -10 * iter_))
    # point = [rotateX(Point, rotate_angle) for Point in extend_point_set]
    # for i in range(0, div_num + 1):
    #     pos = deque([rotate(p, i * slice_angle) for p in point])
    #     pos.rotate(-i)
    #     new_list = list(pos)
    #     path.append([new_list[id] for id in (0, 6, 12, 18, 24, 30)])
    move_table_set = []
    move_table_set.append([rotateX(point, -rotate_angle) for point in localPoint])
    move_table_set.append([rotateX(point, rotate_angle) for point in localPoint])
    new_ = deque([rotate(rotateX(point, -rotate_angle), 60)for point in localPoint])
    new_.rotate(-1)
    move_table_set.append(list(new_))
    new_ = deque([rotate(rotateX(point, -rotate_angle), -120) for point in localPoint])
    new_.rotate(2)
    move_table_set.append(list(new_))
    new_ = deque([rotate(rotateX(point, -rotate_angle), 120)for point in localPoint])
    new_.rotate(-2)
    move_table_set.append(list(new_))
    new_ = deque([rotate(rotateX(point, -rotate_angle), -60)for point in localPoint])
    new_.rotate(1)
    move_table_set.append(list(new_))
    move_table_set.append([rotateX(point, rotate_angle) for point in localPoint])
    move_table_set.append([rotateX(point, -rotate_angle) for point in localPoint])
    new_ = deque([rotate(rotateX(point, -rotate_angle), -120) for point in localPoint])
    new_.rotate(2)
    move_table_set.append(list(new_))
    new_ = deque([rotate(rotateX(point, -rotate_angle), 60)for point in localPoint])
    new_.rotate(-1)
    move_table_set.append(list(new_))
    new_ = deque([rotate(rotateX(point, -rotate_angle), -60) for point in localPoint])
    new_.rotate(1)
    move_table_set.append(list(new_))
    new_ = deque([rotate(rotateX(point, -rotate_angle), 120) for point in localPoint])
    new_.rotate(-2)
    move_table_set.append(list(new_))
    rotate_table = {
        'name': 'rotate_right',
        'type': 'world_frame_point',
        'LF': [r_path[0] for r_path in move_table_set],
        'LM': [r_path[1] for r_path in move_table_set],
        'LH': [r_path[2] for r_path in move_table_set],
        'RH': [r_path[3] for r_path in move_table_set],
        'RM': [r_path[4] for r_path in move_table_set],
        'RF': [r_path[5] for r_path in move_table_set],
        'len': len(move_table_set)
    }
    return rotate_table

def gen_rotate_left_table(rotate_angle, localPoint, div_num=30):
    rotate_table = gen_rotate_right_table(rotate_angle, localPoint, div_num=div_num)
    for leg_index in ('LF', 'LM', 'LH', 'RF', 'RM', 'RH'):
        rotate_table[leg_index].reverse()
    rotate_table['name'] = 'rotate_left'
    return rotate_table

def gen_turn_path_left_table(turn_angle, default_leg_length, div_num=20):
    radRerDegree = pi / 180.0
    half_angle = turn_angle / 2
    Radius = RADIUS_LENGTH + default_leg_length
    angle_slice = turn_angle / div_num
    half_path_length = Radius * sin(half_angle * radRerDegree)
    path_1 = []
    for i in range(0, div_num + 1):
        ang = angle_slice * i - half_angle
        x = Radius * sin(ang * radRerDegree)
        y = Radius * cos(ang * radRerDegree) - Radius
        z = sqrt(half_path_length ** 2 - x ** 2)
        path_1.append(point3d(x, y, z))
    for i in range(1, div_num):
            ang = half_angle - angle_slice * i
            x = Radius * sin(ang * radRerDegree)
            y = Radius * cos(ang * radRerDegree) - Radius
            z = 0
            path_1.append(point3d(x, y, z))

    raw = {
        'name': 'turn_left',
        'type': 'tip_frame_point',
        'RM': path_1,
        'RH': [rotate(point, 60) for point in path_1],
        'LH': [rotate(point, 120) for point in path_1],
        'LM': [rotate(point, 180) for point in path_1],
        'LF': [rotate(point, 240) for point in path_1],
        'RF': [rotate(point, 300) for point in path_1],
        'len': len(path_1)
    }
    for index in ('RM', 'LF', 'LH'):
        li = raw[index]
        queue = deque(li)
        queue.rotate(-div_num)
        raw[index] = list(queue)
    return raw

def gen_turn_path_right_table(turn_angle, default_leg_length, div_num=20):
    turn_table = gen_turn_path_left_table(turn_angle, default_leg_length, div_num=div_num)
    for leg_index in ('LF', 'LM', 'LH', 'RF', 'RM', 'RH'):
        turn_table[leg_index].reverse()
    turn_table['name'] = 'turn_right'
    return turn_table

def gen_default_pos_table(lower_pos):
    return {
        'name': 'default_pos',
        'type': 'angle',
        'LF': [lower_pos],
        'LM': [lower_pos],
        'LH': [lower_pos],
        'RH': [lower_pos],
        'RM': [lower_pos],
        'RF': [lower_pos],
        'len': 1
    }


def gen_rolling_loop(upper_expand, upper_hold, Time=1000, div=3):
    movement_table = {
        'name': 'upper_rolling_pos',
        'type': 'angle',
    }
    pos_div=[]
    pos_div.append(upper_hold)
    for j in range(1, div+1):
        li = []
        for i in range(0, 2):
             li.append(upper_hold[i]+(upper_expand[i]-upper_hold[i])*j/div)
        pos_div.append(li)
    movement_table['ULF'] = [pos_div[3]]
    movement_table['URF'] = [[],pos_div[3], pos_div[0]]
    movement_table['URM'] = [[],[],pos_div[3], pos_div[0]]
    movement_table['URH'] = [[],[],[],pos_div[3], pos_div[0]]
    movement_table['ULH'] = [[],[],[],[],pos_div[3], pos_div[0]]
    movement_table['ULM'] = [[],[],[],[],[],pos_div[3], pos_div[0]]
    movement_table['len'] = len(movement_table['ULM'])
    return movement_table

def stage_1(tail=117, Time=1000):
    movement_table = {
        'name': 'stage_1',
        'type': 'angle',
        'len': 2
    }
    movement_table['LF'] =[[15, -5, tail], [-15, -5, tail]]
    movement_table['RH'] =[[-15, -5, tail], [15, -5, tail]]
    movement_table['LM'] =[[-15, -5, tail], [15, -5, tail]]
    movement_table['RM'] =[[15, -5, tail], [-15, -5, tail]]
    movement_table['LH'] =[[15, -5, tail], [-15, -5, tail]]
    movement_table['RF'] =[[-15, -5, tail], [15, -5, tail]]

    movement_table['ULF'] =[[120, -10], [75, -5]]
    movement_table['URH'] =[[75, -5], [120, -10]]
    movement_table['ULM'] =[[75, -5], [120, -10]]
    movement_table['URM'] =[[120, -10], [75, -5]]
    movement_table['ULH'] =[[120, -10], [75, -5]]
    movement_table['URF'] =[[75, -5], [120, -10]]
    return movement_table


def gen_step_convert(step_len, div_num=2):
    path = []

    slice_length = 2 * step_len / div_num
    for i in range(1, div_num+1):
        x = i * slice_length - step_len
        y = 0
        z = sqrt(step_len**2 - x**2)*0.4
        z = z - 30
        path.append(point3d(x, y, z))
        end_point = point3d(x, y, z)
    for i in range(0, div_num):
        path.append(end_point)
    RM_path = [rotate(point, -90) for point in path]
    RF_path = [rotate(point, -150) for point in path]
    RH_path = [rotate(point, -30) for point in path]
    LM_path = [rotate(point, 90) for point in path]
    LF_path = [rotate(point, 150) for point in path]
    LH_path = [rotate(point, 30) for point in path]

    for pa in (LM_path, RF_path, RH_path):
        for i in range(div_num, 2*div_num):
            pa[i] = point3d(0, 0, -30)

    queue = deque(LM_path)
    queue.rotate(div_num)
    LM_path = list(queue)
    queue = deque(RF_path)
    queue.rotate(div_num)
    RF_path = list(queue)
    queue = deque(RH_path)
    queue.rotate(div_num)
    RH_path = list(queue)

    return {
        'name': 'convert',
        'type': 'tip_frame_point',
        'RF': RF_path,
        'RM': RM_path,
        'RH': RH_path,
        'LF': LF_path,
        'LH': LH_path,
        'LM': LM_path,
        'len': len(path)
    }

def gen_leg_hold_up():
    movement_table = {
        'name': 'leg_hold_up',
        'type': 'angle',
    }
    movement_table['LF'] = [[0, 60, 100]]
    movement_table['RF'] = [[0, 60, 100]]
    movement_table['RM'] = [[0, 60, 100]]
    movement_table['RH'] = [[0, 60, 100]]
    movement_table['LH'] = [[0, 60, 100]]
    movement_table['LM'] = [[0, 60, 100]]
    movement_table['len'] = 1
    return movement_table

def gen_up_half_open():
    movement_table = {
        'name': 'up_half_open',
        'type': 'angle',
    }
    movement_table['ULF'] = [[90, -25], []]
    movement_table['URF'] = [[], [90, -25]]
    movement_table['URM'] = [[90, -25], []]
    movement_table['URH'] = [[], [90, -25]]
    movement_table['ULH'] = [[90, -25], []]
    movement_table['ULM'] = [[], [90, -25]]
    movement_table['len'] = 2
    return movement_table
def gen_up_hold():
    movement_table = {
        'name': 'up_hold',
        'type': 'angle',
    }
    movement_table['ULF'] = [[],[115, -25]]
    movement_table['URF'] = [[112, -25], []]
    movement_table['URM'] = [[],[115, -25]]
    movement_table['URH'] = [[112, -25], []]
    movement_table['ULH'] = [[],[115, -25]]
    movement_table['ULM'] = [[112, -25], []]
    movement_table['len'] = 2
    return movement_table
def gen_up_crawl_hold():
    movement_table = {
        'name': 'up_crawl_hold',
        'type': 'angle',
    }
    movement_table['ULF'] = [[],[90, 50]]
    movement_table['URF'] = [[90, 40], []]
    movement_table['URM'] = [[],[90, 50]]
    movement_table['URH'] = [[90, 40], []]
    movement_table['ULH'] = [[],[90, 50]]
    movement_table['ULM'] = [[90, 40], []]
    movement_table['len'] = 2
    return movement_table



if __name__ == '__main__':
    gen_rolling_loop([45,0],[90,0])