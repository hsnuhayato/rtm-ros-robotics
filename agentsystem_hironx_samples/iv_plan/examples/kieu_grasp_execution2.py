# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('iv_plan')
from set_env import *
from demo_common import *
from record_obj_frm import *
import re

def observe_pose():
    a = [0.0,
         0.0,
         1.0471823303019583,
         -0.61085651155175535,
         0.0,
         -2.4958208303518914,
         2.2831341959228148e-05,
         0.0,
         0.0,
         0.61085651155175535,
         0.0,
         -2.4958208303518914,
         0.0,
         0.0,
         0.0,
         -0.013962633907794952,
         0.0,
         0.001745329238474369,
         -0.010471975430846214,
         0.0,
         -0.001745329238474369,
         0.0,
         -0.012217304669320583]

    r.set_joint_angles(a)


def readfile(filename):
    f = open(filename)
    m = []
    for l in f.readlines(): m.append([float(s) for s in re.split(' ', l[:-1])])
    f.close()

    return m


def read_contact(filename='/home/leus/workspace/grasp_set/contact.pcd'):
    f = open(filename)
    for i in range(10):
        f.readline()
    m = []
    for l in f.readlines(): m.append([float(s) for s in re.split(' ', l[:-1])])
    f.close()

    return m


def read_frame_from_file(filename='/home/leus/workspace/grasp_set/obj_frm.txt'):
    frm = FRAME()
    m = readfile(filename)
    frm.mat = MATRIX(m[0:3])
    frm.vec = VECTOR(vec=m[3])

    return frm


def read_hand_pose_type(filename):
    m = readfile(filename)
    for i in range(len(m)-1,1,-1):
        if (m[i]==m[i-1]==m[i-2]):
            return int (m[i][0])
            break
    return -1


def approach_vector(hand='right'):
    if (hand == 'right'):
        m = readfile('/home/leus/workspace/grasp_set_2/approach_right.txt')
    else:
        m = readfile('/home/leus/workspace/grasp_set_2/approach_left.txt')

    # for point in m:
    #     if (point != [0,0,0]):
    #         start_point_in_camera_frame = 1000*VECTOR(vec=point)
    #         break
    start_point_in_camera_frame = 1000*VECTOR(vec=m[len(m)-20])
    end_point_in_camera_frame = 1000*VECTOR(vec=m[len(m)-1])

    r.set_joint_angle(1,0)      #ロボットの首の角度
    r.set_joint_angle(2,1.047)  #ロボットの首の角度

    start_point_in_world = r.get_link('HEAD_JOINT1_Link').where()*r.Thd_kinectdepth*start_point_in_camera_frame
    end_point_in_world = r.get_link('HEAD_JOINT1_Link').where()*r.Thd_kinectdepth*end_point_in_camera_frame

    vector = start_point_in_world + (-1)*end_point_in_world

    return start_point_in_world, end_point_in_world, vector


def approach_direction(hand='right'):
    s, e, v = approach_vector(hand)

    angle_with_x = arccos(abs(dot(v,VECTOR(vec=[1,0,0])) / sqrt(dot(v,v))))
    angle_with_y = arccos(abs(dot(v,VECTOR(vec=[0,1,0])) / sqrt(dot(v,v))))
    angle_with_z = arccos(abs(dot(v,VECTOR(vec=[0,0,1])) / sqrt(dot(v,v))))

    if ( (angle_with_x <= angle_with_y) and (angle_with_x <= angle_with_z) ):
        direction = 'x'
    if ( (angle_with_y <= angle_with_z) and (angle_with_y <= angle_with_x) ):
        direction = 'y'
    if ( (angle_with_z <= angle_with_x) and (angle_with_z <= angle_with_y) ):
        direction = 'z'

    return direction


def pre_grasp_type(hand='right'):
    if (hand == 'right'):
        index = read_hand_pose_type('/home/leus/workspace/grasp_set_2/pre_grasp_type_right.txt')
    else:
        index = read_hand_pose_type('/home/leus/workspace/grasp_set_2/pre_grasp_type_left.txt')

    if (index == 0 or index == 1 or index == 5): return 1
    else: return 2


def grasp_type(hand='right'):
    if (hand == 'right'):
        index = read_hand_pose_type('/home/leus/workspace/grasp_set_2/grasp_type_right.txt')
    else:
        index = read_hand_pose_type('/home/leus/workspace/grasp_set_2/grasp_type_left.txt')

    if (index == 0): return 1
    if (index == 1): return 2
    if (index == 2 or index == 3): return 3
    if (index == 4 or index == 5): return 4
    if (index == 6): return 6


def contact_points(hand='right'):
    if (hand == 'right'):
        m = read_contact('/home/leus/workspace/grasp_set_2/contact_right.pcd')
    else:
        m = read_contact('/home/leus/workspace/grasp_set_2/contact_left.pcd')

    vector_list_in_cam = []
    for point in m:
        vector_list_in_cam.append(1000*VECTOR(vec=point))

    r.set_joint_angle(1,0)      #ロボットの首の角度
    r.set_joint_angle(2,1.047)  #ロボットの首の角度

    vector_list_in_world = []
    for vector_in_cam in vector_list_in_cam:
        vector_in_world = r.get_link('HEAD_JOINT1_Link').where()*r.Thd_kinectdepth*vector_in_cam
        vector_list_in_world.append(vector_in_world)

    return vector_list_in_world

def grasp_point(hand='right'):
    vector_list_in_world = contact_points(hand)

    vector_sum = VECTOR(vec=[0,0,0])
    for vector_in_world in vector_list_in_world:
        vector_sum = vector_sum + VECTOR(vec=vector_in_world)

    res = ( 1.0/ len(vector_list_in_world) )*vector_sum

    return res


def vec_dist(input1, input2):
    return sqrt( (input1[0] - input2[0])*(input1[0] - input2[0]) +
                 (input1[1] - input2[1])*(input1[1] - input2[1]) +
                 (input1[2] - input2[2])*(input1[2] - input2[2]) )


def closest_dist(input_vector, target_vector_list):
    tmp_min = vec_dist(input_vector, target_vector_list[0])
    for vector in target_vector_list:
        if ( vec_dist(input_vector, vector) <= tmp_min):
            tmp_min = vec_dist(input_vector, vector)

    return tmp_min

def palm_contact(hand='right'):
    vector_list_in_world = contact_points(hand)
    if ( closest_dist(grasp_point(), vector_list_in_world) <= 2 ): return True
    else: return False


def min_max_x(hand='right'):
    vector_list_in_world=contact_points(hand)
    min_x = max_x = vector_list_in_world[0][0]
    for vector_in_world in vector_list_in_world:
        if (vector_in_world[0] > max_x): max_x = vector_in_world[0]
        if (vector_in_world[0] < min_x): min_x = vector_in_world[0]

    return min_x, max_x


def min_max_y(hand='right'):
    vector_list_in_world=contact_points(hand)
    min_y = max_y = vector_list_in_world[0][1]
    for vector_in_world in vector_list_in_world:
        if (vector_in_world[1] > max_y): max_y = vector_in_world[1]
        if (vector_in_world[1] < min_y): min_y = vector_in_world[1]

    return min_y, max_y


def min_max_z(hand='right'):
    vector_list_in_world=contact_points()
    min_z = max_z = vector_list_in_world[0][2]
    for vector_in_world in vector_list_in_world:
        if (vector_in_world[2] > max_z): max_z = vector_in_world[2]
        if (vector_in_world[2] < min_z): min_z = vector_in_world[2]

    return min_z, max_z


def approach_frame(hand='right'):
    # obj_frm_learned_in_world = read_frame_from_file()
    # obj_frm_now_in_world = detect_AR_tags_with_kinect()[0][1]
    if (hand == 'right'):
        s, e, v = approach_vector(hand)
        direction = approach_direction(hand)
        pre_grasp = pre_grasp_type(hand)

        tmp = e
        approach = FRAME()
        approach.vec = tmp
        # approach.vec = obj_frm_now_in_world*(-obj_frm_learned_in_world)*tmp

        if (direction == 'x' and pre_grasp == 1):
            approach.mat = MATRIX(c=pi) *MATRIX(b=-pi)
            approach.vec[0]+=50
        if (direction == 'x' and pre_grasp == 2):
            approach.mat = MATRIX(c=pi) *MATRIX(a=pi/2)*MATRIX(b=-pi)
            approach.vec[0]+=50
        if (direction == 'y' and pre_grasp == 1):
            approach.mat = MATRIX(c=pi) *MATRIX(c=pi/2)*MATRIX(b=-pi)
            approach.vec[1]+=50
        if (direction == 'y' and pre_grasp == 2):
            approach.mat = MATRIX(c=pi) *MATRIX(b=pi/2)*MATRIX(c=pi/2)*MATRIX(b=-pi)
            approach.vec[1]+=50
        if (direction == 'z' and pre_grasp == 1):
            approach.mat = MATRIX(c=pi) *MATRIX(b=-pi/2)
            approach.vec[2]+=50
        if (direction == 'z' and pre_grasp == 2):
            approach.mat = MATRIX(c=pi) *MATRIX(c=pi/2)*MATRIX(b=-pi/2)
            approach.vec[2]+=50

    else:
        s, e, v = approach_vector(hand)
        direction = approach_direction(hand)
        pre_grasp = pre_grasp_type('right')

        tmp = e
        approach = FRAME()
        approach.vec = tmp
        # approach.vec = obj_frm_now_in_world*(-obj_frm_learned_in_world)*tmp

        if (direction == 'x' and pre_grasp == 1):
            approach.mat = MATRIX(c=pi) *MATRIX(b=-pi)
            approach.vec[0]+=50
        if (direction == 'x' and pre_grasp == 2):
            approach.mat = MATRIX(c=pi) *MATRIX(a=-pi/2)*MATRIX(b=-pi)
            approach.vec[0]+=50
        if (direction == 'y' and pre_grasp == 1):
            approach.mat = MATRIX(c=pi) *MATRIX(a=pi)*MATRIX(c=pi/2)*MATRIX(b=-pi)
            approach.vec[1]-=50
        if (direction == 'y' and pre_grasp == 2):
            approach.mat = MATRIX(c=pi) *MATRIX(b=-pi/2)*MATRIX(a=pi)*MATRIX(c=pi/2)*MATRIX(b=-pi)
            approach.vec[1]-=50
        if (direction == 'z' and pre_grasp == 1):
            approach.mat = MATRIX(c=pi) *MATRIX(b=-pi/2)
            approach.vec[2]+=50
        if (direction == 'z' and pre_grasp == 2):
            approach.mat = MATRIX(c=pi) *MATRIX(c=-pi/2)*MATRIX(b=-pi/2)
            approach.vec[2]+=50

    return approach


def grasp_plan(hand='right'):
    # obj_frm_learned_in_world = read_frame_from_file()
    # obj_frm_now_in_world = detect_AR_tags_with_kinect()[0][1]

    # grasp_point_now = obj_frm_now_in_world*(-obj_frm_learned_in_world)*grasp_point()
    grasp_point_now = grasp_point(hand)
    target_frame = FRAME()
    target_frame.mat = approach_frame(hand).mat;
    target_frame.vec = grasp_point_now

    if ( grasp_type() == 1 ):
        if ( approach_direction() == 'y' and pre_grasp_type() == 1 ):
            target_frame.vec[0] = grasp_point_now[0] + 25
            if (hand == 'right'):
                target_frame.vec[1] = grasp_point_now[1] + 35
            else:
                target_frame.vec[1] = grasp_point_now[1] - 35
            grasp_width = 50
        elif ( approach_direction() == 'y' and pre_grasp_type() == 2 ):
            target_frame.vec[2] = grasp_point_now[2] - 25
            if (hand == 'right'):
                target_frame.vec[1] = grasp_point_now[1] + 35
            else:
                target_frame.vec[1] = grasp_point_now[1] - 35
            grasp_width = 50
        elif ( approach_direction() == 'z' and pre_grasp_type() == 2 ):
            target_frame.vec[0] = grasp_point_now[0] + 25
            target_frame.vec[2] = grasp_point_now[2] + 90
            grasp_width = 50
        elif ( approach_direction() == 'x' and pre_grasp_type() == 2 ):
            target_frame.vec[0] = grasp_point_now[0] + 35
            target_frame.vec[2] = grasp_point_now[2] - 25
            grasp_width = 50
        else:
            if ( approach_direction() == 'x' and pre_grasp_type() == 1 ):
                if ( palm_contact() == True ):
                    a, b = min_max_y()
                    if ( (b - a - 20)>0 ): grasp_width = b - a - 20
                    else: grasp_width = 0
                    target_frame.vec[0] = grasp_point_now[0] + 35
                else:
                    grasp_width = 2*closest_dist( grasp_point(), contact_points() ) + 20
                    target_frame.vec[0] = grasp_point_now[0] + 50
            if ( approach_direction() == 'z' and pre_grasp_type() == 1 ):
                if ( palm_contact() == True ):
                    a, b = min_max_y()
                    if ( (b - a - 20)>0 ): grasp_width = b - a - 20
                    else: grasp_width = 0
                    target_frame.vec[2] = grasp_point_now[2] + 90
                else:
                    grasp_width = 2*closest_dist( grasp_point(), contact_points() ) + 20
                    target_frame.vec[2] = grasp_point_now[2] + 90


    if ( grasp_type() == 2 or grasp_type() == 3 ):
        if ( approach_direction() == 'x' ):
            target_frame.vec[0] = grasp_point_now[0] + 50
        if ( approach_direction() == 'y' ):
            if (hand == 'right'):
                target_frame.vec[1] = grasp_point_now[1] + 60
            else:
                target_frame.vec[1] = grasp_point_now[1] - 60
        if ( approach_direction() == 'z' ):
            target_frame.vec[2] = grasp_point_now[2] + 90

        grasp_width = 0


    if ( grasp_type() == 4 or grasp_type() == 5):
        if ( approach_direction() == 'x' ):
            target_frame.vec[0] = grasp_point_now[0] + 50
            if ( pre_grasp_type() == 1):
                a, b = min_max_y()
            if ( pre_grasp_type() == 2):
                a, b = min_max_z()
        if ( approach_direction() == 'y' ):
            if (hand == 'right'):
                target_frame.vec[1] = grasp_point_now[1] + 50
            else:
                target_frame.vec[1] = grasp_point_now[1] - 50
            if ( pre_grasp_type() == 1):
                a, b = min_max_x()
            if ( pre_grasp_type() == 2):
                a, b = min_max_z()
        if ( approach_direction() == 'z' ):
            target_frame.vec[2] = grasp_point_now[2] + 70
            if ( pre_grasp_type() == 1):
                a, b = min_max_y()
            if ( pre_grasp_type() == 2):
                a, b = min_max_x()

        if ( (b - a - 25)>0 ): grasp_width = b - a - 25
        else: grasp_width = 0

    if ( grasp_type() == 6 or grasp_type() == 7 ):
        if ( approach_direction() == 'x' ):
            target_frame.vec[0] = grasp_point_now[0] + 70
        if ( approach_direction() == 'y' ):
            if (hand == 'right'):
                target_frame.vec[1] = grasp_point_now[1] + 70
            else:
                target_frame.vec[1] = grasp_point_now[1] - 70
        if ( approach_direction() == 'z' ):
            target_frame.vec[2] = grasp_point_now[2] + 80

        if ( (2*closest_dist(grasp_point(), contact_points()) - 25) > 0): grasp_width = 2*closest_dist(grasp_point(), contact_points()) - 25
        else: grasp_width = 0

    return target_frame, grasp_width

def grasp_execute_two_hands(real_robot=False, time_mode=False):
    obj_frm_learned_in_world = read_frame_from_file('/home/leus/workspace/grasp_set_2/obj_frm.txt')
    obj_frm_now_in_world = detect_AR_tags_with_kinect()[0][1]

    Tr = obj_frm_now_in_world*(-obj_frm_learned_in_world)

    left_approach_frame_learned_in_world = approach_frame('left')
    left_approach_frame_now_in_world = Tr*left_approach_frame_learned_in_world

    left_target_frame_learned_in_world, left_grasp_width = grasp_plan('left')
    left_target_frame_now_in_world = Tr*left_target_frame_learned_in_world

    right_approach_frame_learned_in_world = approach_frame('right')
    right_approach_frame_now_in_world = Tr*right_approach_frame_learned_in_world

    right_target_frame_learned_in_world, right_grasp_width = grasp_plan('right')
    right_target_frame_now_in_world = Tr*right_target_frame_learned_in_world

    show_frame(left_target_frame_now_in_world, name='left_target frame')
    show_frame(left_approach_frame_now_in_world, name='left_approach frame')

    show_frame(right_target_frame_now_in_world, name='right_target frame')
    show_frame(right_approach_frame_now_in_world, name='right_approach frame')

    # Execution simultaneously
    # r_v1 = r.ik(right_approach_frame_now_in_world,joints='rarm')[0]
    # r.set_joint_angles(r_v, joints='rarm')

    # l_v1 = r.ik(left_approach_frame_now_in_world,joints='larm')[0]
    # r.set_joint_angles(l_v, joints='larm')

    # time.sleep(1.0)

    # if (real_robot): sync()

    # r.grasp(width=100, hand='right')

    # r.grasp(width=100, hand='left')

    # time.sleep(1.0)

    # if (real_robot): sync()

    # r_v2 = r.ik(right_target_frame_now_in_world, joints='rarm')[0]
    # r.set_joint_angles(r_v2, joints='rarm')

    # l_v2 = r.ik(left_target_frame_now_in_world, joints='larm')[0]
    # r.set_joint_angles(l_v2, joints='larm')

    # time.sleep(1.0)

    # if (real_robot): sync()

    # r.grasp(width=right_grasp_width, hand='right')
    # r.grasp(width=left_grasp_width, hand='left')

    # time.sleep(1.0)

    # if (real_robot): sync()

    # r_fp = r.fk(arm='right')
    # r_fp.vec[2] += 100
    # r.set_arm_joint_angles(r.ik(r_fp,joints='rarm')[0], arm='right')
    # r1 = r.ik(r_fp,joints='rarm')[0]

    # l_fp = r.fk(arm='left')
    # l_fp.vec[2] += 100
    # l1 = r.ik(l_fp,joints='larm')[0]

    # if (real_robot): sync()

    r_v1 = r.ik(right_approach_frame_now_in_world,joints='rarm')[0]
    l_v1 = r.ik(left_approach_frame_now_in_world,joints='larm')[0]

    r_v2 = r.ik(right_target_frame_now_in_world, joints='rarm')[0]
    l_v2 = r.ik(left_target_frame_now_in_world, joints='larm')[0]

    r.set_joint_angles(r_v1, joints='rarm')
    r.set_joint_angles(l_v1, joints='larm')

    time.sleep(0.5)

    if (real_robot): sync()

    r.grasp(width=100, hand='right')
    r.grasp(width=100, hand='left')

    time.sleep(0.5)

    if (real_robot): sync()

    r.set_joint_angles(r_v2, joints='rarm')
    r.set_joint_angles(l_v2, joints='larm')

    time.sleep(0.5)

    if (real_robot): sync()

    r.grasp(width=right_grasp_width, hand='right')
    r.grasp(width=left_grasp_width, hand='left')

    time.sleep(0.5)

    if (real_robot): sync()

    r_fp = r.fk(arm='right')
    r_fp.vec[2] += 100
    r1 = r.ik(r_fp,joints='rarm')[0]

    l_fp = r.fk(arm='left')
    l_fp.vec[2] += 100
    l1 = r.ik(l_fp,joints='larm')[0]

    r.set_arm_joint_angles(r1, arm='right')
    r.set_arm_joint_angles(l1, arm='left')

    if (real_robot): sync()

    # if (real_robot): sync(joints='larm')

    # if (real_robot): sync(joints='rarm')

def grasp_execute(hand='right',real_robot=False):

    obj_frm_learned_in_world = read_frame_from_file()
    obj_frm_now_in_world = detect_AR_tags_with_kinect()[0][1]

    approach_frame_learned_in_world = approach_frame()
    approach_frame_now_in_world = obj_frm_now_in_world*(-obj_frm_learned_in_world)*approach_frame_learned_in_world

    target_frame_learned_in_world, grasp_width = grasp_plan()
    target_frame_now_in_world = obj_frm_now_in_world*(-obj_frm_learned_in_world)*target_frame_learned_in_world

    # target_frame_now_in_world.vec[2] -= 10 # mug
    # target_frame_now_in_world.vec[2] -= 10 # pen
    # target_frame_now_in_world.vec[2] -= 42 # tape
    # target_frame_now_in_world.vec[2] -= 18 # measure
    # target_frame_now_in_world.vec[2] -= 8 # remote
    # target_frame_now_in_world.vec[2] -= 15 # box
    # approach_frame_now_in_world = approach_frame()
    # target_frame_now_in_world, grasp_width = grasp_plan()

    show_frame(target_frame_now_in_world, name='target frame')
    show_frame(approach_frame_now_in_world, name='approach frame')

    v = r.ik(approach_frame_now_in_world,joints='torso_rarm')[0]
    r.set_joint_angles(v, joints='torso_rarm')
    if (grasp_type() != 2): r.grasp(width=100, hand='right')
    else: r.grasp(width=40, hand='right')
    if (real_robot): sync()


    v = r.ik(target_frame_now_in_world, joints='torso_rarm')[0]
    r.set_joint_angles(v, joints='torso_rarm')
    if (real_robot): sync()
    r.grasp(width=grasp_width, hand='right')
    if (real_robot): sync()

    fp = r.fk(arm='right')
    fp.vec[2] += 100
    # fp.mat = MATRIX(b=-pi/8)*fp.mat
    r.set_arm_joint_angles(r.ik(fp,joints='rarm')[0], arm='right')
    if (real_robot): sync(joints='rarm')




    # v = r.ik(approach_frame_now_in_world,joints='torso_larm')[0]
    # r.set_joint_angles(v, joints='torso_larm')
    # r.grasp(width=100, hand='left')
    # if (real_robot): sync()


    # v = r.ik(target_frame_now_in_world, joints='torso_larm')[0]
    # r.set_joint_angles(v, joints='torso_larm')
    # if (real_robot): sync()
    # r.grasp(width=grasp_width, hand='left')
    # if (real_robot): sync()

    # fp = r.fk(arm='left')
    # fp.vec[2] += 100
    # r.set_arm_joint_angles(r.ik(fp,joints='larm')[0], arm='left')
    # if (real_robot): sync(joints='larm')
