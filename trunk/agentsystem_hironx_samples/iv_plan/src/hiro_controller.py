# -*- coding: utf-8 -*-

import socket
import pickle
import operator

# from set_env import *
from numpy import *
from ivutils import *
from setup_rtchandle import *


class HIROController:
    def __init__(self, nameserver, seq_proxy_host='VisionPC', seq_proxy_port=10103):
        self.seq_proxy_address = socket.gethostbyname(seq_proxy_host)
        self.seq_proxy_port = seq_proxy_port
        self.joint_states = zeros(23)

    def connect(self):
        h_rh = get_handle('RobotHardware0.rtc', ns)
        self.jstt_port = h_rh.outports['jointDatOut']
        # h_rh = get_handle('HIRONXController(Robot)0.rtc', ns)
        # self.jstt_port = h_rh.outports['q']
        # rospy.Subscriber('/hiro/joint_state', JointState, self.update_joint_state)

    def read_joint_state(self):
        data = self.jstt_port.read()
        # not yet used
        # secs = data.tm.sec
        # nsecs = data.tm.nsec
        self.joint_states = reduce(operator.__concat__, data.qState)
        # self.joint_states = data.data
        # velocity = reduce(operator.__concat__, data.dqState)

    # def update_joint_state(self, joint_state_msg):
    #     self.joint_states = joint_state_msg.position

    def get_joint_angles(self):
        self.read_joint_state()
        return self.joint_states

    def send_goal(self, joint_angless, duration, wait=True):
        # convert from numpy.float64 to float
        goal = map(lambda joint_angles: map(lambda x: float(x), joint_angles), joint_angless)
        msg = ('goal', goal, duration, wait)
        self.send_msg(msg)

    def send_trajectory(self, ps, duration=2.0, arm='right'):
        traj = []
        s = 0.001
        for p in ps:
            x,y,z = p.vec
            a,b,c = euler_from_matrix(p.mat, axes='sxyz')
            # p.mat.abc() = euler_from_matrix(p.mat, axes='szyx')
            traj.append(([s*x,s*y,s*z,a,b,c], duration))
        msg = ('trajectory', traj)
        self.send_msg(msg, timeout=30.0)

    def send_msg(self, msg, timeout = 10.0):
        soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if timeout > 0.0:
            soc.settimeout(timeout)
        soc.connect((self.seq_proxy_address, self.seq_proxy_port))
        soc.sendall(pickle.dumps(msg))
        msgstr = soc.recv(4096)
        msg = pickle.loads(msgstr)
        print msg
        soc.close()
