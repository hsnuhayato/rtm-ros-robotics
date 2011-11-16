# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('iv_plan')
import set_env
from ivutils import *
from hiro_controller import *

import rospy
from ar_pose.msg import ARMarkers
from tf.transformations import *

import operator

def encode_FRAME(f):
    return reduce(operator.__add__, f.mat) + f.vec

def decode_FRAME(ds):
    return FRAME(mat=array(ds[0:9]).reshape(3,3).tolist(), vec=ds[9:])


class MyRobotInterface(HIROController):
    def __init__(self, nameserver):
        HIROController.__init__(self, nameserver)
        self.lhand_markers = []

    def connect(self):
        HIROController.connect(self)
        rospy.init_node('pick_piece')
        rospy.Subscriber('/hiro/lhand/ar_pose_marker', ARMarkers, self.lhand_callback)
        rospy.Subscriber('/hiro/rhand/ar_pose_marker', ARMarkers, self.rhand_callback)

    def rhand_callback(self, msg):
        if len(msg.markers) > 0:
            self.rhand_markers = msg.markers

    def lhand_callback(self, msg):
        if len(msg.markers) > 0:
            self.lhand_markers = msg.markers

    def recognize(self, camera='lhand', thre=1.5):
        def parse_marker(marker):
            if rospy.Time.now().to_sec() - marker.header.stamp.to_sec() > thre:
                return None
            else:
                return [marker.id, marker.pose.pose]

        if camera == 'lhand':
            return filter(None, [parse_marker(m) for m in self.lhand_markers])
        else:
            return filter(None, [parse_marker(m) for m in self.rhand_markers])


from setup_rtchandle import *

h_ctrans = ns.rtc_handles['CoordTrans0.rtc']
h_ctrans.activate()
ctsvc = h_ctrans.services['CoordTransService'].provided['CoordTransService0']

robotframe = [1,0,0,-150, 0,1,0,0, 0,0,1,0, 0,0,0,1]

def pose2mat(pose):
    f = quaternion_matrix([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
    scale = 1000.0
    f[0:3,3] = array([pose.position.x,pose.position.y,pose.position.z]) * scale
    return f.reshape(16).tolist()


rr = MyRobotInterface(set_env.nameserver)

# rr.connect()
# pose = rr.recognize()[0][1]
# ctsvc.ref.Query('lhandcam', pose2mat(pose), robotframe, rr.get_joint_angles())
