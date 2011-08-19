##
## real_hiro.py
##
## R.Hanai 2011.04.02 -
##

import socket
import pickle
import operator

## ROS
import roslib; roslib.load_manifest('iv_plan')
import rospy
import tf
from sensor_msgs.msg import JointState
from ar_pose.msg import ARMarkers
import geometry_msgs.msg
from tf.transformations import *
##


from numpy import *
from utils import *

## RTM
from rtc_handle import *
import RTC
import _GlobalIDL
import rospy
try:
    nameserver = rospy.get_param('hiro/nameserver')
    env = RtmEnv(sys.argv, [nameserver])
    ns = env.name_space[nameserver]
    ns.list_obj()
except:
    warn('HIRO-NX base system is not running, or ROS_MASTER_URI is not set correctly, maybe ...')
##


def get_handle(name, nspace):
    return nspace.rtc_handles[name]

def narrow_service(handle, service, port):
    svc = handle.services[service].provided[port]
    svc.narrow_ref(globals())
    return svc

def activate(handles):
    for hdl in handles:
        hdl.activate()

def deactivate(handles):
    for hdl in handles:
        hdl.deactivate()

def connect(handle1, port1, handle2, port2):
    con = IOConnector([handle1.outports[port1], handle2.inports[port2]])
    con.connect()

def disconnect(con):
    con.disconnect()


# global h_seq, seq_svc, jstt_port, mysvc, h_my
# h_seq = get_handle('seq.rtc', ns)
# seq_svc = narrow_service(h_seq, 'SequencePlayerService', 'service0')
# h_my = get_handle('MyServiceProvider0.rtc', ns)
# mysvc = narrow_service(h_my, 'MyService', 'myservice0')



class RealHIRO:
    def __init__(self):
        self.joint_states = zeros(23)
        self.rhand_pose_markers = []
        self.lhand_pose_markers = []
        self.kinect_centers = []
        self.kinect_pose_markers = []

    def connect(self):
        rospy.init_node('motion_planner')

        # rospy.Subscriber('/hiro/joint_state', JointState, self.update_joint_state)
        self.listener = tf.TransformListener()

        h_rh = get_handle('RobotHardware0.rtc', ns)
        self.jstt_port = h_rh.outports['jointStt']

        # h_rh = get_handle('HIRONXController(Robot)0.rtc', ns)
        # self.jstt_port = h_rh.outports['q']

        # self.h_leyecap = get_handle('leye_capture.rtc', ns)
        # activate([self.h_leyecap])

        #self.h_reyecap = get_handle('reye_capture.rtc', ns)
        #self.h_rhandcap = get_handle('rhand_capture.rtc', ns)
        #activate([self.h_leyecap, self.h_reyecap])

        rospy.Subscriber('/hiro/rhand/ar_pose_marker', ARMarkers, self.update_rhand_cam)
        rospy.Subscriber('/hiro/lhand/ar_pose_marker', ARMarkers, self.update_lhand_cam)
        rospy.Subscriber('/calc_center', geometry_msgs.msg.PoseStamped, self.update_calc_center)
        rospy.Subscriber('/ar_pose_marker', ARMarkers, self.update_kinect_AR)
    
    def __del__(self):
        #deactivate([self.h_leyecap, self.h_reyecap])
        deactivate([self.h_leyecap])

    def update_joint_state(self, joint_state_msg):
        self.joint_states = joint_state_msg.position

    def update_rhand_cam(self, msg):
        if len(msg.markers) > 0:
            self.rhand_pose_markers = msg.markers
        #     marker = msg.markers[0]
        #     print 'stamp ', 
        #     print marker.header.stamp
        #     print 'frame_id ',
        #     print marker.header.frame_id
        #     print 'id ',
        #     print marker.id
        #     print 'position ',
        #     print marker.pose.pose.position
        #     print 'orientation ',
        #     print marker.pose.pose.orientation

    def update_lhand_cam(self, msg):
        if len(msg.markers) > 0:
            self.lhand_pose_markers = msg.markers

    def update_kinect_AR(self, msg):
        if len(msg.markers) > 0:
            self.kinect_pose_markers = msg.markers

    def update_calc_center(self, msg):
        def near(p1,p2,eps=0.02):
            return linalg.norm([p1.x-p2.x,p1.y-p2.y,p1.z-p2.z]) < eps
        # point = msg.pose.position
        # orientation = msg.pose.orientation
        # header = msg.header

        # append if the new observation is different
        # from any of registered ones.
        for cs in self.kinect_centers:
            if near(msg.pose.position, cs.pose.position):
                return

        self.kinect_centers.append(msg)

        # # discard old observations
        # tnow = rospy.Time.now().to_sec()
        # for cs in self.kinect_centers:
        #     if tnow - .stamp.to_sec() > thre:

    def read_joint_state(self):
        data = self.jstt_port.read()
        # not yet used
        # secs = data.tm.sec
        # nsecs = data.tm.nsec
        self.joint_states = reduce(operator.__concat__, data.qState)
        # self.joint_states = data.data
        # velocity = reduce(operator.__concat__, data.dqState)

    def get_joint_angles(self):
        self.read_joint_state() # for RT-middle
        return self.joint_states

    def get_tf(self, from_frm='/leye', to_frm='/checkerboard'):
        tfs = {}
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform(from_frm, to_frm,
                                           now, rospy.Duration(3.0))
            return self.listener.lookupTransform(from_frm, to_frm, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.Exception):
            return None

    def detect(self, wait=True, camera='leye', thre=1.5):
        '''camera := "leye" | "rhand" | "lhand", returns the recent recognition result within thre[sec] for hand cameras'''
        if camera == 'leye':
            rate = rospy.Rate(2.0)
            if wait:
                while not rospy.is_shutdown():
                    tf = self.get_tf()
                    # if tfs.has_key(camera):
                    #     tf = tfs[camera]
                    if not tf == None:
                        break
                    rate.sleep()
            else:
                tf = self.get_tf('/leye', '/checkerboard')
                if tf == None:
                    return None

            (trans, rot) = tf
            Tcam_obj = FRAME(mat=MATRIX(mat=quaternion_matrix(rot)[0:3,0:3].tolist()),
                             vec=VECTOR(vec=(1000.0*array(trans)).tolist()))
            return Tcam_obj

        elif camera == 'kinect_rgb':
            rate = rospy.Rate(2.0)
            if wait:
                while not rospy.is_shutdown():
                    tf = self.get_tf('/openni_rgb_optical_frame', '/checkerboard_k')
                    if not tf == None:
                        break
                    rate.sleep()
            else:
                tf = self.get_tf('/openni_rgb_optical_frame', '/checkerboard_k')
                if tf == None:
                    return None

            (trans, rot) = tf
            Tcam_obj = FRAME(mat=MATRIX(mat=quaternion_matrix(rot)[0:3,0:3].tolist()),
                             vec=VECTOR(vec=(1000.0*array(trans)).tolist()))
            return Tcam_obj

        elif camera == 'kinect_point_center':
            tnow = rospy.Time.now().to_sec()
            for cs in self.kinect_centers:
                if tnow - cs.header.stamp.to_sec() > thre:
                    self.kinect_centers.remove(cs)

            return [FRAME(mat=MATRIX(mat=quaternion_matrix([0,0,0,0])[0:3,0:3].tolist()),
                          vec=VECTOR(1000.0*cs.pose.position.x,1000.0*cs.pose.position.y,1000.0*cs.pose.position.z)) for cs in self.kinect_centers]

            # p = self.kinect_center_point
            # trans = 1000.0 * array([p.x, p.y, p.z])
            # rot = [0,0,0,0]


        elif camera == 'rhand' or camera == 'lhand':
            def parse_marker(marker):
                if rospy.Time.now().to_sec() - marker.header.stamp.to_sec() > thre:
                    return None
                else:
                    p = marker.pose.pose.position
                    trans = 1000.0 * array([p.x, p.y, p.z])
                    q = marker.pose.pose.orientation
                    rot = [q.x, q.y, q.z, q.w]

                    return (#marker.header.frame_id,
                            marker.id,
                            FRAME(mat=MATRIX(mat=quaternion_matrix(rot)[0:3,0:3].tolist()),
                                  vec=VECTOR(vec=(trans.tolist()))))

            if camera == 'rhand':
                return filter(None, [parse_marker(m) for m in self.rhand_pose_markers])
            else:
                return filter(None, [parse_marker(m) for m in self.lhand_pose_markers])

        elif camera == 'kinect_AR':
            def parse_marker(marker):
                if rospy.Time.now().to_sec() - marker.header.stamp.to_sec() > thre:
                    return None
                else:
                    p = marker.pose.pose.position
                    trans = 1000.0 * array([p.x, p.y, p.z])
                    q = marker.pose.pose.orientation
                    rot = [q.x, q.y, q.z, q.w]

                    return (#marker.header.frame_id,
                            marker.id,
                            FRAME(mat=MATRIX(mat=quaternion_matrix(rot)[0:3,0:3].tolist()),
                                  vec=VECTOR(vec=(trans.tolist()))))

            return filter(None, [parse_marker(m) for m in self.kinect_pose_markers])

        else:
            print 'specified camera is not supported'

    # def send_head_goal(self, joint_angles):
    #     js = list(self.joint_states)
    #     js[0:3] = joint_angles[0:3]
    #     self.send_goal(js)

    # def send_rarm_goal(self, joint_angles):
    #     js = list(self.joint_states)
    #     js[3:9] = joint_angles[0:6]
    #     self.send_goal(js)

    # def send_rhand_goal(self, joint_angles):
    #     js = list(self.joint_states)
    #     js[15:19] = joint_angles[0:4]
    #     self.send_goal(js)

    def send_goal(self, joint_angless, duration, wait=True):
        # need to convert from numpy.float64 to float
        goal = map(lambda joint_angles: map(lambda x: float(x), joint_angles), joint_angless)
        msg = ('goal', goal, duration, wait)
        self.send_msg(msg)

    def send_trajectory(self, ps, duration=2.0, arm='right'):
        traj = []
        s = 0.001
        for p in ps:
            x,y,z = p.vec
            a,b,c = euler_from_matrix(p.mat, axes='sxyz')
            # p.mat.abc() = eusler_from_matrix(p.mat, axes='szyx')
            traj.append(([s*x,s*y,s*z,a,b,c], duration))
        msg = ('trajectory', traj)
        self.send_msg(msg, timeout=30.0)

    def send_msg(self, msg, host='192.168.128.253', port = 10103, timeout = 10.0):
        soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if timeout > 0.0:
            soc.settimeout(timeout)
        soc.connect((host,port))
        soc.sendall(pickle.dumps(msg))
        msgstr = soc.recv(4096)
        msg = pickle.loads(msgstr)
        print msg
        soc.close()

# seq.waitInterpolation()
# seq.setJointAngles(jvs, tm)
# seq.setJointAngle(jname, jv, tm)
# seq.isEmpty()
# seq.clear()
# seq.clearNoWait()
