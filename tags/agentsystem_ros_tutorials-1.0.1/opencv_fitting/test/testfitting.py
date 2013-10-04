import roslib; roslib.load_manifest('opencv_fitting')
import rospy
import nose
from geometry_msgs.msg import Pose2D
from image_msgs.msg import Image
import cv
from cv_bridge import CvBridge

class TestImageResults(object):
    def setup(self):
        self.realpose = None
        self.received = False
        rospy.init_node('testfitting', anonymous=True)
        self.subellipse = rospy.Subscriber("ellipse", Pose2D, self.posecb)
        self.pub = rospy.Publisher('image', Image)
        self.bridge = CvBridge()

    def teardown(self):
        self.subellipse.unregister()

    def posecb(self,msg):
        assert( abs(msg.x - self.received.x) <= 0.5)
        assert( abs(msg.y - self.received.y) <= 0.5)
        assert( abs(msg.theta - self.received.theta) <= 0.2)
        self.received = True

    def test_sendimage(self):
        self.realpose = Pose2D(100,100,0.5)
        I = cv.CreateImage([256,256],cv.IPL_DEPTH_8U,1)
        cv.Ellipse(I,center=(self.realpose.x,self.realpose.y),axes=(100,50),angle=self.realpose.theta,start_angle=0,end_angle=6.28,color=(0,0,0))
        self.pub.publish(bridge.cv_to_imgmsg(I))
        # wait for results


