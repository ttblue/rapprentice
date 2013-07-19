import rospy
import roslib; roslib.load_manifest('ar_track_service')
from ar_track_service.srv import MarkerPositions, MarkerPositionsRequest, MarkerPositionsResponse
from ar_track_alvar.msg import AlvarMarkers
from sensor_msgs.msg import PointCloud2

class TopicListener(object):
    "stores last message"
    last_msg = None
    def __init__(self,topic_name,msg_type):
        self.sub = rospy.Subscriber(topic_name,msg_type,self.callback)        

        rospy.loginfo('waiting for the first message: %s'%topic_name)
        while self.last_msg is None: rospy.sleep(.01)
        rospy.loginfo('ok: %s'%topic_name)

    def callback(self,msg):
        self.last_msg = msg


pcL = TopicListener('/camera/depth_registered/points', PointCloud2)
getMarkers = rospy.ServiceProxy('getMarkers', MarkerPositions)

def getPose ():
    req = MarkerPositionsRequest()
    req.pc = pcL.last_msg
    return getMarkers(req)
