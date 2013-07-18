import rospy
import roslib; roslib.load_manifest('ar_track_service')
from ar_track_service.srv import MarkerPositions, MarkerPositionsRequest, MarkerPositionsResponse
from ar_track_alvar.msg import AlvarMarkers
from sensor_msgs.msg import PointCloud2

getMarkers = rospy.ServiceProxy('getMarkers', MarkerPositions)


pc = PointCloud2()
req = MarkerPositionsRequest()
req.pc = pc

print req

resp = getMarkers(req)

print resp