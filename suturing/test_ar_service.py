#!/usr/bin/ipython -i

import rospy
import roslib; roslib.load_manifest('ar_track_service')
from ar_track_service.srv import MarkerPositions, MarkerPositionsRequest, MarkerPositionsResponse
from ar_track_alvar.msg import AlvarMarkers
from sensor_msgs.msg import PointCloud2
import cloudprocpy
import numpy as np
import tf

from rapprentice import ros_utils as ru, clouds, berkeley_pr2, conversions as conv

import subprocess
subprocess.call("killall XnSensorServer", shell=True)

rospy.init_node('test_ar_service')

rospy.wait_for_service('getMarkers')
getMarkers = rospy.ServiceProxy('getMarkers', MarkerPositions)
grabber = cloudprocpy.CloudGrabber()
grabber.startRGBD()

def getPC ():
    r, d = grabber.getRGBD()
    x = clouds.depth_to_xyz(d, berkeley_pr2.f)
    return ru.xyzrgb2pc(x, r, 'map')


def getPoses ():
    req = MarkerPositionsRequest()
    req.pc = getPC()
    return getMarkers(req)

def testTransforms (tfms, child_frames, parent_frames, time = 25):
    """
    Basic code to test transforms. Visualize on RViz or something.
    """
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(0.2)
    
    initTime  = rospy.Time.now()
    totTime = rospy.Duration(time)

    if not time:
        foreverCheck = True
    else:
        foreverCheck = False
    
    try:
        while foreverCheck or (rospy.Time.now() - initTime < totTime):
            for tfm, child_frame, parent_frame in zip(tfms, child_frames,parent_frames):
                (trans, rot) = conv.hmat_to_trans_rot(tfm)
                br.sendTransform(trans, rot,
                                 rospy.Time.now(),
                                 child_frame,
                                 parent_frame)
                rate.sleep()
    except KeyboardInterrupt:
        return

def displayMarkerTfm ():
    # Get response from service
    markers = getPoses()
    ids = [marker.id for marker in markers.markers.markers]
    poses = [marker.pose.pose for marker in markers.markers.markers]
    tfms = [conv.pose_to_hmat(pose) for pose in poses]
    
    parent_frames = ['map' for _ in ids]
    child_frames = ['marker_%d'%id for id in ids]
    
    testTransforms(tfms, child_frames, parent_frames)