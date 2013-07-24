#!/usr/bin/ipython -i

import rospy
import roslib; roslib.load_manifest('ar_track_service')
from ar_track_service.srv import MarkerPositions, MarkerPositionsRequest, MarkerPositionsResponse
from ar_track_alvar.msg import AlvarMarkers
from sensor_msgs.msg import PointCloud2
import cloudprocpy
import tf

import threading as th


from rapprentice import ros_utils as ru, clouds, berkeley_pr2, conversions as conv

import subprocess, time
rospy.init_node('test_ar_service')

rospy.wait_for_service('getMarkers')
getMarkers = rospy.ServiceProxy('getMarkers', MarkerPositions)
grabber = cloudprocpy.CloudGrabber()
grabber.startRGBD()

pc = PointCloud2()
br = tf.TransformBroadcaster()
tfms, child_frames, parent_frames = [], [], []
pub = rospy.Publisher('pc_test', PointCloud2)

def publisher ():
    global br, tfms, child_frames, parent_frames
    for tfm, child_frame, parent_frame in zip(tfms, child_frames,parent_frames):
        (trans, rot) = conv.hmat_to_trans_rot(tfm)
        br.sendTransform(trans, rot,
                         rospy.Time.now(),
                         child_frame,
                         parent_frame)
        time.sleep(0.1)
    pub.publish(pc)


class threadClass (th.Thread):
    
    def run (self):
        while True:
            publisher()
            time.sleep(0.1)
    

def getPC ():
    global pc
    r, d = grabber.getRGBD()
    x = clouds.depth_to_xyz(d, berkeley_pr2.f)
    pc = ru.xyzrgb2pc(x, r, 'camera_link_p')


def getPoses ():
    req = MarkerPositionsRequest()
    getPC()
    req.pc = pc
    return getMarkers(req)


def displayMarkerTfm ():
    global parent_frames, child_frames, tfms
    # Get response from service
    markers = getPoses()
    ids = [marker.id for marker in markers.markers.markers]
    poses = [marker.pose.pose for marker in markers.markers.markers]
    tfms = [conv.pose_to_hmat(pose) for pose in poses]
    
    parent_frames = ['camera_link_p' for _ in ids]
    child_frames = ['marker_%d'%id for id in ids]
    
    #     testTransforms(tfms, child_frames, parent_frames)
    
ob = threadClass ()
ob.start()