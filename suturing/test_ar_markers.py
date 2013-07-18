import roslib; roslib.load_manifest('ar_track_alvar')
import rospy
#import subprocess, os
#import os.path as osp

from ar_track_alvar.msg import AlvarMarkers

#ar_launch = osp.join(os.getenv('RAPPRENTICE_SOURCE_DIR'), 'launch', 'ar_tracker.launch')
#FNULL = open(os.devnull, 'w')
#subprocess.call("roslaunch " + ar_launch, shell=True, stdout=FNULL, stderr=subprocess.STDOUT)
print "ready"

markers = None

def callback (msg):
    global markers
    markers = msg.markers
    for marker in markers:
            print marker
    exit()

rospy.init_node("test_ar_markers")
sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)

#while True:
#    key = raw_input("q - quit, any other key - print marker pos")
#    if key != "q":
#        for marker in markers:
#            print marker
#    else:
#        break
