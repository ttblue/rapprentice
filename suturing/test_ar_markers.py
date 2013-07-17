import roslib; roslib.load_manifest('ar_track_alvar')
import rospy
import subprocess

from ar_track_alvar import AlvarMarkers

subprocess.call("rosrun ar_track_alvar")