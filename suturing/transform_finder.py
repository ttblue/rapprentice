#!/usr/bin/env python

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import roslib; roslib.load_manifest("interactive_markers")
import rospy
import copy

from sensor_msgs.msg import PointCloud2
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from tf.broadcaster import TransformBroadcaster

from rapprentice import ros_utils as ru, conversions
from rapprentice.yes_or_no import yes_or_no
import myviz

server = None
app = None
outer_RVIZ = False
menu_handler = MenuHandler()
done = False
found = False

def doneCallback (feedback):
    global done, found
    done = True
    found = True
    if not outer_RVIZ and app is not None:
        app.exit()

def redoCallback (feedback):
    global done, found
    done = True
    if not outer_RVIZ and app is not None:
        app.exit()

def nullCallback (feedback):
    pass

def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker

def makeBoxControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control


#####################################################################
# Marker Creation

def make6DofMarker( fixed ):
    global server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/base_footprint"
    int_marker.scale = 0.05

    int_marker.name = "tfm_marker"
    int_marker.description = "Transform finder"

    # insert a box
    makeBoxControl(int_marker)

    if fixed:
        int_marker.name += "_fixed"
        int_marker.description += "\n(fixed orientation)"

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    # make one control using default visuals
    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.description="Options"
    control.name = "menu_only_control"
    int_marker.controls.append(copy.deepcopy(control))

    server.insert(int_marker, nullCallback)
    menu_handler.apply( server, int_marker.name )

def makeMoveMarker( ):
    global server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/base_footprint"
    int_marker.scale = 0.05

    int_marker.name = "pos_marker"
    int_marker.description = "Position finder"

    # insert a box
    makeBoxControl(int_marker)


    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    # make one control using default visuals
    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.description="Options"
    control.name = "menu_only_control"
    int_marker.controls.append(copy.deepcopy(control))

    server.insert(int_marker, nullCallback)
    menu_handler.apply( server, int_marker.name )


def find_keypoint_transform_processing (kp, xyz, rgb):
    global server, app, done, found
    app = None
    found = False
    done = False
    
    if rospy.get_name() == '/unnamed':
        rospy.init_node("find_keypoint_%s"%kp, anonymous=True, disable_signals=True)
        
    pcpub = rospy.Publisher('keypoint_pcd', PointCloud2)
    server = InteractiveMarkerServer("basic_controls")

    menu_handler.insert("Done?", callback=doneCallback)
    menu_handler.insert("Re-try with PC 1 second later?", callback=redoCallback)

    pcmsg = ru.xyzrgb2pc(xyz, rgb, 'base_footprint')

    make6DofMarker( False )
    #make6DofMarker( True )

    server.applyChanges()
    
    if yes_or_no("Do you want to open your own rviz window? Might be a better idea."):
        outer_RVIZ = True
    else:
        outer_RVIZ = False
        app = myviz.QApplication( [""] )
        viz = myviz.MyViz()
        viz.resize( 1000, 1000 )
        viz.show()

    p = rospy.Rate(10)
    while not rospy.is_shutdown() and not done:
        pcpub.publish(pcmsg)
        if not outer_RVIZ:
            app.processEvents()
        p.sleep()
    
    # Unhappy with data
    if not found:
        return None
    
    marker = server.get("tfm_marker")
    
    print "Pose of the marker for %s"%kp
    print marker.pose
    
    rospy.signal_shutdown('Finished finding transform')
    
    return conversions.pose_to_hmat(marker.pose).tolist()

def find_keypoint_transform_execution (kp, cloud_topic):
    global server, app, found, done
    found = False
    done = False
    app = None
    
    if rospy.get_name() == '/unnamed':
        rospy.init_node("find_keypoint_%s"%kp, anonymous=True, disable_signals=True)
        
    server = InteractiveMarkerServer("basic_controls")

    menu_handler.insert("Done?", callback=doneCallback)
    menu_handler.insert("Re-try?", callback=redoCallback)

    make6DofMarker( False )
    #make6DofMarker( True )

    server.applyChanges()
    
    if yes_or_no("Do you want to open your own rviz window? Might be a better idea."):
        outer_RVIZ = True
    else:
        outer_RVIZ = False
        app = myviz.QApplication( [""] )
        viz = myviz.MyViz()
        viz.resize( 1000, 1000 )
        viz.show()

    print "Listen to the point_cloud data on: %s"%cloud_topic
    
    p = rospy.Rate(10)
    while not rospy.is_shutdown() and not done:
        if not outer_RVIZ:
            app.processEvents()
        p.sleep()
    
    # Unhappy with data
    if not found:
        return None
    
    marker = server.get("tfm_marker")
    
    print "Pose of the marker for %s"%kp
    print marker.pose
    
    rospy.signal_shutdown('Finished finding transform')
    
    return conversions.pose_to_hmat(marker.pose).tolist()


def find_keypoint_position_processing (kp, xyz, rgb):
    global server, app, done, found
    found = False
    done = False
    app = None
    
    if rospy.get_name() == '/unnamed':
        rospy.init_node("find_keypoint_%s"%kp, anonymous=True, disable_signals=True)
        
    pcpub = rospy.Publisher('keypoint_pcd', PointCloud2)
    server = InteractiveMarkerServer("basic_controls")

    menu_handler.insert("Done?", callback=doneCallback)
    menu_handler.insert("Re-try with PC 1 second later?", callback=redoCallback)

    pcmsg = ru.xyzrgb2pc(xyz, rgb, 'base_footprint')

    makeMoveMarker()

    server.applyChanges()
    
    if yes_or_no("Do you want to open your own rviz window? Might be a better idea."):
        outer_RVIZ = True
    else:
        outer_RVIZ = False
        app = myviz.QApplication( [""] )
        viz = myviz.MyViz()
        viz.resize( 1000, 1000 )
        viz.show()

    p = rospy.Rate(10)
    while not rospy.is_shutdown() and not done:
        pcpub.publish(pcmsg)
        if not outer_RVIZ:
            app.processEvents()
        p.sleep()
    
    # Unhappy with data
    if not found:
        return None
    
    marker = server.get("pos_marker")
    
    pos = marker.pose.position
    print "Position of the marker for %s"%kp
    print pos
    
    rospy.signal_shutdown('Finished finding transform')
    
    return [pos.x, pos.y, pos.z]

def find_keypoint_position_execution (kp, cloud_topic):
    global server, app, found, done
    found = False
    done = False
    app = None
    
    if rospy.get_name() == '/unnamed':
        rospy.init_node("find_keypoint_%s"%kp, anonymous=True, disable_signals=True)
        
    server = InteractiveMarkerServer("basic_controls")

    menu_handler.insert("Done?", callback=doneCallback)
    menu_handler.insert("Re-try?", callback=redoCallback)

    makeMoveMarker()

    server.applyChanges()
    
    if yes_or_no("Do you want to open your own rviz window? Might be a better idea."):
        outer_RVIZ = True
    else:
        outer_RVIZ = False
        app = myviz.QApplication( [""] )
        viz = myviz.MyViz()
        viz.resize( 1000, 1000 )
        viz.show()

    print "Listen to the point_cloud data on: %s"%cloud_topic
    
    p = rospy.Rate(10)
    while not rospy.is_shutdown() and not done:
        if not outer_RVIZ:
            app.processEvents()
        p.sleep()
    
    # Unhappy with data
    if not found:
        return None
    
    marker = server.get("pos_marker")
    
    pos = marker.pose.position
    print "Position of the marker for %s"%kp
    print pos
    
    rospy.signal_shutdown('Finished finding transform')
    
    return [pos.x, pos.y, pos.z]

