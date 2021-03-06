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

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from tf.broadcaster import TransformBroadcaster
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

import myviz

server = None
menu_handler = MenuHandler()

def poseCallback ( msg ):
    global server
    marker = server.get("pos_marker")
    print "Marker's pose:"
    print marker.pose
    #print "Msg:", msg

def nullCallback (feedback):
    pass

def createPointAxes (marker, d=1.1, n=50.0):
        
    t = 0.0
    while t < d:
        p1, p2, p3 = Point(), Point(), Point()
        c1, c2, c3 = ColorRGBA(), ColorRGBA(), ColorRGBA()
        
        p1.x = p2.y = p3.z = t
        c1.r = c2.g = c3.b = 1
        c1.a = c2.a = c3.a = 0.5
        
        marker.points.append(p1)
        marker.colors.append(c1)
        marker.points.append(p2)
        marker.colors.append(c2)
        marker.points.append(p3)
        marker.colors.append(c3)
        
        t += d/n

def createPointAxes2 (marker, d=1.1, n=50.0):
        
    t = 0.0
    while t < d:
        p1, p2, p3 = Point(), Point(), Point()
        c1, c2, c3 = ColorRGBA(), ColorRGBA(), ColorRGBA()
        
        p1.x = p1.y = p1.z = t
        p2.x = p2.y = p2.z = t
        p3.x = p3.y = p3.z = t
        c1.r = c2.g = c3.b = 1
        c1.a = c2.a = c3.a = 0.5
        
        marker.points.append(p1)
        marker.colors.append(c1)
        marker.points.append(p2)
        marker.colors.append(c2)
        marker.points.append(p3)
        marker.colors.append(c3)
        
        t += d/n


def createPointLine (marker, d=1.1, n=50.0):
        
    t = 0.0
    while t < d:
        p = Point()
        p.x = t
        c = ColorRGBA()
        c.r, c.a = 1, 0.5
        
        marker.points.append(p)
        marker.colors.append(c)
        
        t += d/n


def makePointAxisMarker( msg ):
    marker = Marker()

    marker.type = Marker.LINE_STRIP
    marker.scale.x = 1 #msg.scale
    marker.scale.y = msg.scale * 0.05
    marker.scale.z = msg.scale * 0.05
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0
    createPointAxes (marker, msg.scale*0.45)

    return marker

def makePointAxisMarker2(  ):
    marker = Marker()

    marker.type = Marker.LINE_STRIP
    marker.scale.x = 1 #msg.scale
    marker.scale.y = 1 * 0.05
    marker.scale.z = 1 * 0.05
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0
    createPointAxes2 (marker, 1*0.45)

    return marker

def makePointLineMarker( msg ):
    marker = Marker()

    marker.type = Marker.POINTS
    marker.scale.x = msg.scale * 0.05
    marker.scale.y = msg.scale * 0.05
    marker.scale.z = msg.scale * 0.05
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0
    createPointLine (marker, msg.scale*0.45)

    return marker

def makeBoxMarker( msg ):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.25
    marker.scale.y = msg.scale * 0.25
    marker.scale.z = msg.scale * 0.25
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker


def makeMarkerControl( msg, mtype='axis' ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    if mtype == 'axis':
        control.markers.append( makePointAxisMarker(msg) )
    elif mtype == 'box':
        control.markers.append( makeBoxMarker(msg) )
    elif mtype == 'line':
        control.markers.append( makePointLineMarker(msg) )
    msg.controls.append( control )
    return control

#####################################################################
# Marker Creation

def make6DofMarker( fixed, single_axis=False):
    global server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/base_footprint"
    int_marker.scale = 0.05
    int_marker.pose.position.x = 0.5
    int_marker.pose.position.y = 0
    int_marker.pose.position.x = 1

    # insert a marker
    if single_axis:
        int_marker.name = "dir_marker"
        int_marker.description = "Direction finder"
        makeMarkerControl(int_marker, 'line')        
    else:
        int_marker.name = "pose_marker"
        int_marker.description = "Transform finder"
        makeMarkerControl(int_marker)

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
    
    int_marker.pose.position.x = 0.5
    int_marker.pose.position.y = 0
    int_marker.pose.position.z = 1


    
    int_marker.description = "Position finder"

    # insert a marker
    makeMarkerControl(int_marker, 'box')


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

def test_markers ():
    global server, app, found, done
    found = False
    done = False
    app = None
    
    rospy.init_node("test_im", anonymous=True)
    
    server = InteractiveMarkerServer("test_im")
    menu_handler.insert("Pose", callback=poseCallback)

    make6DofMarker(True)
    
    m = makePointAxisMarker2()
    m.header.frame_id = "base_footprint"
    xx = rospy.Publisher("here_this", Marker)
    
    server.applyChanges()
    import time
    while True:
        xx.publish(m)
        time.sleep(0.1)
    
    rospy.spin()

if __name__ == '__main__':
    test_markers()