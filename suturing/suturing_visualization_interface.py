import cv2
import numpy as np
import sys
import rospy

from rapprentice.colorize import colorize
from rapprentice import bag_proc as bp, ros2rave, berkeley_pr2, clouds
import transform_finder as tff

KEYPOINTS = ['lh', 'rh', 'tc', 'mc', 'bc', 'ne', 'hn1', 'hn2', 'nt', 'ntt', 'stand', 'rzr', 'none']
KEYPOINTS_FULL = {  'lh':'left_hole',
                    'rh': 'right_hole',
                    'tc': 'top_cut',
                    'mc': 'middle_cut',
                    'bc': 'bottom_cut',
                    'ne': 'needle_end',
                    'hn1': 'hole1_normal',
                    'hn2': 'hole2_normal',
                    'nt': 'needle_tip',
                    'ntt': 'needle_tip_transform',
                    'rzr': 'razor',
                    'stand': 'stand',
                    'none': 'none' }
KEYPOINTS_SHORT = {KEYPOINTS_FULL[k]:k for k in KEYPOINTS_FULL}

MARKER_KEYPOINTS = {'stand':[0,1,3]}
SINGLE_POINT_KEYPOINTS = ['lh','rh','tc','mc','bc']
INTERACTIVE_POSITION_KEYPOINTS = ['ne', 'rzr', 'nt']
INTERACTIVE_DIRECTION_KEYPOINTS = ['hn1','hn2']
INTERACTIVE_TRANSFORM_KEYPOINTS = ['ntt']

WIN_NAME = 'Find Keypoints'

def find_Twk (joints, names):
    """
    Finds the transform of the kinect on the PR2's head with respect to /base_footprint.
    """
    robot = bp.get_robot()
    r2r = ros2rave.RosToRave(robot, names)
    r2r.set_values(robot, joints)
    return berkeley_pr2.get_kinect_transform(robot)

def transform_pointclouds (xyz_depth, tfm):
    """
    Transforms N points of a point cloud (given by an Nx3 matrix) with the transform tfm.
    """
    xyz = clouds.depth_to_xyz(xyz_depth, berkeley_pr2.f)
    return xyz.dot(tfm[:3,:3].T) + tfm[:3,3][None,None,:]

def show_pointclouds(rgb_data):
    """
    Display the point clouds.
    """
    nc = len(rgb_data)
    if nc == 1:
        cv2.imshow(WIN_NAME, rgb_data[0])
        cv2.waitKey(100)
    else:
        rgb_plot_multi = []
        for i in range(nc):
            rgb_plot_multi.append(rgb_data[i])
            cv2.imshow(WIN_NAME, rgb_plot_multi[i])
            cv2.waitKey(100)


def get_kp_clouds(grabber, num_clouds, tfm):
    
    if num_clouds == 1:
        rgb, depth = grabber.getRGBD()
        xyz_tf = transform_pointclouds(depth, tfm)
        return xyz_tf, rgb

    else:    
        xyz_tfs = []
        rgb_plots = []        
        for _ in range(num_clouds):
            rgb, depth = grabber.getRGBD()
            xyz_tf = transform_pointclouds(depth, tfm)
            xyz_tfs.append(xyz_tf)
            rgb_plots.append(rgb)
        return xyz_tfs, rgb_plots


def find_kp_single_cloud (kp, xyz_tf, rgb_img):
    """
    Find the key-point from single point cloud.
    """
    ### clicking set-up 
    class GetClick:
        x = None
        y = None
        done = False
        def callback(self, event, x, y, flags, param):
            if self.done:
                return
            elif event == cv2.EVENT_LBUTTONDOWN:
                self.x = x
                self.y = y
                self.done = True
    
    
    
    print colorize('Click on the center of the %s.'%KEYPOINTS_FULL[kp], 'red', bold=True)
    #print colorize('If this keypont is occluded, select the image window and press any key.', 'red', bold=True)
    
    gc = GetClick()
    cv2.imshow(WIN_NAME, rgb_img)
    cv2.setMouseCallback(WIN_NAME, gc.callback)
    while not gc.done:
        k = cv2.waitKey(100)
        #TODO
        #if k == -1:
        #    continue
        #else: 
        #    last_loc, found_seg = 1,1#get_last_kp_loc(past_keypts, kp, current_seg)
        #    print kp, 'found in segment %s at location %s'%(found_seg, last_loc)
        #    return last_loc

    row_kp = gc.x
    col_kp = gc.y
    
    cv2.circle(rgb_img, (row_kp, col_kp), 5, (0, 0, 255), -1)
    cv2.imshow(WIN_NAME, rgb_img)    
    cv2.waitKey(100)

    xyz_tf[np.isnan(xyz_tf)] = -2
    x, y, z = xyz_tf[col_kp, row_kp]

    print kp, '3d location', x, y, z

    #return (x, y, z), (col_kp, row_kp)
    return [dx, y, z]

def find_kp_processing (kp, frame_stamp, tfm, video_dir):
    """
    Process the video to find the kp.
    """
    if kp in MARKER_KEYPOINTS:
        print "Will store all the visible AR markers for %s."%kp
        return [0,0,0]
        
    elif kp in SINGLE_POINT_KEYPOINTS:
        key_rgb, key_depth_img = bp.get_video_frames(video_dir, [frame_stamp])
        xyz_tf = transform_pointclouds(key_depth_img[0], tfm)
        return find_kp_single_cloud(kp, xyz_tf, key_rgb[0])

    elif kp in INTERACTIVE_POSITION_KEYPOINTS:
        time_added = 0
        while True:
            key_rgb_imgs, key_depth_images = bp.get_video_frames(video_dir, [frame_stamp + time_added])
            xyz_tf = transform_pointclouds(key_depth_images[0], tfm)
            kp_loc = tff.find_keypoint_position_processing(kp, xyz_tf, key_rgb_imgs[0])

            if kp_loc is not None:
                # Special case for nt
                if kp=='nt' and kp_loc[2] < 0.8:
#                     x: 0.51092261076
#                     y: 0.0939055234194
#                     z: 0.933865368366

                    print '%s too low. Try again.'%kp
                else:
                    return kp_loc
            else:
                time_added += 1
                if time_added > 15:
                    print 'Could not find valid %s over 15 seconds. Something seems to be wrong.'%kp
                    return None
                    
    elif kp in INTERACTIVE_DIRECTION_KEYPOINTS:
        time_added = 0
        while True:
            key_rgb_imgs, key_depth_images = bp.get_video_frames(video_dir, [frame_stamp + time_added])
            xyz_tf = transform_pointclouds(key_depth_images[0], tfm)
            kp_loc = tff.find_keypoint_transform_processing(kp, xyz_tf, key_rgb_imgs[0], True)

            if kp_loc is None:
                time_added += 1
                if time_added > 15:
                    print 'Could not find valid %s over 15 seconds. Something seems to be wrong.'
                    return None
            else:
                p = kp_loc[0:3,3]
                n = kp_loc[0:3,0]
                return [p.tolist(), n.tolist()]
                
    elif kp in INTERACTIVE_TRANSFORM_KEYPOINTS:
        time_added = 0
        while True:
            key_rgb_imgs, key_depth_images = bp.get_video_frames(video_dir, [frame_stamp + time_added])
            xyz_tf = transform_pointclouds(key_depth_images[0], tfm)
            kp_loc = tff.find_keypoint_transform_processing(kp, xyz_tf, key_rgb_imgs[0])
            
            if kp_loc is None:
                time_added += 1
                if time_added > 15:
                    print 'Could not find valid %s over 15 seconds. Something seems to be wrong.'%kp
                    return None
                else:
                    print 'Try to find the %s again.'%KEYPOINTS_FULL[kp]
            else:
                return kp_loc.tolist()
    
    elif kp == 'none':
        return [0,0,0]
    

def find_kp_execution (kp, grabber, tfm):
    """
    Find keypoint location during execution time.
    """
    if kp in MARKER_KEYPOINTS:
        print "Will find all the visible AR markers for %s."%kp
        return [0,0,0]

    elif kp in SINGLE_POINT_KEYPOINTS:
        xyz_tf, rgb = get_kp_clouds(grabber, 1, tfm)
        return find_kp_single_cloud(kp, xyz_tf, rgb)
    
    elif kp in INTERACTIVE_POSITION_KEYPOINTS:
        attempts = 0
        while True:
            kp_loc = tff.find_keypoint_position_execution(kp, grabber, tfm)
            
            if kp_loc is not None:
                if kp=='nt' and kp_loc[2] < 0.8:
                    print 'Needle tip too low. Try again.'
                else:
                    return kp_loc
            else:
                attempts += 1
                if attempts > 15:
                    print 'Could not find valid %s in 15 attempts. Something seems to be wrong.'%kp
                    return None

    elif kp in INTERACTIVE_DIRECTION_KEYPOINTS:
        attempts = 0
        while True:
            kp_loc = tff.find_keypoint_transform_execution(kp, grabber, tfm, True)

            if kp_loc is None:
                attempts += 1
                if attempts > 15:
                    print 'Could not find valid %s in 15 attempts. Something seems to be wrong.'%kp
                    return None
            else:
                p = kp_loc[0:3,3]
                n = kp_loc[0:3,0]
                return [p.tolist(), n.tolist()]

    elif kp in INTERACTIVE_TRANSFORM_KEYPOINTS:
        attempts = 0
        while True:
            # TODO: Check cloud topic
            kp_loc = tff.find_keypoint_transform_execution(kp, grabber, tfm)
            
            if kp_loc is None:
                attempts += 1
                if attempts > 15:
                    print 'Could not find valid %s in 15 attempts. Something seems to be wrong.'%kp
                    return None
                else:
                    print 'Try to find the %s again.'%KEYPOINTS_FULL[kp]
            else:
                return kp_loc.tolist()
 
    elif kp == 'none':
        return [0,0,0]

############################################################################# OLD CODE

def get_last_kp_loc(exec_keypts, desired_keypt, current_seg):        
    
    search_seg = current_seg - 1
       
    while(True):
        if search_seg < 0:
            print "Reached beginning of execution and couldn't find desired keypoint! Aborting..."
            sys.exit(1)            
        else:
            search_seg_names = exec_keypts[search_seg]["names"]
            search_seg_locs = exec_keypts[search_seg]["locations"]
        
            for k in range(len(search_seg_names)):
                if search_seg_names[k] == desired_keypt:
                    kp_loc = search_seg_locs[k]
                    kp_found = True
            
        if kp_found:        
            return kp_loc, search_seg
        else:
            search_seg -= 1

# No longer needed - using interactive markers.
def find_kp_red_block (kp, xyz_tfs, rgb_imgs):
    """
    Find the key-point from clicking red block.
    """
    
    ### clicking set-up 
    class GetClick:
        xy = None
        done = False
        def callback(self, event, x, y, flags, param):
            if self.done:
                return
            elif event == cv2.EVENT_LBUTTONDOWN:
                self.xy = (x,y)
                self.done = True


    rect_corners = []
    nc = len(xyz_tfs)
    rgb_plot = rgb_imgs[0].copy()
    print colorize("Click at the corners of a rectangle which encompasses the red block.", 'red', bold=True)

    cv2.imshow(WIN_NAME, rgb_plot)
    for i in xrange(2):
        gc = GetClick()
        cv2.setMouseCallback(WIN_NAME, gc.callback)
        while not gc.done:
            cv2.waitKey(100)
        rect_corners.append(gc.xy)

    xy_tl = np.array(rect_corners).min(axis=0)
    xy_br = np.array(rect_corners).max(axis=0)

    cv2.rectangle(rgb_plot, tuple(xy_tl), tuple(xy_br), (0, 255, 0))
    cv2.imshow(WIN_NAME, rgb_plot)
    cv2.waitKey(100)
    
    colmin, rowmin = xy_tl
    colmax, rowmax = xy_br
    
    high_red_xyz = []

    valid_pts = 0
    # extract depths from drawn rectangle

    FOAM_HEIGHT = .85
    
    for i in range(nc):        
        z_rectangle = xyz_tfs[i][rowmin:rowmax, colmin:colmax, 2] # z values in extracted rectangle
        
        hsv = cv2.cvtColor(rgb_imgs[i][rowmin:rowmax, colmin:colmax].copy(),cv2.COLOR_BGR2HSV)
        h=hsv[:,:,0]
        s=hsv[:,:,1]
        v=hsv[:,:,2]
        
        color_mask = ((h<20) | (h>150)) & (s > 100) & (v > 90)
        print "max z ", np.max(z_rectangle)
        height_mask = z_rectangle > FOAM_HEIGHT + .025
        
        total_mask = color_mask & height_mask
        print "%ith image has %i valid points (above foam and red)"%(i, total_mask.sum())
        valid_pts += total_mask.sum()
        
        high_red_xyz.extend(xyz_tfs[i][rowmin:rowmax, colmin:colmax, :][total_mask])
    
    if valid_pts == 0:
        return None, 0
    
    xyz_avg = np.median(high_red_xyz,axis=0)
    
    from rapprentice.pcl_utils import xyz2uv
    row, col = xyz2uv(xyz_avg).astype('int')[0]

    cv2.circle(rgb_plot, (row, col), 3, (255, 0, 0), 2)
    cv2.imshow(WIN_NAME, rgb_plot)
    cv2.waitKey(100)    

    print 'Key point location', xyz_avg
 
    return xyz_avg, valid_pts


# No longer needed - using interactive markers.
def find_needle_tip (kp, xyz_tfs, rgb_imgs):
    """
    Find the key-point from multiple point clouds, but with a minimum distance threshold.
    """
        ### clicking set-up 
    class GetClick:
        xy = None
        done = False
        def callback(self, event, x, y, flags, param):
            if self.done:
                return
            elif event == cv2.EVENT_LBUTTONDOWN:
                self.xy = (x,y)
                self.done = True
                
    
    rect_corners = []
    nc = len(xyz_tfs)
    rgb_plot = rgb_imgs[0].copy()
    print colorize("click at the corners of a rectangle which encompasses the needle tip", 'red', bold=True)

    cv2.imshow(WIN_NAME, rgb_plot)
    for i in xrange(2):
        gc = GetClick()
        cv2.setMouseCallback(WIN_NAME, gc.callback)
        while not gc.done:
            cv2.waitKey(10)
        rect_corners.append(gc.xy)

    xy_tl = np.array(rect_corners).min(axis=0)
    xy_br = np.array(rect_corners).max(axis=0)

    cv2.rectangle(rgb_plot, tuple(xy_tl), tuple(xy_br), (0, 255, 0))
    cv2.imshow(WIN_NAME, rgb_plot)
    cv2.waitKey(10)

    colmin, rowmin = xy_tl
    colmax, rowmax = xy_br

    row_needle = []
    col_needle = []

    xneedle = np.zeros((nc))
    yneedle = np.zeros((nc))
    zneedle = np.zeros((nc))

    # extract depths from drawn rectangle
    for i in range(nc):
 
        z_rectangle = xyz_tfs[i][rowmin:rowmax, colmin:colmax, 2] # z values in extracted rectangle
        z_rectangle[np.isnan(z_rectangle)] = -2
        row_needle_temp, col_needle_temp = np.unravel_index(z_rectangle.argmax(), z_rectangle.shape)

        col_needle_temp += colmin # since these indices are from small rectangle
        row_needle_temp += rowmin

        col_needle.append(col_needle_temp)
        row_needle.append(row_needle_temp)
        
        xneedle[i], yneedle[i], zneedle[i] = xyz_tfs[i][row_needle_temp, col_needle_temp]

        del z_rectangle

    ind = np.argmax(zneedle)

    max_needle = []
    max_needle.append(xneedle[ind])
    max_needle.append(yneedle[ind])
    max_needle.append(zneedle[ind])

    #cv2.circle(rgb_plots[ind], (col_needle[ind], row_needle[ind]), 3, (255, 0, 0), 2)
    #cv2.imshow(WIN_NAME, rgb_plots[ind])
    #cv2.waitKey(100)
        

    cv2.circle(rgb_plot, (col_needle[ind], row_needle[ind]), 3, (255, 0, 0), 2)
    cv2.imshow(WIN_NAME, rgb_plot)
    cv2.waitKey(100)    
       
    print '%s\'s 3d location: '%KEYPOINTS_FULL[kp], max_needle
    
    # plot the point cloud with a circle around "highest" point    
    #from mayavi import mlab
    #x,y,z = clouds[ind][0][~np.isnan(clouds[ind][0][:,:,0])].T    
    #mlab.points3d(x,y,z,color=(1,0,0), mode="2dvertex")
    #mlab.points3d([max_needle[0]], [max_needle[1]], [max_needle[2]], color=(0,1,0), mode="sphere", scale_factor=.04, opacity=.2)
    #mlab.show()

    #raw_input("press enter when done looking")        
    #print "at end of needle tip func"

    return max_needle
