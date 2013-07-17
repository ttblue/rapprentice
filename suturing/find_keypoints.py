import rosbag
import numpy as np

# Could be generalized to any interface, as long as they have the appropriate things.
import suturing_visualization_interface as svi
from rapprentice import bag_proc as bp
from rapprentice.yes_or_no import yes_or_no

def create_annotations(stamps, meanings, bagfile, video_dir):
    # initialize with basic information of start, stop, look times and description 
    seg_infos = bp.joy_to_annotations(stamps, meanings)

    frame_stamps = [t["look"] for t in seg_infos]
    key_rgb_imgs, _ = bp.get_video_frames(video_dir, frame_stamps)

    bag = rosbag.Bag(bagfile)
    jnames, jstamps, traj = bp.extract_joints(bag)
    
    key_joint_inds = np.searchsorted(jstamps, frame_stamps)
    key_joints = [traj[i] for i in key_joint_inds]
    
    for i,t in enumerate(frame_stamps):
        print 'Looking for key-points in segment %i.'%i
        keypoint_info = {}
        Twk = svi.find_Twk(key_joints[i], jnames)
        
        while True:
            svi.show_pointclouds([key_rgb_imgs[i]])
            kp = raw_input('Which key points are important for this segment?\nChoices are: ' + str(svi.KEYPOINTS) + '.\n Please only enter one key point at a time: ')
            if kp not in svi.KEYPOINTS:
                print 'Invalid keypoint: %s'%kp
            elif kp in keypoint_info.keys():
                if yes_or_no('Already have information for %s. Overwrite?'%kp):
                    kp_loc = svi.find_kp_processing(kp, frame_stamps[i], Twk, video_dir)
                    if kp_loc is None:
                        print 'Something went wrong with keypoint selection.'
                    else:
                        keypoint_info[svi.KEYPOINTS_FULL[kp]] = kp_loc
                        print 'Key point %s saved for this segment.'%kp
            else:
                kp_loc = svi.find_kp_processing(kp, frame_stamps[i], Twk, video_dir)
                if kp_loc is None:
                    print 'Something went wrong with keypoint selection.'
                else:
                    keypoint_info[svi.KEYPOINTS_FULL[kp]] = kp_loc 
                    print 'Key point %s saved for this segment.'%kp
                
            if not yes_or_no('Enter another key point for this segment?'):
                break
        
        seg_infos[i]['extra_information'] = []
        if yes_or_no('Is the needle-tip the relevant end effector for this segment?'):
            if yes_or_no('Is the needle in the left gripper?'):
                seg_infos[i]['extra_information'].append("l_grab")
            else:
                seg_infos[i]['extra_information'].append("r_grab")
        
        seg_infos[i]['key_points'] = keypoint_info

    return seg_infos


def get_keypoints_execution (grabber, keys, tfm):
    """
    Find keypoints during execution phase.
    If fake cloud, cloud would not be none.
    """
    keypoints= {}
    for key in keys:
        while True:
            #svi.show_pointclouds([key_rgb_imgs[i]])
            print "Find key point %s."%svi.KEYPOINTS_SHORT[key]
            kp_loc =  svi.find_kp_execution(svi.KEYPOINTS_SHORT[key], grabber, tfm)
            
            if kp_loc is None:
                print "Something went wrong with key-point selection. Try again."
            else:
                keypoints[key] = kp_loc
                break
    
    return keypoints


def key_points_to_points (keypoints):
    """
    Converts key-points into points.
    """
    points = []
    dist = 0.1
    for key in keypoints:
        loc = np.array(keypoints[key])
        if loc.shape == (4,4):
            if len(keypoints) == 1:
                return loc, True
            # Unpack all the axes and the origin of the transform
            x,y,z,p = loc[0:3].T
            points.append(p)
            points.append(p+dist*x)
            points.append(p+dist*y)
            points.append(p+dist*z)
        elif loc.shape == (2,3):
            # Unpack the point and its normal
            p,n = loc
            points.append(p)
            points.append(p+dist*n)
        elif key != "none":
            points.append(loc)
    return np.array(points), False
