from rapprentice import ros2rave, func_utils, berkeley_pr2
import fastrapp
import numpy as np
import cv2
import openravepy
import os.path as osp

def extract_joints(bag):
    """returns (names, traj) 
    """
    traj = []
    stamps = []
    for (_, msg, _) in bag.read_messages(topics=['/joint_states']):        
        traj.append(msg.position)
        stamps.append(msg.header.stamp.to_sec())
    assert len(traj) > 0
    names = msg.name
    return names, stamps, traj
    
def extract_joy(bag):
    """sounds morbid    
    """

    stamps = []
    meanings = []
    button2meaning = {
        12: "look",
        0: "start",
        3: "stop",
        7: "l_open",
        5: "l_close",
        15: "r_open",
        13: "r_close",
        14: "done"
    }
    check_buttons = button2meaning.keys()
    message_stream = bag.read_messages(topics=['/joy'])
    (_,last_msg,_) = message_stream.next()
    for (_, msg, _) in message_stream:
        for i in check_buttons:
            if msg.buttons[i] and not last_msg.buttons[i]:
                stamps.append(msg.header.stamp.to_sec())
                meanings.append(button2meaning[i])
        last_msg = msg
        
    return stamps, meanings

        
def find_disjoint_subsequences(li, seq):
    """
    Returns a list of tuples (i,j,k,...) so that seq == (li[i], li[j], li[k],...)
    Greedily find first tuple, then second, etc.
    """
    subseqs = []
    cur_subseq_inds = []
    for (i_el, el) in enumerate(li):
        if el == seq[len(cur_subseq_inds)]:
            cur_subseq_inds.append(i_el)
            if len(cur_subseq_inds) == len(seq):
                subseqs.append(cur_subseq_inds)
                cur_subseq_inds = []
    return subseqs
    
def joy_to_annotations(stamps, meanings):
    """return a list of dicts giving info for each segment
    [{"look": 1234, "start": 2345, "stop": 3456},...]
    """
    out = []
    ind_tuples = find_disjoint_subsequences(meanings, ["look","start","stop"])
    for tup in ind_tuples:
        out.append({"look":stamps[tup[0]], "start":stamps[tup[1]], "stop":stamps[tup[2]]})
    
    done_inds = [i for (i,meaning) in enumerate(meanings) if meaning=="done"]
    for ind in done_inds:
        out.append({"done":None,"look":stamps[ind], "start":stamps[ind], "stop":stamps[ind]+1})
    
    return out

def add_kinematics_to_group(group, linknames, manipnames, jointnames, robot):
    "do forward kinematics on those links"
    if robot is None: robot = get_robot()
    r2r = ros2rave.RosToRave(robot, group["joint_states"]["name"])
    link2hmats = dict([(linkname, []) for linkname in linknames])
    links = [robot.GetLink(linkname) for linkname in linknames]
    rave_traj = []
    rave_inds = r2r.rave_inds
    for ros_vals in group["joint_states"]["position"]:
        r2r.set_values(robot, ros_vals)
        rave_vals = r2r.convert(ros_vals)
        robot.SetDOFValues(rave_vals, rave_inds)
        rave_traj.append(rave_vals)
        for (linkname,link) in zip(linknames, links):
            link2hmats[linkname].append(link.GetTransform())
    for (linkname, hmats) in link2hmats.items():
        group.create_group(linkname)
        group[linkname]["hmat"] = np.array(hmats)      
        
    rave_traj = np.array(rave_traj)
    rave_ind_list = list(rave_inds)
    for manipname in manipnames:
        arm_inds = robot.GetManipulator(manipname).GetArmIndices()
        group[manipname] = rave_traj[:,[rave_ind_list.index(i) for i in arm_inds]]
        
    for jointname in jointnames:
        joint_ind = robot.GetJointIndex(jointname)
        group[jointname] = rave_traj[:,rave_ind_list.index(joint_ind)]
        
    
    
    
@func_utils.once
def get_robot():
    env = openravepy.Environment()
    env.Load("robots/pr2-beta-static.zae")
    robot = env.GetRobots()[0]
    return robot
    
def add_bag_to_hdf(bag, annotations, hdfroot, demo_name):
    joint_names, stamps, traj = extract_joints(bag)
    traj = np.asarray(traj)
    stamps = np.asarray(stamps)
    
    robot = get_robot()

    for seg_info in annotations:


        group = hdfroot.create_group(demo_name + "_" + seg_info["name"])
    
        start = seg_info["start"]
        stop = seg_info["stop"]
        look = seg_info["look"]
        
        [i_start, i_stop, i_look] = np.searchsorted(stamps, [start, stop, look])
        
        stamps_seg = stamps[i_start:i_stop+1]
        traj_seg = traj[i_start:i_stop+1]
        look_dofs = traj[i_look]
        sample_inds = fastrapp.resample(traj_seg, np.arange(len(traj_seg)), .01, np.inf, np.inf)
        print "trajectory has length", len(sample_inds),len(traj_seg)

    
        traj_ds = traj_seg[sample_inds,:]
        stamps_ds = stamps_seg[sample_inds]
    
        group["description"] = seg_info["description"]
        group["stamps"] = stamps_ds
        group.create_group("joint_states")
        group["joint_states"]["name"] = joint_names
        group["joint_states"]["position"] = traj_ds
        group["joint_states"]["look_position"] = look_dofs
        link_names = ["l_gripper_tool_frame","r_gripper_tool_frame","l_gripper_r_finger_tip_link","l_gripper_l_finger_tip_frame","r_gripper_r_finger_tip_link","r_gripper_l_finger_tip_frame"]
        special_joint_names = ["l_gripper_joint", "r_gripper_joint"]
        manip_names = ["leftarm", "rightarm"]
        
        add_kinematics_to_group(group, link_names, manip_names, special_joint_names, robot)
        
        if seg_info.get('key_points') is not None:
            group.create_group("key_points")
            for key in seg_info["key_points"]:
                group["key_points"][key] = seg_info["key_points"][key]
        if seg_info.get("ar_marker_poses") is not None:
            group.create_group("ar_marker_poses")
            for i in seg_info["ar_marker_poses"]:
                group["ar_marker_poses"][str(i)] = seg_info["ar_marker_poses"][i]
        if seg_info.get('ree') is not None:
            group["ree"] = seg_info["ree"]
        if seg_info.get('lee') is not None:
            group["lee"] = seg_info["lee"]
        if seg_info.get('extra_information') is not None:
            group["extra_information"] = seg_info["extra_information"]

def searchsortednearest(a,v):
    higher_inds = np.fmin(np.searchsorted(a,v), len(a)-1)
    lower_inds = np.fmax(higher_inds-1, 0)
    closer_inds = higher_inds
    lower_is_better = np.abs(a[higher_inds] - v) > np.abs(a[lower_inds] - v)
    closer_inds[lower_is_better] = lower_inds[lower_is_better]
    return closer_inds

def get_video_frames(video_dir, frame_stamps):
    video_stamps = np.loadtxt(osp.join(video_dir,"stamps.txt"))

    frame_inds = searchsortednearest(video_stamps, frame_stamps)
    
    from glob import glob
    rgbnames = glob(osp.join(video_dir, "rgb*.jpg"))
    depthnames = glob(osp.join(video_dir, "depth*.png"))
        
    ind2rgbfname = dict([(int(osp.splitext(osp.basename(fname))[0][3:]), fname) for fname in rgbnames])
    ind2depthfname = dict([(int(osp.splitext(osp.basename(fname))[0][5:]), fname) for fname in depthnames])
    
    #print ind2depthfname
    
    rgbs = []
    depths = []
    for frame_ind in frame_inds:
        rgb = cv2.imread(ind2rgbfname[frame_ind])
        assert rgb is not None
        rgbs.append(rgb)
        depth = cv2.imread(ind2depthfname[frame_ind],2)
        assert depth is not None
        depths.append(depth)
    return rgbs, depths

def get_num_frames(video_dir, frame_stamp, num_frames):
    """
    Returns the next @num_frames frames in the video from frame_stamp.
    """
    video_stamps = np.loadtxt(osp.join(video_dir,"stamps.txt"))
    frame_ind = np.searchsorted(video_stamps, frame_stamp)
    
    rgbs = []
    depths = []
    for i in xrange(frame_ind, frame_ind + num_frames):
        rgb = cv2.imread(osp.join(video_dir,"rgb%.2i.jpg"%i))
        depth = cv2.imread(osp.join(video_dir,"depth%.2i.png"%i),2)
        if rgb is None:
            break
        rgbs.append(rgb)
        depths.append(depth)
        
    return rgbs, depths

def get_next_frame(video_dir, frame_stamp):
    """
    Returns the next frame in the video from frame_stamp.
    Returns next frame stamp, rgb image and depth image.
    """
    video_stamps = np.loadtxt(osp.join(video_dir,"stamps.txt"))
    frame_ind = np.searchsorted(video_stamps, frame_stamp)
    
    new_ind = frame_ind + 1
    
    rgb = cv2.imread(osp.join(video_dir,"rgb%.2i.jpg"%new_ind))
    depth = cv2.imread(osp.join(video_dir,"depth%.2i.png"%new_ind),2)
    
    return video_stamps[new_ind], rgb, depth

def add_rgbd_to_hdf(video_dir, annotations, hdfroot, demo_name):
    
    frame_stamps = [seg_info["look"] for seg_info in annotations]
    
    rgb_imgs, depth_imgs = get_video_frames(video_dir, frame_stamps)
    
    for (i_seg, seg_info) in enumerate(annotations):        
        group = hdfroot[demo_name + "_" + seg_info["name"]]
        group["rgb"] = rgb_imgs[i_seg]
        group["depth"] = depth_imgs[i_seg]
        robot = get_robot()
        r2r = ros2rave.RosToRave(robot, group["joint_states"]["name"])
        r2r.set_values(robot, group["joint_states"]["position"][0])
        T_w_h = robot.GetLink("head_plate_frame").GetTransform()
        T_w_k = T_w_h.dot(berkeley_pr2.T_h_k)
        group["T_w_k"] = T_w_k

