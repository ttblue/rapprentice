#!/usr/bin/env python

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("h5file", type=str)

parser.add_argument("--openloop", type=int, default=0)
parser.add_argument("--execution", type=int, default=0)
parser.add_argument("--animation", type=int, default=0)
parser.add_argument("--ask", type=int, default=0)
parser.add_argument("--fake_data_segment",type=str)
parser.add_argument("--fake_data_transform", type=float, nargs=6, metavar=("tx","ty","tz","rx","ry","rz"),
    default=[0,0,0,0,0,0], help="translation=(tx,ty,tz), axis-angle rotation=(rx,ry,rz)")

parser.add_argument("--interactive",action="store_true")

args = parser.parse_args()

#if args.fake_data_segment is None: assert args.execution==1

###################


"""
Workflow:
1. Fake data + animation only
    --fake_data_segment=xxx --execution=0
2. Fake data + Gazebo. Set Gazebo to initial state of fake data segment so we'll execute the same thing.
    --fake_data_segment=xxx --execution=1
    This is just so we know the robot won't do something stupid that we didn't catch with openrave only mode.
3. Real data + Gazebo
    --execution=1 
    The problem is that the gazebo robot is in a different state from the real robot, in particular, the head tilt angle. TODO: write a script that       sets gazebo head to real robot head
4. Real data + Real execution.
    --execution=1

The question is, do you update the robot's head transform.
If you're using fake data, don't update it.

"""


from rapprentice import registration, colorize, berkeley_pr2, yes_or_no, \
     animate_traj, ros2rave, plotting_openrave, task_execution, resampling, PR2
from rapprentice import pr2_trajectories, retiming, math_utils as mu
import rospy

import subprocess

import cloudprocpy, trajoptpy, json, openravepy
import numpy as np, h5py
from numpy import asarray

import find_keypoints as fk, suturing_visualization_interface as svi

subprocess.call("killall XnSensorServer", shell=True)

def redprint(msg):
    print colorize.colorize(msg, "red", bold=True)
    
def split_trajectory_by_gripper(seg_info):
    rgrip = asarray(seg_info["r_gripper_joint"])
    lgrip = asarray(seg_info["l_gripper_joint"])

    thresh = .04 # open/close threshold

    n_steps = len(lgrip)


    # indices BEFORE transition occurs
    l_openings = np.flatnonzero((lgrip[1:] >= thresh) & (lgrip[:-1] < thresh))
    r_openings = np.flatnonzero((rgrip[1:] >= thresh) & (rgrip[:-1] < thresh))
    l_closings = np.flatnonzero((lgrip[1:] < thresh) & (lgrip[:-1] >= thresh))
    r_closings = np.flatnonzero((rgrip[1:] < thresh) & (rgrip[:-1] >= thresh))

    before_transitions = np.r_[l_openings, r_openings, l_closings, r_closings]
    after_transitions = before_transitions+1
    seg_starts = np.unique(np.r_[0, after_transitions])
    seg_ends = np.unique(np.r_[before_transitions, n_steps-1])

    return seg_starts, seg_ends

def binarize_gripper(angle):
    open_angle = .08
    closed_angle = 0    
    thresh = .04
    if angle > thresh: return open_angle
    else: return closed_angle


    
def plan_follow_traj(robot, manip_name, ee_link, new_hmats, old_traj, start_fixed=True):
        
    n_steps = len(new_hmats)
    assert old_traj.shape[0] == n_steps
    assert old_traj.shape[1] == 7
    
    arm_inds  = robot.GetManipulator(manip_name).GetArmIndices()
    robot.SetActiveDOFs(arm_inds)
    cur_dofs = robot.GetActiveDOFValues()
    
    ee_linkname = ee_link.GetName()
    
    init_traj = old_traj.copy()
    #init_traj[0] = robot.GetDOFValues(arm_inds)

    request = {
        "basic_info" : {
            "n_steps" : n_steps,
            "manip" : manip_name,
            "start_fixed" : start_fixed
        },
        "costs" : [
        {
            "type" : "joint_vel",
            "params": {"coeffs" : [1]}
        },
        {
            "type" : "collision",
            "params" : {"coeffs" : [10],"dist_pen" : [0.005]}
        }                
        ],
        "constraints" : [
        ],
        "init_info" : {
            "type":"given_traj",
            "data":[x.tolist() for x in init_traj]
        }
    }
    
    if start_fixed and len(init_traj)>0:
        request["init_info"]["data"][0] = cur_dofs.tolist()
        
    poses = [openravepy.poseFromMatrix(hmat) for hmat in new_hmats]
    
    for (i_step,pose) in enumerate(poses):
        request["costs"].append(
            {"type":"pose",
             "params":{
                "xyz":pose[4:7].tolist(),
                "wxyz":pose[0:4].tolist(),
                "link":ee_linkname,
                "timestep":i_step,
                "pos_coeffs":[20,20,20],
                "rot_coeffs":[20,20,20]
             }
            })
    
    
    
    s = json.dumps(request)
    prob = trajoptpy.ConstructProblem(s, Globals.env) # create object that stores optimization problem
    result = trajoptpy.OptimizeProblem(prob) # do optimization
    traj = result.GetTraj()    
        
    saver = openravepy.RobotStateSaver(robot)
    pos_errs = []
    for i_step in xrange(1,n_steps):
        row = traj[i_step]
        robot.SetDOFValues(row, arm_inds)
        tf = ee_link.GetTransform()
        pos = tf[:3,3]
        pos_err = np.linalg.norm(poses[i_step][4:7] - pos)
        pos_errs.append(pos_err)
    pos_errs = np.array(pos_errs)
        
    print "planned trajectory for %s. max position error: %.3f. all position errors: %s"%(manip_name, pos_errs.max(), pos_errs)
            
    return traj
    
    
def set_gripper_maybesim(lr, value):
    if args.execution:
        gripper = {"l":Globals.pr2.lgrip, "r":Globals.pr2.rgrip}[lr]
        gripper.set_angle(value)
        Globals.pr2.join_all()
    else:
        Globals.robot.SetDOFValues([value*5], [Globals.robot.GetJoint("%s_gripper_l_finger_joint"%lr).GetDOFIndex()])
        
def exec_traj_maybesim(bodypart2traj, speed_factor=0.5):
    if args.animation:
        """
        dof_inds = []
        trajs = []
        for (part_name, traj) in bodypart2traj.items():
            manip_name = {"larm":"leftarm","rarm":"rightarm"}[part_name]
            dof_inds.extend(Globals.robot.GetManipulator(manip_name).GetArmIndices())            
            trajs.append(traj)
        full_traj = np.concatenate(trajs, axis=1)
        Globals.robot.SetActiveDOFs(dof_inds)
        animate_traj.animate_traj(full_traj, Globals.robot, restore=False,pause=True)
        """
        name2part = {"lgrip":Globals.pr2.lgrip,
                     "rgrip":Globals.pr2.rgrip,
                     "larm":Globals.pr2.larm,
                     "rarm":Globals.pr2.rarm,
                     "base":Globals.pr2.base}
        dof_inds = []
        trajs = []
        vel_limits = []
        for (part_name, traj) in bodypart2traj.items():
            manip_name = {"larm":"leftarm","rarm":"rightarm"}[part_name]
            vel_limits.extend(name2part[part_name].vel_limits)
            dof_inds.extend(Globals.robot.GetManipulator(manip_name).GetArmIndices())
            if traj.ndim == 1: traj = traj.reshape(-1,1)            
            trajs.append(traj)
        
        trajectories = np.concatenate(trajs, 1)
        print trajectories.shape
        times = retiming.retime_with_vel_limits(trajectories, np.array(vel_limits))
        times_up = np.linspace(0, times[-1], int(np.ceil(times[-1]/.1)))
        full_traj = mu.interp2d(times_up, times, trajectories)
        print full_traj.shape
        
        Globals.robot.SetActiveDOFs(dof_inds)
        animate_traj.animate_traj(full_traj, Globals.robot, restore=False,pause=True)
        #return True
    if args.execution:
        pr2_trajectories.follow_body_traj(Globals.pr2, bodypart2traj, speed_factor=speed_factor)
        return True


def select_segment(demofile):
    seg_names = demofile.keys()
    print "choose from the following options (type an integer)"
    for (i, seg_name) in enumerate(seg_names):
        print "%i: %s"%(i,seg_name)
    choice_ind = task_execution.request_int_in_range(len(seg_names))
    chosen_seg = seg_names[choice_ind] 
    return chosen_seg

def find_closest(demofile):
    "for now, just prompt the user"
    return select_segment(demofile)
            
            
def arm_moved(joint_traj):    
    return ((joint_traj[1:] - joint_traj[:-1]).ptp(axis=0) > .05).any()
        
def tpsrpm_plot_cb(x_nd, y_md, targ_Nd, corr_nm, wt_n, f):
    ypred_nd = f.transform_points(x_nd)
    handles = []
    handles.append(Globals.env.plot3(ypred_nd, 3, (0,1,0)))
    handles.extend(plotting_openrave.draw_grid(Globals.env, f.transform_points, x_nd.min(axis=0), x_nd.max(axis=0), xres = .1, yres = .1, zres = .04))
    Globals.viewer.Step()
    

import threading as th, time
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from rapprentice import clouds, ros_utils as ru, conversions as conv
from sensor_msgs.msg import PointCloud2


class Globals:
    # Rave
    env = None
    robot = None
    demo_env = None
    demo_robot = None
    
    # Needle tip
    needle_tip = None
    demo_needle_tip = None
    rel_ntfm = None
    demo_rel_ntfm = None

    # PR2 control
    pr2 = None
    sponge = None



class threadClass(th.Thread):
    
#     new_traj = None
#     old_traj = None
    grabber = None
#     
    def __init__(self):
        th.Thread.__init__(self)
        self.old_pub = rospy.Publisher('old_traj', Path)
        self.new_pub = rospy.Publisher('new_traj', Path)
        self.pc_pub = rospy.Publisher('scene_points', PointCloud2)
        self.new_traj = Path()
        self.old_traj = Path()
#         self.old_traj.header.frame_id = "/base_footprint"
#         self.new_traj.header.frame_id = "/base_footprint"
        #self.new_traj = Marker()
        #self.old_traj = Marker()
        self.old_traj.header.frame_id = "/base_footprint"
        #self.old_traj.name = "old_traj"
        self.new_traj.header.frame_id = "/base_footprint"
        #self.new_traj.name = "new_traj"
#         self.old_traj.color.r = 1
#         self.old_traj.color.a = 0.5        
#         self.new_traj.color.b = 1
#         self.new_traj.color.a = 0.5 
#         self.old_traj.scale.x = 1
#         self.old_traj.scale.y = 1
#         self.old_traj.scale.z = 1
#         self.new_traj.scale.x = 1
#         self.new_traj.scale.y = 1
#         self.new_traj.scale.z = 1
#         self.new_traj.pose = Pose()
#         self.old_traj.pose = Pose()  
#         self.old_traj.type = Marker.POINTS
#         self.new_traj.type = Marker.POINTS
#     
    
    def get_PC (self):
        r, d = self.grabber.getRGBD()
        x = clouds.depth_to_xyz(d, berkeley_pr2.f)
        if Globals.pr2 is not None:
            Globals.pr2.update_rave()
            tfm = berkeley_pr2.get_kinect_transform(Globals.robot)
            x = x.dot(tfm[:3,:3].T) + tfm[:3,3][None,None,:]
        return ru.xyzrgb2pc(x, r, 'base_footprint')
    
    def run (self):
        while True:
            self.publish()

    def publish (self):
        if self.grabber is not None:
                pc = self.get_PC()
                self.pc_pub.publish(pc)
        self.old_pub.publish(self.old_traj)
        self.old_pub.publish(self.new_traj)
        time.sleep(0.1)
        
thc = threadClass()

def update_markers (new_traj, old_traj):
    
    print new_traj
    old_poses = [conv.hmat_to_pose(hmat) for hmat in old_traj]
    old_poses_s = []
    for pose in old_poses:
        ps = PoseStamped()
        ps.pose = pose
        ps.header.frame_id = 'base_footprint'
        old_poses_s.append(ps)
    thc.old_traj.poses = old_poses_s
     
    new_poses = [conv.hmat_to_pose(hmat) for hmat in new_traj]
    new_poses_s = []
    for pose in new_poses:
        ps = PoseStamped()
        ps.pose = pose
        ps.header.frame_id = 'base_footprint'
        new_poses_s.append(ps)
    thc.new_traj.poses = new_poses_s
#     thc.old_traj = [conv.hmat_to_pose(hmat).position for hmat in old_traj]
#     thc.new_traj = [conv.hmat_to_pose(hmat).position for hmat in new_traj]

###################

# Use only the relevant AR markers
def find_common_marker_poses (poses1, poses2, keys):
    kp1, kp2 = {}, {}
    common_id = np.intersect1d(poses1.keys(), poses2.keys())
    marker_keys = [key for key in keys if key in svi.MARKER_KEYPOINTS]
    
    for key in marker_keys:
        for i in svi.MARKER_KEYPOINTS[key]:
            if i in common_id:
                kp1[i] = poses1[i]
                kp2[i] = poses2[i]
        
    return kp1, kp2


def addBoxToRave (env, name, pos=None, halfExtents=None):
    """
    Adds a box of given position/half-extents to the rave Environment.
    """
    # default arguments, kind of
    if pos is None or halfExtents is None:
        if name == "table":
            pos = [0.75,0,0.72]
            halfExtents = [0.5,0.45,0.05]
        elif name == "sponge":
            pos = [0.6,0,0.8]
            halfExtents = [0.3,0.3,0.015]
        
    with env:
        body = openravepy.RaveCreateKinBody(env,'')
        body.SetName(name)
        body.InitFromBoxes(np.array([pos + halfExtents]),True)
        env.AddKinBody(body,True)
    return body

# Sets the rave demo arm to the initial position and grabs needle
# Sets the rave robot arm to current position and grabs needle
# Stores relative transform for later use
def setup_needle_grabs (lr, joint_names, old_joints, old_tfm, new_tfm):
    joint_names = list(joint_names)
    arm = {"l":"leftarm", "r":"rightarm"}[lr]
    manip = Globals.demo_robot.GetManipulator(arm)
    arm_names = [j.GetName() for j in Globals.demo_robot.GetJoints(manip.GetArmJoints())]
    arm_inds = [joint_names.index(name) for name in arm_names]
    
    arm_joint_vals = np.asarray([old_joints[j] for j in arm_inds])
        
    Globals.demo_robot.SetActiveDOFs(manip.GetArmIndices())
    Globals.demo_robot.SetActiveDOFValues(arm_joint_vals)
    Globals.demo_needle_tip.SetTransform(old_tfm)
    Globals.demo_robot.Grab(Globals.demo_needle_tip, Globals.demo_robot.GetLink("%s_gripper_tool_frame"%lr))
    
    Globals.pr2.update_rave()
    Globals.needle_tip.SetTransform(new_tfm)
    Globals.robot.Grab(Globals.needle_tip, Globals.robot.GetLink("%s_gripper_tool_frame"%lr))
    
    T_w_g = Globals.robot.GetLink("%s_gripper_tool_frame"%lr).GetTransform()
    T_w_n = new_tfm
    Globals.rel_ntfm = np.linalg.inv(T_w_g).dot(T_w_n)
    T_w_g_demo = Globals.robot.GetLink("%s_gripper_tool_frame"%lr).GetTransform()
    T_w_n_demo = old_tfm
    Globals.demo_rel_ntfm = np.linalg.inv(T_w_g_demo).dot(T_w_n_demo)

# Now that the needle has been grabbed by the demo robot, find needle trajectory
def setup_needle_traj (lr, arm_traj):
    needle_traj = []
    arm = {"l":"leftarm", "r":"rightarm"}[lr]
    Globals.demo_robot.SetActiveDOFs(Globals.demo_robot.GetManipulator(arm).GetArmIndices())
    for row in arm_traj:
        Globals.demo_robot.SetActiveDOFValues(row)
        needle_traj.append(Globals.demo_needle_tip.GetTransform())
    
    return np.asarray(needle_traj)

def unwrap_joint_traj (lr, new_joint_traj):
    
    arm= {"l":Globals.pr2.larm, "r":Globals.pr2.rarm}[lr]
    current_dofs = arm.get_joint_positions()
    new_traj = []
    for dofs in new_joint_traj:
        for i in [4,6]:
            dofs[i] = PR2.closer_ang(dofs[i], current_dofs[i])
        new_traj.append(dofs)
    new_traj = np.asarray(new_traj)
    return new_traj
#     new_roll_traj = np.mod(new_joint_traj[:,[4,6]], np.pi*2) # roll of wrist
#     print new_roll_traj
#     arm= {"l":Globals.pr2.larm, "r":Globals.pr2.rarm}[lr]
#     current_val = arm.get_joint_positions()[[4,6]]
#     current_val_wrapped = np.mod(current_val, 2*np.pi)
#     offset = current_val - current_val_wrapped
#     
#     new_roll_traj = new_roll_traj + offset
#     new_joint_traj[:,[4,6]] = new_roll_traj
#     return new_joint_traj  


def main():

    demofile = h5py.File(args.h5file, 'r')
    
    trajoptpy.SetInteractive(args.interactive)
    rospy.init_node("exec_task",disable_signals=True)
    
    thc.start()
    
    if args.execution:
        Globals.pr2 = PR2.PR2()
        Globals.env = Globals.pr2.env
        Globals.robot = Globals.pr2.robot
        
    else:
        Globals.env = openravepy.Environment()
        Globals.env.StopSimulation()
        Globals.env.Load("robots/pr2-beta-static.zae")
        Globals.robot = Globals.env.GetRobots()[0]
        
    import trajoptpy.make_kinbodies as mk
    #Globals.env.Load("/home/sibi/sandbox/rapprentice/objects/table.xml")
    addBoxToRave(Globals.env, "table")
    Globals.sponge = addBoxToRave(Globals.env, "sponge") # Not sure about this
    Globals.needle_tip = mk.create_spheres(Globals.env, [(0,0,0)], radii=0.02, name="needle_tip")
    Globals.demo_env = Globals.env.CloneSelf(1)
    Globals.demo_env.StopSimulation()
    Globals.demo_robot = Globals.demo_env.GetRobot("pr2")
    Globals.demo_needle_tip = Globals.demo_env.GetKinBody("needle_tip")
    
    if not args.fake_data_segment:
        grabber = cloudprocpy.CloudGrabber()
        grabber.startRGBD()
        thc.grabber = grabber

    #Globals.viewer = trajoptpy.GetViewer(Globals.env)
    print "j"
    #####################
    
    #Globals.env.SetViewer('qtcoin')
    #threadClass().start()

    while True:
        redprint("Acquire point cloud")
         
        ################################
        redprint("Finding closest demonstration")
        if args.fake_data_segment:
            seg_name = args.fake_data_segment
        else:
            seg_name = find_closest(demofile)
        
        seg_info = demofile[seg_name]
        # redprint("using demo %s, description: %s"%(seg_name, seg_info["description"]))
    
        ################################
        
        redprint("Generating end-effector trajectory")    

        handles = []
        
        if seg_info.get('ar_marker_poses') is None:
            use_markers = False
        else:
            use_markers = True

        if use_markers:
            old_marker_poses_str = seg_info['ar_marker_poses']
            old_marker_poses = {}
            for i in old_marker_poses_str:
                old_marker_poses[int(i)] = old_marker_poses_str[i]
        
        
        # TODO: Have a check for only transformations -> rigid transformations
        if args.fake_data_segment:
            new_keypoints = demofile[args.fake_data_segment]["key_points"]
            if use_markers:
                new_marker_poses_str = demofile[args.fake_data_segment]["ar_marker_poses"]
                new_marker_poses = {}
                for i in new_marker_poses_str:
                    new_marker_poses[int(i)] = new_marker_poses_str[i]
            if seg_info['key_points'].keys().sort() != new_keypoints.keys().sort():
                print "Keypoints don't match."
                exit(1)
        else:
            Globals.pr2.update_rave()
            T_w_k = berkeley_pr2.get_kinect_transform(Globals.robot)
            new_keypoints, new_marker_poses = fk.get_keypoints_execution(grabber, seg_info['key_points'].keys(), T_w_k, use_markers)


        print "Warping the points for the new situation."

        if "l_grab" in seg_info['extra_information'] or "r_grab" in seg_info['extra_information']:
            use_ntt_kp = False
        else:
            use_ntt_kp = True

        # Points from keypoints
        old_xyz, rigid, kp_mapping = fk.key_points_to_points(seg_info['key_points'], use_ntt_kp)
        new_xyz, _, _ = fk.key_points_to_points(new_keypoints, use_ntt_kp)

        # Points from markers
        if use_markers:
            old_common_poses, new_common_poses = find_common_marker_poses(old_marker_poses, new_marker_poses, new_keypoints.keys())
            old_m_xyz, rigid_m, _ = fk.key_points_to_points(old_common_poses, False)
            new_m_xyz, _, _ = fk.key_points_to_points(new_common_poses, False)

            if len(old_m_xyz) > 0 and rigid: rigid = False
            elif rigid_m and not old_xyz.any(): rigid = True

            # concatenate points
            if old_xyz.any() and old_m_xyz.any():
                old_xyz = np.r_[old_xyz, old_m_xyz]
            elif old_m_xyz.any():
                old_xyz = old_m_xyz
        
            if new_xyz.any() and new_m_xyz.any():
                new_xyz = np.r_[new_xyz, new_m_xyz]
            elif new_m_xyz.any():
                new_xyz = new_m_xyz
                
        if new_xyz.any() and new_xyz.shape != (4,4):
            #handles.append(Globals.env.plot3(old_xyz,5, (1,0,0,1)))
            #handles.append(Globals.env.plot3(old_xyz,5, np.array([(1,0,0) for _ in xrange(old_xyz.shape[0])])))
            handles.append(Globals.env.plot3(new_xyz,5, np.array([(0,0,1) for _ in xrange(old_xyz.shape[0])])))

        print 'Old points:', old_xyz
        print 'New points:', new_xyz
        
        #if not yes_or_no.yes_or_no("Use identity?"):
        if rigid:
            f = registration.ThinPlateSpline()
            rel_tfm = new_xyz.dot(np.linalg.inv(old_xyz))
            f.init_rigid_tfm(rel_tfm)
        elif len(new_xyz) > 0:
            #f.fit(demopoints_m3, newpoints_m3, 10,10)
            # TODO - check if this regularization on bending is correct
            if "right_hole_normal" in new_keypoints or "left_hole_normal" in new_keypoints:
                bend_c = 0.05
                rot_c = [1e-5,1e-5,0.1]
                wt = 5
                wt_n = np.ones(len(old_xyz))
                if kp_mapping.get("right_hole_normal"):
                    wt_n[kp_mapping["right_hole_normal"][0]] = wt
                if kp_mapping.get("left_hole_normal"):
                    wt_n[kp_mapping["left_hole_normal"][0]] = wt
                print "Found hole normals"
            else:
                bend_c = 0.05
                rot_c = 1e-5#[0,0,1e-5]
                wt_n = None
            f = registration.fit_ThinPlateSpline(old_xyz, new_xyz, bend_coef = bend_c,rot_coef = rot_c)
            np.set_printoptions(precision=3)
            print "nonlinear part", f.w_ng
            print "affine part", f.lin_ag
            print "translation part", f.trans_g
            print "residual", f.transform_points(old_xyz) - new_xyz
            print "max error ", np.max(np.abs(f.transform_points(old_xyz) - new_xyz))
        else:
            f = registration.ThinPlateSpline()
            bend_c = 0
            rot_c = 0
#         else:
#             f = registration.ThinPlateSpline()            
#             bend_c = 0
#             rot_c = 0
#         
        if old_xyz.any() and old_xyz.shape != (4,4):
            tfm_xyz = f.transform_points(old_xyz)
            handles.append(Globals.env.plot3(tfm_xyz,5, np.array([(0,1,0) for _ in xrange(tfm_xyz.shape[0])])))
        
        #f = registration.ThinPlateSpline() XXX XXX
        
        if new_xyz.any() and new_xyz.shape != (4,4):
            handles.extend(plotting_openrave.draw_grid(Globals.env, f.transform_points, old_xyz.min(axis=0), old_xyz.max(axis=0), xres = .1, yres = .1, zres = .04))

        if args.ask:
            if new_xyz.any() and new_xyz.shape != (4,4):
                import visualize
                visualize.plot_tfm(old_xyz, new_xyz, bend_c, rot_c)
                lines = plotting_openrave.gen_grid(f.transform_points, np.array([0,-1,0]), np.array([1,1,1]))
                plotting_openrave.plot_lines(lines)    
        
        miniseg_starts, miniseg_ends = split_trajectory_by_gripper(seg_info)
        success = True
        print colorize.colorize("mini segments:", "red"), miniseg_starts, miniseg_ends

        # Assuming only lgrab or rgrab
        use_needle = None
        for lr in 'lr':      
            if "%s_grab"%lr in seg_info['extra_information']:
                if "needle_tip_transform" in seg_info['key_points']:
                    demo_ntfm = np.array(seg_info['key_points']['needle_tip_transform'])
                    ntfm = np.array(new_keypoints['needle_tip_transform'])
                    use_needle = lr
                elif Globals.rel_ntfm is not None:
                    T_w_g = Globals.robot.GetLink("%s_gripper_tool_frame"%lr).GetTransform()
                    T_g_n = Globals.rel_ntfm
                    ntfm = T_w_g.dot(T_g_n)
                    T_w_g_demo = Globals.demo_robot.GetLink("%s_gripper_tool_frame"%lr).GetTransform()
                    T_g_n_demo = Globals.demo_rel_ntfm
                    demo_ntfm = T_w_g_demo.dot(T_g_n_demo)
                    use_needle = lr

        if use_needle is not None:
            setup_needle_grabs(use_needle, seg_info["joint_states"]["name"], seg_info["joint_states"]["look_position"], demo_ntfm, ntfm)
        
        for (i_miniseg, (i_start, i_end)) in enumerate(zip(miniseg_starts, miniseg_ends)):
            
            if args.execution=="real": Globals.pr2.update_rave()


            # Changing the trajectory according to the end-effector
            full_traj = np.c_[seg_info["leftarm"][i_start:i_end+1], seg_info["rightarm"][i_start:i_end+1]]
            full_traj = mu.remove_duplicate_rows(full_traj)
            _, ds_traj =  resampling.adaptive_resample(full_traj, tol=.01, max_change=.1) # about 2.5 degrees, 10 degrees

            Globals.robot.SetActiveDOFs(np.r_[Globals.robot.GetManipulator("leftarm").GetArmIndices(), Globals.robot.GetManipulator("rightarm").GetArmIndices()])
            Globals.demo_robot.SetActiveDOFs(np.r_[Globals.robot.GetManipulator("leftarm").GetArmIndices(), Globals.robot.GetManipulator("rightarm").GetArmIndices()])        
            Globals.demo_robot.SetDOFValues(Globals.robot.GetDOFValues())
        

            ################################    
            redprint("Generating joint trajectory for segment %s, part %i"%(seg_name, i_miniseg))

            bodypart2traj = {}
            
            arms_used = ""
        
            for lr in 'lr':
                
#                 if "%s_grab"%lr in seg_info['extra_information']:
#                     if "needle_tip_transform" in seg_info['key_points']:
#                         demo_ntfm = np.array(seg_info['key_points']['needle_tip_transform'])
#                         ntfm = np.array(new_keypoints['needle_tip_transform'])
#                         use_needle = True
#                     elif Globals.rel_ntfm is not None:
#                         T_w_g = Globals.robot.GetLink("%s_gripper_tool_frame"%lr).GetTransform()
#                         T_g_n = Globals.rel_ntfm
#                         ntfm = T_w_g.dot(T_g_n)
#                         T_w_g_demo = Globals.demo_robot.GetLink("%s_gripper_tool_frame"%lr).GetTransform()
#                         T_g_n_demo = Globals.demo_rel_ntfm
#                         demo_ntfm = T_w_g_demo.dot(T_g_n_demo)
#                         use_needle = True
#                     else:
#                         use_needle = False
#                 else:
#                     use_needle = False
                
                if use_needle == lr:
                    Globals.sponge.Enable(False)
                    old_ee_traj = setup_needle_traj (lr, ds_traj)
                    link = Globals.needle_tip.GetLinks()[0]
                    
                    redprint("Using needle for trajectory execution...")

                else:
                    Globals.sponge.Enable(True)
                    ee_link_name = "%s_gripper_tool_frame"%lr
                    link = Globals.robot.GetLink(ee_link_name)
                    demo_link = Globals.demo_robot.GetLink(ee_link_name)
                    old_ee_traj = []
                    for row in ds_traj:
                        Globals.demo_robot.SetActiveDOFValues(row)
                        old_ee_traj.append(demo_link.GetTransform())
                
############ CAN YOU PLOT THIS OLD_EE_TRAJ?        
                old_ee_traj = np.asarray(old_ee_traj)
                old_joint_traj = {'l':ds_traj[:,:7], 'r':ds_traj[:,7:]}[lr]

                if arm_moved(old_joint_traj):
                    
                    manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
                    new_ee_traj = f.transform_hmats(old_ee_traj)
                    
#################### CAN YOU PLOT THIS NEW_EE_TRAJ?
                    
                    handles.append(Globals.env.drawlinestrip(old_ee_traj[:,:3,3], 2, (1,0,0,1)))
                    handles.append(Globals.env.drawlinestrip(new_ee_traj[:,:3,3], 2, (0,1,0,1)))
                    #old_poses = [conv.hmat_to_pose(hmat) for hmat in old_ee_traj]
                    #print new_ee_traj
                    #new_poses = [conv.hmat_to_pose(hmat) for hmat in new_ee_traj]
                    update_markers(new_ee_traj, old_ee_traj)
                    #thc.run()
                    

                    if args.execution: Globals.pr2.update_rave()
                    new_joint_traj = plan_follow_traj(Globals.robot, manip_name,
                                                      link, new_ee_traj, old_joint_traj)
                    new_joint_traj = unwrap_joint_traj (lr, new_joint_traj)
                    # (robot, manip_name, ee_link, new_hmats, old_traj):
                    part_name = {"l":"larm", "r":"rarm"}[lr]
                    bodypart2traj[part_name] = new_joint_traj
                    
                    arms_used += lr


            ################################    
            redprint("Executing joint trajectory for segment %s, part %i using arms '%s'"%(seg_name, i_miniseg, arms_used))

            for lr in 'lr':
                set_gripper_maybesim(lr, binarize_gripper(seg_info["%s_gripper_joint"%lr][i_start]))
            #trajoptpy.GetViewer(Globals.env).Idle()

            if len(bodypart2traj) > 0:
                exec_traj_maybesim(bodypart2traj, speed_factor=1)
        
            # TODO measure failure condtions
            Globals.robot.ReleaseAllGrabbed()

            if not success:
                break
            
            
        redprint("Segment %s result: %s"%(seg_name, success))
    
        if args.fake_data_segment: break

def openloop():
    
    demofile = h5py.File(args.h5file, 'r')
    rospy.init_node("exec_task",disable_signals=True)
    
    if args.execution:
        Globals.pr2 = PR2.PR2()
        Globals.env = Globals.pr2.env
        Globals.robot = Globals.pr2.robot
        
    else:
        Globals.env = openravepy.Environment()
        Globals.env.StopSimulation()
        Globals.env.Load("robots/pr2-beta-static.zae")
        Globals.robot = Globals.env.GetRobots()[0]
    
    while True:
        seg_name = select_segment(demofile)
        seg_info = demofile[seg_name]
     
        miniseg_starts, miniseg_ends = split_trajectory_by_gripper(seg_info)
        print colorize.colorize("mini segments:", "red"), miniseg_starts, miniseg_ends
        for (i_miniseg, (i_start, i_end)) in enumerate(zip(miniseg_starts, miniseg_ends)):
            
            ################################    
            redprint("Generating joint trajectory for segment %s, part %i"%(seg_name, i_miniseg))

            bodypart2traj = {}
            arms_used = ""
            
            # Not sure about frequency - look into this
            for lr in 'lr':
                manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
                _, joint_traj = resampling.adaptive_resample(asarray(seg_info[manip_name][i_start:i_end+1]), tol=0.01, max_change=.1)
                print "Shape of %s traj: "%lr, joint_traj.shape
                if arm_moved(joint_traj):
                    part_name = {"l":"larm", "r":"rarm"}[lr]
                    bodypart2traj[part_name] = joint_traj
                    arms_used += lr
        
            redprint("Executing open-loop trajectory for segment %s, part %i using arms '%s'"%(seg_name, i_miniseg, arms_used))
            
            for lr in 'lr':
                set_gripper_maybesim(lr, binarize_gripper(seg_info["%s_gripper_joint"%lr][i_start]))
               
            print len(bodypart2traj) 
            if len(bodypart2traj) > 0:
                exec_traj_maybesim(bodypart2traj, speed_factor=0.1)

if args.openloop:
    openloop()
else:
    main()
