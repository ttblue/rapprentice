#!/usr/bin/env python

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("h5file", type=str)

parser.add_argument("--openloop", type=int, default=0)
parser.add_argument("--execution", type=int, default=0)
parser.add_argument("--animation", type=int, default=0)

parser.add_argument("--fake_data_segment",type=str)
parser.add_argument("--fake_data_transform", type=float, nargs=6, metavar=("tx","ty","tz","rx","ry","rz"),
    default=[0,0,0,0,0,0], help="translation=(tx,ty,tz), axis-angle rotation=(rx,ry,rz)")

parser.add_argument("--interactive",action="store_true")

args = parser.parse_args()

if args.fake_data_segment is None: assert args.execution==1

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


from rapprentice import registration, colorize, berkeley_pr2, \
     animate_traj, ros2rave, plotting_openrave, task_execution, resampling
from rapprentice import pr2_trajectories, retiming, math_utils as mu
from pr2 import PR2
import rospy

import cloudprocpy, trajoptpy, json, openravepy
import numpy as np, h5py
from numpy import asarray

import find_keypoints as fk   

    
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


    
def plan_follow_traj(robot, manip_name, ee_link, new_hmats, old_traj):
        
    n_steps = len(new_hmats)
    assert old_traj.shape[0] == n_steps
    assert old_traj.shape[1] == 7
    
    arm_inds  = robot.GetManipulator(manip_name).GetArmIndices()
    robot.SetActiveDOFs(arm_inds)

    ee_linkname = ee_link.GetName()
    
    init_traj = old_traj.copy()
    #init_traj[0] = robot.GetDOFValues(arm_inds)

    request = {
        "basic_info" : {
            "n_steps" : n_steps,
            "manip" : manip_name,
            "start_fixed" : False
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
                "rot_coeff":[20,20,20]
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
        
def exec_traj_maybesim(bodypart2traj):
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
        return True
    if args.execution:
        pr2_trajectories.follow_body_traj(Globals.pr2, bodypart2traj, speed_factor=0.5)
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
###################


class Globals:
    # Rave
    env = None
    robot = None
    demo_env = None
    demo_robot = None
    
    # Needle tip
    needle_tip = None
    demo_needle_tip = None

    # PR2 control
    pr2 = None
    
    

def setup_needle_traj (lr, arm_traj, old_tfm, new_tfm):
    
    Globals.demo_needle_tip.SetTransform(old_tfm)
    Globals.demo_robot.Grab(Globals.demo_needle_tip, Globals.demo_robot.GetLink("%s_gripper_tool_frame"%lr))
    Globals.needle_tip.SetTransform(new_tfm)
    Globals.robot.Grab(Globals.needle_tip, Globals.robot.GetLink("%s_gripper_tool_frame"%lr))
    
    needle_traj = []
    for row in arm_traj:
        Globals.demo_robot.SetActiveDOFValues(row)
        needle_traj.append(Globals.demo_needle_tip.GetTransform())
    
    return needle_traj
    

def main():

    demofile = h5py.File(args.h5file, 'r')
    
    trajoptpy.SetInteractive(args.interactive)

    if args.execution:
        rospy.init_node("exec_task",disable_signals=True)
        Globals.pr2 = PR2.PR2()
        Globals.env = Globals.pr2.env
        Globals.robot = Globals.pr2.robot
        
    else:
        Globals.env = openravepy.Environment()
        Globals.env.StopSimulation()
        Globals.env.Load("robots/pr2-beta-static.zae")
        Globals.robot = Globals.env.GetRobots()[0]
        
    Globals.needle_tip = trajoptpy.make_kinbodies.create_spheres(Globals.env, [(0,0,0)], radii=0.02, name="needle_tip")
    Globals.demo_env = Globals.env.CloneSelf(1)
    Globals.demo_env.StopSimulation()
    Globals.demo_robot = Globals.demo_env.GetRobot("pr2")
    Globals.demo_needle_tip = Globals.demo_env.GetKinBody("needle_tip")
    Globals.env.Load("/home/sibi/sandbox/rapprentice/objects/table.xml")

    if not args.fake_data_segment:
        grabber = cloudprocpy.CloudGrabber()
        grabber.startRGBD()

    Globals.viewer = trajoptpy.GetViewer(Globals.env)

    #####################

    while True:
        
    
        redprint("Acquire point cloud")
        if args.fake_data_segment:
            new_keypoints = np.squeeze(demofile[args.fake_data_segment]["key_points"])
        else:    
            Globals.pr2.rarm.goto_posture('side')
            Globals.pr2.larm.goto_posture('side')            
            Globals.pr2.join_all()
            
            Globals.pr2.update_rave()
            
            T_w_k = berkeley_pr2.get_kinect_transform(Globals.robot)            

    
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
        
        # TODO: Have a check for only transformations -> rigid transformations
        if args.fake_data_segment:
            if seg_info['key_points'].sort() != new_keypoints.sort():
                print "Keypoints don't match."
                exit(1)
            old_xyz, rigid = fk.key_points_to_points(seg_info['key_points'])
            new_xyz, _ = fk.key_points_to_points(new_keypoints)
        else:
            new_keypoints = fk.get_keypoints_execution(grabber, seg_info['key_points'].keys(), T_w_k)
            old_xyz, rigid = fk.key_points_to_points(seg_info['key_points'])
            new_xyz, _ = fk.key_points_to_points(new_keypoints)
        
        
        handles.append(Globals.env.plot3(old_xyz,5, (1,0,0,1)))
        handles.append(Globals.env.plot3(new_xyz,5, (0,0,1,1)))


        if rigid:
            f = registration.ThinPlateSpline()
            rel_tfm = new_xyz.dot(np.linalg.inv(old_xyz))
            f.init_rigid_tfm(rel_tfm)
        elif len(new_xyz) > 0:
                f = registration.ThinPlateSpline()
                #f.fit(demopoints_m3, newpoints_m3, 10,10)
                # TODO - check if this regularization on bending is correct
                f.fit(old_xyz, new_xyz, bend_coef=0.01,rot_coef=.01)
                np.set_printoptions(precision=3)
                print "nonlinear part", f.w_ng
                print "affine part", f.lin_ag
                print "translation part", f.trans_g
                print "residual", f.transform_points(old_xyz) - new_xyz
        else:
            f = registration.ThinPlateSpline()            
        
        #f = registration.ThinPlateSpline() XXX XXX
        
        handles.extend(plotting_openrave.draw_grid(Globals.env, f.transform_points, old_xyz.min(axis=0), old_xyz.max(axis=0), xres = .1, yres = .1, zres = .04))
        
        
        # TODO plot
        # plot_warping_and_trajectories(f, old_xyz, new_xyz, old_ee_traj, new_ee_traj)
    
        use_needle = "%s_grab" in seg_info["extra_information"] and "needle_tip_transform" in seg_info['key_points']
    
        miniseg_starts, miniseg_ends = split_trajectory_by_gripper(seg_info)
        success = True
        print colorize.colorize("mini segments:", "red"), miniseg_starts, miniseg_ends
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
                if use_needle:
                    old_ee_traj = setup_needle_traj (lr, ds_traj, seg_info["key_points"]["needle_tip_transform"])
                    link = Globals.needle_tip.GetLinks()[0]
                else:
                    ee_link_name = "%s_gripper_tool_frame"%lr
                    link = Globals.robot.GetLink(ee_link_name)
                    old_ee_traj = asarray(seg_info[ee_link_name]["hmat"])
                    
                old_joint_traj = {'l':ds_traj[:,:7], 'r':ds_traj[:,7:]}[lr]

                if arm_moved(old_joint_traj):
                    
                    manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
                    new_ee_traj = f.transform_hmats(old_ee_traj)                
                    handles.append(Globals.env.drawlinestrip(old_ee_traj[:,:3,3], 2, (1,0,0,1)))
                    handles.append(Globals.env.drawlinestrip(new_ee_traj[:,:3,3], 2, (0,1,0,1)))
                
                    if args.execution: Globals.pr2.update_rave()
                    new_joint_traj = plan_follow_traj(Globals.robot, manip_name,
                                                      link, new_ee_traj, old_joint_traj)
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
                exec_traj_maybesim(bodypart2traj)
        
            # TODO measure failure condtions

            if not success:
                break
            
        redprint("Segment %s result: %s"%(seg_name, success))
    
    
        if args.fake_data_segment: break

def openloop():
    
    demofile = h5py.File(args.h5file, 'r')
    
    if args.execution:
        rospy.init_node("exec_task",disable_signals=True)
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
            
            freq = 5
            # Not sure about frequency - look into this
            for lr in 'lr':
                manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
                time_length = np.ptp(seg_info["stamps"])
                num = np.round(time_length*freq)
                print "Number of steps: ", num
                _, joint_traj = resampling.adaptive_resample(asarray(seg_info[manip_name][i_start:i_end+1]), tol=0.01, max_change=.1, min_steps=num)
                if arm_moved(joint_traj):
                    part_name = {"l":"larm", "r":"rarm"}[lr]
                    bodypart2traj[part_name] = joint_traj
                    arms_used += lr
        
            redprint("Executing open-loop trajectory for segment %s, part %i using arms '%s'"%(seg_name, i_miniseg, arms_used))
            
            for lr in 'lr':
                set_gripper_maybesim(lr, binarize_gripper(seg_info["%s_gripper_joint"%lr][i_start]))
                
            if len(bodypart2traj) > 0:
                exec_traj_maybesim(bodypart2traj)

if args.openloop:
    openloop()
else:
    main()
