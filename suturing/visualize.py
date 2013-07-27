from rapprentice import plotting_openrave as po, registration, yes_or_no, PR2, berkeley_pr2
from suturing import suturing_visualization_interface as svi, transform_finder as tff
import cloudprocpy

import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def identity (a):
    return a

wt = 2
n_wt = np.ones(13)
n_wt[0], n_wt[3] = wt, wt 


def plot_mesh(f, mins, maxes, xres = .1, yres = .1, zres = .04):
    
    fig = matplotlib.pyplot.figure()
    ax = Axes3D(fig)
    #ax  = fig.add_subplot(111, projection = '3d')
    
    lines = po.grid_lines(f, mins, maxes, xres, yres, zres)
    
    for line in lines:
        ax.plot(line[:,0].tolist(), line[:,1].tolist(), line[:,2].tolist(), color='b')
        #for i in xrange(line.shape[0]-1):
        #    p1, p2 = line[i], line[i+1]
        #    ax.plot([p1[0],p2[0]], [p1[1],p2[1]], [p1[2],p2[2]])
        
    matplotlib.pyplot.show()

def plot_mesh_points(f, old_xyz, new_xyz):
    
    fig = matplotlib.pyplot.figure()
    fig.patch.set_facecolor('black')
    ax = Axes3D(fig)
    #ax  = fig.add_subplot(111, projection = '3d')
    
    lines = po.grid_lines(f, old_xyz.min(axis=0), old_xyz.max(axis=0), xres = .1, yres = .1, zres = .04)
    
    for line in lines:
        ax.plot(line[:,0].tolist(), line[:,1].tolist(), line[:,2].tolist(), color='y')
        #for i in xrange(line.shape[0]-1):
        #    p1, p2 = line[i], line[i+1]
        #    ax.plot([p1[0],p2[0]], [p1[1],p2[1]], [p1[2],p2[2]])
        
    tfm_xyz = f(old_xyz)
    
    ax.scatter(old_xyz[:,0].tolist(), old_xyz[:,1].tolist(), old_xyz[:,2].tolist(), color='r')
    ax.scatter(tfm_xyz[:,0].tolist(), tfm_xyz[:,1].tolist(), tfm_xyz[:,2].tolist(), color='g')
    ax.scatter(new_xyz[:,0].tolist(), new_xyz[:,1].tolist(), new_xyz[:,2].tolist(), color='b')
        
    matplotlib.pyplot.show()


def plot_tfm (old_xyz, new_xyz, bend_c, rot_c, wt_n=None):
    
    f = registration.fit_ThinPlateSpline(old_xyz, new_xyz, bend_coef=bend_c,rot_coef=rot_c, wt_n=wt_n)
    print "nonlinear part", f.w_ng
    print "affine part", f.lin_ag
    print "translation part", f.trans_g
    print "residual", f.transform_points(old_xyz) - new_xyz
    print "max error ", np.max(np.abs(f.transform_points(old_xyz) - new_xyz))
    plot_mesh_points(f.transform_points, old_xyz, new_xyz)
    

def store_in_yaml (name):
    
    import yaml, rospy
    rospy.init_node("store_points", disable_signals = True)
    
    #subprocess.call("killall XnSensorServer", shell=True)

    pr2 = PR2.PR2()
    grabber = cloudprocpy.CloudGrabber()
    grabber.startRGBD()

    n = input("Enter the number of points you are saving: ")
    
    point_sets = {}

    d = 0
    while True:
        print "Demo %i"%(d+1)
        
        point_sets[d] = []
        
        T_w_k = berkeley_pr2.get_kinect_transform(pr2.robot)
        xyz_tf, rgb = svi.get_kp_clouds(grabber, 1, T_w_k)
        for _ in xrange(n):
            print "Save point set (make sure you click on them in the same order every situation)."
            
            point = svi.find_kp_single_cloud("nt", xyz_tf, rgb)
            point_sets[d].append([float(point[0]), float(point[1]), float(point[2])])
            
        if not yes_or_no.yes_or_no("Do you want to save another situation?"):
            break
        
        d += 1
    
    print point_sets
    
    with open(name, 'w') as fl:    
        yaml.dump(point_sets, fl)


def store_normals_in_yaml (name):
    
    import yaml, rospy
    
    #subprocess.call("killall XnSensorServer", shell=True)

    rospy.init_node("store_normals", disable_signals = True)
    pr2 = PR2.PR2()
    grabber = cloudprocpy.CloudGrabber()
    grabber.startRGBD()

    point_sets = {}

    d = 0
    dist = 0.1
    while True:
        print "Demo %i"%(d+1)
        
        point_sets[d] = []
        T_w_k = berkeley_pr2.get_kinect_transform(pr2.robot)
        
        for i in [1,2]:
            print "Save hole normal %i:"%i
            kp_loc = svi.find_kp_execution('hn%i'%i, grabber, T_w_k)
            p = kp_loc[0]
            n = kp_loc[1]
            point_sets[d].append([p[0], p[1], p[2]])
            point_sets[d].append([p[0]+dist*n[0]/2, p[1]+dist*n[1]/2, p[2]+dist*n[2]/2])
            point_sets[d].append([p[0]+dist*n[0], p[1]+dist*n[1], p[2]+dist*n[2]])

            
        if not yes_or_no.yes_or_no("Do you want to save another situation?"):
            break
        
        d += 1
    
    print point_sets
    
    with open(name, 'w') as fl:    
        yaml.dump(point_sets, fl)
        
        
def update_normals (name):
    
    import yaml
    with open(name, "r") as fh: pf = yaml.load(fh)
    for k in pf:
        h1 = np.array(pf[k][0])
        h2 = np.array(pf[k][3])
        for alpha in [0.2,0.4,0.6,0.8]:
            pf[k].append((h1*alpha + h2*(1-alpha)).tolist())
    
    with open(name, "w") as fh: yaml.dump(pf, fh)


def update_normals2 (name):
    
    import yaml
    with open(name, "r") as fh: pf = yaml.load(fh)
    dist = 0.1
    for k in pf:
        h1 = np.array(pf[k][0])
        h2 = np.array(pf[k][3])
        h = h2 - h1
        hm = (h1+h2)/2
        n1 = np.array(pf[k][2]) - np.array(pf[k][0])
        n2 = np.array(pf[k][5]) - np.array(pf[k][3])
        n = (n1+n2)/2
    
        x = np.cross(h,n)
        x = x/np.linalg.norm(x)

        pf[k].append(hm.tolist())
        pf[k].append(hm+dist*x)
        pf[k].append(hm-dist*x)
        
        
    with open(name, "w") as fh: yaml.dump(pf, fh)
        
        
        
def np_point_sets (name):
    
    import yaml
    with open(name, "r") as fh: pf = yaml.load(fh)
    
    for k in pf:
        pf[k] = np.array(pf[k])

    return pf

def set_weight(wt_):
    global wt
    wt = wt_
    n_wt[0], n_wt[3] = wt, wt