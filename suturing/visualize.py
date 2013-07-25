from rapprentice import plotting_openrave as po, registration, yes_or_no
from suturing import suturing_visualization_interface as svi
import cloudprocpy

import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def identity (a):
    return a

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
    
    ax.scatter(old_xyz[:,0].tolist(), old_xyz[:,1].tolist(), old_xyz[:,2].tolist(), color='b')
    ax.scatter(tfm_xyz[:,0].tolist(), tfm_xyz[:,1].tolist(), tfm_xyz[:,2].tolist(), color='g')
    ax.scatter(new_xyz[:,0].tolist(), new_xyz[:,1].tolist(), new_xyz[:,2].tolist(), color='r')
        
    matplotlib.pyplot.show()


def plot_tfm (old_xyz, new_xyz, bend_c, rot_c):
    
    f = registration.fit_ThinPlateSpline(old_xyz, new_xyz, bend_coef=bend_c,rot_coef=rot_c)
    plot_mesh_points(f.transform_points, old_xyz, new_xyz)
    

def store_in_yaml (name):
    
    import yaml
    
    #subprocess.call("killall XnSensorServer", shell=True)

    grabber = cloudprocpy.CloudGrabber()
    grabber.startRGBD()

    n = input("Enter the number of points you are saving: ")
    
    point_sets = {}

    d = 0
    while True:
        print "Demo %i"%(d+1)
        
        point_sets[d] = []
        
        xyz_tf, rgb = svi.get_kp_clouds(grabber, 1, np.eye(4))        
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
        
def np_point_sets (name):
    
    import yaml
    with open(name, "r") as fh: pf = yaml.load(fh)
    
    for k in pf:
        pf[k] = np.array(pf[k])

    return pf