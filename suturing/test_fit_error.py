import visualize
import numpy as np
from rapprentice import registration
bend_c = 0.1
rot_c = [0.1,0.1,0.1]

pf = visualize.np_point_sets("test/normals.yaml")

m = 0
for i in xrange(len(pf)):
    for j in xrange(len(pf)):
        old_xyz = pf[i]
        new_xyz = pf[j]
        f = registration.fit_ThinPlateSpline(old_xyz, new_xyz, bend_coef=bend_c,rot_coef=rot_c)
        errs = f.transform_points(old_xyz) - new_xyz
        err = np.max(np.abs(errs))
	m = np.max([err,m])
        print "max err:", err
        print old_xyz, new_xyz
    
print "Maximum error over all: ", m
