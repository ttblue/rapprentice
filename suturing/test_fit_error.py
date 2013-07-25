import visualize
import numpy as np
from rapprentice import registration
bend_c = 0
rot_c = 0

pf = visualize.np_point_sets("rot_test")

for i in xrange(len(pf)):
    for j in xrange(len(pf)):
        old_xyz = pf[i]
        new_xyz = pf[j]
        f = registration.fit_ThinPlateSpline(old_xyz, new_xyz, bend_coef=bend_c,rot_coef=rot_c)
        errs = f.transform_points(old_xyz) - new_xyz
        err = np.max(np.abs(errs))
        print "max err:", err
    