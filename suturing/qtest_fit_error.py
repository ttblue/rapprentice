from visualize import np_point_sets2
import numpy as np
from rapprentice import registration
import sys

assert len(sys.argv) == 6

bend_c = float(sys.argv[1])
rot_c = [float(sys.argv[2]),float(sys.argv[3]),float(sys.argv[4])]
scale_c = float(sys.argv[5])

pf = np_point_sets2("test/pierce2.yaml")

ms = []
inds = range(len(pf))
for i in inds:
    m = 0
    for j in inds:
        old_xyz = pf[i]
        new_xyz = pf[j]
        f = registration.fit_ThinPlateSpline_RotReg(old_xyz, new_xyz, bend_coef=bend_c,rot_coefs=rot_c,scale_coef=scale_c)
        errs = f.transform_points(old_xyz) - new_xyz
        err = np.max(np.abs(errs))
	m = np.max([err,m])
        print "(%i, %i)"%(i,j)," max err:", err
        #print old_xyz, new_xyz
    ms.append(m)

print "Maximum error for each old_xyz with others: ", ms
print "Maximum error over all: ", np.max(ms)
