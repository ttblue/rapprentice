#f22sarrrom visualize import np_ps
import numpy as np
from rapprentice import registration
from find_keypoints import key_points_to_points as kptp
import sys

assert len(sys.argv) == 6

bend_c = float(sys.argv[1])
rot_c = [float(sys.argv[2]),float(sys.argv[3]),float(sys.argv[4])]
scale_c = float(sys.argv[5])

def np_ps (name):
    import yaml
    with open(name,'r') as fh: pf = yaml.load(fh)

    for k in pf:
        pf[k] = np.array(pf[k][0])
        pf[k] = pf[k][[1,2,3,5,6,7]]#,8,9,10,11]]

    return pf

pf = np_ps("test/pierce2.yaml")

import h5py
de = h5py.File('/home/sibi/sandbox/rapprentice/data/final.h5', 'r')
seg = de['2_pierce0_seg01']
kp = dict(seg['key_points'])
new_kp = {'left_hole_normal': list(kp['left_hole_normal']),
	  'right_hole_normal': list(kp['right_hole_normal'])}
#print new_kp
old_xyz = np.asarray(kptp(new_kp, False)[0])
print old_xyz
ms = []
inds = range(len(pf))
#for i in inds:
#    m = 0
#    for j in inds:
#        old_xyz = pf[i]
#        new_xyz = pf[j]
#        f = registration.fit_ThinPlateSpline_RotReg(old_xyz, new_xyz, bend_coef=bend_c,rot_coefs=rot_c,scale_coef=scale_c)
#        errs = f.transform_points(old_xyz) - new_xyz
#        err = np.max(np.abs(errs))
#	m = np.max([err,m])
#        print "(%i, %i)"%(i,j)," max err:", err
        #print old_xyz, new_xyz
#    ms.append(m)

m=0
for j in inds:
#    old_xyz = pf[i]
    new_xyz = pf[j]
    f = registration.fit_ThinPlateSpline_RotReg(old_xyz, new_xyz, bend_coef=bend_c,rot_coefs=rot_c,scale_coef=scale_c)
    errs = f.transform_points(old_xyz) - new_xyz
    err = np.max(np.abs(errs))
    m = np.max([err,m])
    print j, "max err:", err


print "Maximum error for each old_xyz with others: ", m
#print "Maximum error over all: ", np.max(ms)
