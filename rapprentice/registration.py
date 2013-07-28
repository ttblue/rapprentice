"""
Register point clouds to each other


arrays are named like name_abc
abc are subscripts and indicate the what that tensor index refers to

index name conventions:
    m: test point index
    n: training point index
    a: input coordinate
    g: output coordinate
    d: gripper coordinate
"""

from __future__ import division
import numpy as np
import scipy.spatial.distance as ssd
from rapprentice import tps, svds, math_utils
# from svds import svds



class Transformation(object):
    """
    Object oriented interface for transformations R^d -> R^d
    """
    def transform_points(self, x_ma):
        raise NotImplementedError
    def compute_jacobian(self, x_ma):
        raise NotImplementedError        

        
    def transform_bases(self, x_ma, rot_mad, orthogonalize=True, orth_method = "cross"):
        """
        orthogonalize: none, svd, qr
        """

        grad_mga = self.compute_jacobian(x_ma)
        newrot_mgd = np.array([grad_ga.dot(rot_ad) for (grad_ga, rot_ad) in zip(grad_mga, rot_mad)])
        

        if orthogonalize:
            if orth_method == "qr": 
                newrot_mgd =  orthogonalize3_qr(newrot_mgd)
            elif orth_method == "svd":
                newrot_mgd = orthogonalize3_svd(newrot_mgd)
            elif orth_method == "cross":
                newrot_mgd = orthogonalize3_cross(newrot_mgd)
            else: raise Exception("unknown orthogonalization method %s"%orthogonalize)
        return newrot_mgd
        
    def gripper_transform_bases(self, points, old_rotations, orthogonalize=True, orth_method = "cross"):
        #1. Create small segments for the axes perpendicular to the gripper
        #2. Find the transform of these segements and their vector representations
        #3. Find an orthagonal set of bases for these vectors
        # col(axis) 0 = red = into table
        # col(axis) 1 = green = axis between grippers 
        # col(axis) 2 = blue = in direction of rope
        
        #Calculating segments
        seg_length = .001
        axis_0_segments = [(point - seg_length*axis, point + seg_length*axis) for point, axis in zip(points, old_rotations[:,:,0])]
        axis_2_segments = [(point - seg_length*axis, point + seg_length*axis) for point, axis in zip(points, old_rotations[:,:,2])]
        print "axis_0_segments", axis_0_segments
        print "axis_2_segments", axis_2_segments
        axis_0_rotation = self.compute_rotation(axis_0_segments)
        axis_2_rotation = self.compute_rotation(axis_2_segments)
        new_rotations = [orthogonalize_gripper(axis_0, axis_2) for axis_0, axis_2 in zip(axis_0_rotation, axis_2_rotation)]
        return new_rotations
        
    def compute_rotation(self, segments):
        #segments is a list of tuples
        #returns the normalized vector of the transformed second point - the transformed first point
        starts = np.array([seg[0] for seg in segments])
        ends = np.array([seg[1] for seg in segments])
        transformed_starts = self.transform_points(starts)
        transformed_ends = self.transform_points(ends)
        vectors = [math_utils.normalize(end - start) for start, end in zip(transformed_starts, transformed_ends)]
        return vectors

    def transform_hmats(self, hmat_mAD):
        """
        Transform (D+1) x (D+1) homogenius matrices
        """
        hmat_mGD = np.empty_like(hmat_mAD)
        hmat_mGD[:,3,3] = 1
        hmat_mGD[:,3,2] = 0
        hmat_mGD[:,3,1] = 0
        hmat_mGD[:,3,0] = 0
        hmat_mGD[:,:3,3] = self.transform_points(hmat_mAD[:,:3,3])
        #Modify transform_bases
        hmat_mGD[:,:3,:3] = self.transform_bases(hmat_mAD[:,:3,3], hmat_mAD[:,:3,:3])
        #hmat_mGD[:,:3,:3] = self.gripper_transform_bases(hmat_mAD[:,:3,3], hmat_mAD[:,:3,:3])
        return hmat_mGD        
    def compute_numerical_jacobian(self, x_d, epsilon=0.0001):
        "numerical jacobian"
        x0 = np.asfarray(x_d)
        f0 = self.transform_points(x0)
        jac = np.zeros(len(x0), len(f0))
        dx = np.zeros(len(x0))
        for i in range(len(x0)):
            dx[i] = epsilon
            jac[i] = (self.transform_points(x0+dx) - f0) / epsilon
            dx[i] = 0.
        return jac.transpose()

class ThinPlateSpline(Transformation):
    """
    members:
        x_na: centers of basis functions
        w_ng: 
        lin_ag: transpose of linear part, so you take x_na.dot(lin_ag)
        trans_g: translation part
    
    """
    def __init__(self, d=3):
        "initialize as identity"
        self.x_na = np.zeros((0,d))
        self.lin_ag = np.eye(d)
        self.trans_g = np.zeros(d)
        self.w_ng = np.zeros((0,d))
        
    def init_rigid_tfm (self, tfm, dim=3):
        """
        Assuming dimension = 3.
        """
        if dim != 3:
            raise NotImplementedError("Dimension other than 3 not supported.")
        self.lin_ag = tfm[0:3,0:3].T
        self.trans_g = tfm[0:3,3].T
        self.x_na = np.zeros((0,3))
        self.w_ng = np.zeros((0,3))
        

    def transform_points(self, x_ma):
        y_ng = tps.tps_eval(x_ma, self.lin_ag, self.trans_g, self.w_ng, self.x_na)
        return y_ng
    def compute_jacobian(self, x_ma):
        grad_mga = tps.tps_grad(x_ma, self.lin_ag, self.trans_g, self.w_ng, self.x_na)
        return grad_mga
    def fitting_cost (self, y_ng, bend_coef):
        return tps.tps_cost(self.lin_ag, self.trans_g, self.w_ng, self.x_na, y_ng, bend_coef, return_tuple=True)
        
class Affine(Transformation):
    def __init__(self, lin_ag, trans_g):
        self.lin_ag = lin_ag
        self.trans_g = trans_g
    def transform_points(self, x_ma):
        return x_ma.dot(self.lin_ag) + self.trans_g[None,:]  
    def compute_jacobian(self, x_ma):
        return np.repeat(self.lin_ag.T[None,:,:],len(x_ma), axis=0)
        
class Composition(Transformation):
    def __init__(self, fs):
        "applied from first to last (left to right)"
        self.fs = fs
    def transform_points(self, x_ma):
        for f in self.fs: x_ma = f.transform_points(x_ma)
        return x_ma
    def compute_jacobian(self, x_ma):
        grads = []
        for f in self.fs:
            grad_mga = f.compute_jacobian(x_ma)
            grads.append(grad_mga)
            x_ma = f.transform_points(x_ma)
        totalgrad = grads[0]
        for grad in grads[1:]:
            totalgrad = (grad[:,:,:,None] * totalgrad[:,None,:,:]).sum(axis=-2)
        return totalgrad

def fit_ThinPlateSpline(x_na, y_ng, bend_coef=.1, rot_coef = 1e-5, wt_n=None, rot_target = None):
    """
    x_na: source cloud
    y_nd: target cloud
    smoothing: penalize non-affine part
    angular_spring: penalize rotation
    wt_n: weight the points        
    """
    f = ThinPlateSpline()
    f.lin_ag, f.trans_g, f.w_ng = tps.tps_fit3(x_na, y_ng, bend_coef, rot_coef, wt_n, rot_target = rot_target)
    f.x_na = x_na
    return f   

def fit_ThinPlateSpline_RotReg(x_na, y_ng, bend_coef = .1, rot_coefs = (0.01,0.01,0.0025),scale_coef=.01):
    import fastrapp
    f = ThinPlateSpline()
    rfunc = fastrapp.rot_reg
    fastrapp.set_coeffs(rot_coefs, scale_coef)
    f.lin_ag, f.trans_g, f.w_ng = tps.tps_fit_regrot(x_na, y_ng, bend_coef, rfunc)
    f.x_na = x_na
    return f        
    


def loglinspace(a,b,n):
    "n numbers between a to b (inclusive) with constant ratio between consecutive numbers"
    return np.exp(np.linspace(np.log(a),np.log(b),n))    
    


def unit_boxify(x_na):    
    ranges = x_na.ptp(axis=0)
    dlarge = ranges.argmax()
    unscaled_translation = - (x_na.min(axis=0) + x_na.max(axis=0))/2
    scaling = 1./ranges[dlarge]
    scaled_translation = unscaled_translation * scaling
    return x_na*scaling + scaled_translation, (scaling, scaled_translation)
    
def unscale_tps_3d(f, src_params, targ_params):
    """Only works in 3d!!"""
    assert len(f.trans_g) == 3
    p,q = src_params
    r,s = targ_params
    print p,q,r,s
    fnew = ThinPlateSpline()
    fnew.x_na = (f.x_na  - q[None,:])/p 
    fnew.w_ng = f.w_ng * p / r
    fnew.lin_ag = f.lin_ag * p / r
    fnew.trans_g = (f.trans_g  + f.lin_ag.T.dot(q) - s)/r
    
    return fnew

def unscale_tps(f, src_params, targ_params):
    """Only works in 3d!!"""
    p,q = src_params
    r,s = targ_params
    
    d = len(q)
    
    lin_in = np.eye(d)*p
    trans_in = q
    aff_in = Affine(lin_in, trans_in)
    
    lin_out = np.eye(d)/r
    trans_out = -s/r
    aff_out = Affine(lin_out, trans_out)

    return Composition([aff_in, f, aff_out])
    
    

def tps_rpm(x_nd, y_md, n_iter = 20, reg_init = .1, reg_final = .001, rad_init = .1, rad_final = .005, rot_reg=1e-4,
            plotting = False, f_init = None, plot_cb = None):
    """
    tps-rpm algorithm mostly as described by chui and rangaran
    reg_init/reg_final: regularization on curvature
    rad_init/rad_final: radius for correspondence calculation (meters)
    plotting: 0 means don't plot. integer n means plot every n iterations
    """
    _,d=x_nd.shape
    regs = loglinspace(reg_init, reg_final, n_iter)
    rads = loglinspace(rad_init, rad_final, n_iter)
    if f_init is not None: 
        f = f_init  
    else:
        f = ThinPlateSpline(d)
        f.trans_g = np.median(y_md,axis=0) - np.median(x_nd,axis=0)

    for i in xrange(n_iter):
        xwarped_nd = f.transform_points(x_nd)
        corr_nm = calc_correspondence_matrix(xwarped_nd, y_md, r=rads[i], p=.1, max_iter=10)

        wt_n = corr_nm.sum(axis=1)


        targ_nd = (corr_nm/wt_n[:,None]).dot(y_md)
        
        if plotting and i%plotting==0:
            plot_cb(x_nd, y_md, targ_nd, corr_nm, wt_n, f)
        
        
        f = fit_ThinPlateSpline(x_nd, targ_nd, bend_coef = regs[i], wt_n=wt_n, rot_coef = rot_reg)

    return f

# @profile
def tps_rpm_bij(x_nd, y_md, n_iter = 20, reg_init = .1, reg_final = .001, rad_init = .1, rad_final = .01, rot_reg = 1e-3, 
            plotting = False, plot_cb = None, outliersd = 2., corr_reg=.5, update_rot_target=False):
    """
    tps-rpm algorithm mostly as described by chui and rangaran
    reg_init/reg_final: regularization on curvature
    rad_init/rad_final: radius for correspondence calculation (meters)
    plotting: 0 means don't plot. integer n means plot every n iterations
    """
    
    
    n,d=x_nd.shape
    m = y_md.shape[0]

    f_rot_target = np.eye(d)
    g_rot_target = np.eye(d)
    
    regs = loglinspace(reg_init, reg_final, n_iter)
    rads = loglinspace(rad_init, rad_final, n_iter)

    f = ThinPlateSpline(d)
    f.trans_g = np.median(y_md,axis=0) - np.median(x_nd,axis=0)
    f.lin_ag = np.eye(d)
    f.w_ng = np.zeros((n,d))
    f.x_na = x_nd
    
    g = ThinPlateSpline(d)
    g.trans_g = -f.trans_g
    g.lin_ag = np.eye(d)
    g.w_ng = np.zeros((m,d))
    g.x_na = y_md


    p = np.log(m/10.) * np.exp(-outliersd**2/2.)
    q = np.log(n/10.) * np.exp(-outliersd**2/2.)

    r_n = None
    
    Kx_nn = tps.tps_kernel_matrix(x_nd)
    Ky_mm = tps.tps_kernel_matrix(y_md)
    
    for i in xrange(n_iter):
        xwarped_nd = np.dot(Kx_nn, f.w_ng) + np.dot(x_nd, f.lin_ag) + f.trans_g[None,:]
        ywarped_md = np.dot(Ky_mm, g.w_ng) + np.dot(y_md, g.lin_ag) + g.trans_g[None,:]
        
        fwddist_nm = ssd.cdist(xwarped_nd, y_md,'sqeuclidean')
        invdist_nm = ssd.cdist(x_nd, ywarped_md,'sqeuclidean')
        
        r = rads[i]
        prob_nm = np.exp( (fwddist_nm + invdist_nm) / (-2*r**2) )

        rs_n = np.ones(n)
        cs_m = np.ones(m)
        
        # rs_n = np.sqrt(np.array([abs(np.linalg.det(j)) for j in f.compute_jacobian(x_nd)]))
        # cs_m = np.sqrt(np.array([abs(np.linalg.det(j)) for j in g.compute_jacobian(y_md)]))
        # 
        # # print percentiles(rs_n,np.r_[25,50,75]),percentiles(cs_m,np.r_[25,50,75])
        # rs_n[np.isnan(rs_n)] = 1
        # cs_m[np.isnan(cs_m)] = 1
        
        
        r_n, c_m =  balance_matrix3(prob_nm, 10, p, q, r_n=r_n, rs_n = rs_n, cs_m = cs_m)
        corr_reg1 = corr_reg * (n_iter-1-i)/n_iter
        corr_nm = prob_nm * (r_n[:,None]**corr_reg1 * c_m[None,:]**corr_reg1)
        corr_nm += 1e-10
        corr_nm *= float(n) / corr_nm.sum()

        # r_n, c_m =  balance_matrix3(prob_nm, 1, p, q, rs_n = rs_n, cs_m = cs_m)
        # corr_nm = (r_n[:,None] * c_m[None,:]) + 1e-9
        
        
        
        # corr_nm = prob_nm / prob_nm.sum()  * np.sqrt(n*m)
        
        wt_n = corr_nm.sum(axis=1)
        wt_m = corr_nm.sum(axis=0)

        xtarg_nd = (corr_nm/wt_n[:,None]).dot(y_md)
        ytarg_md = (corr_nm/wt_m[None,:]).T.dot(x_nd)
        
        if plotting and i%plotting==0:
            cbdata = dict(x_nd = x_nd, y_md = y_md, corr_nm = corr_nm, f=f, g=g, iteration=i)
            plot_cb(cbdata)

        f.lin_ag, f.trans_g, f.w_ng = tps.tps_fit3(x_nd, xtarg_nd, bend_coef=regs[i], wt_n=wt_n, rot_coef=regs[i], rot_target = f_rot_target, K_nn = Kx_nn)

        g.lin_ag, g.trans_g, g.w_ng = tps.tps_fit3(y_md, ytarg_md, bend_coef=regs[i], wt_n=wt_m, rot_coef=regs[i], rot_target = g_rot_target, K_nn = Ky_mm)
        
        if update_rot_target:
            f_rot_target, g_rot_target = orthogonalize3_svd(np.array([f.lin_ag, g.lin_ag]))
            # axf,angf = logmap(f_rot_target)
            # axg,angg = logmap(g_rot_target)
            # print angf*180/np.pi, angg*180/np.pi
    
    
    f._cost = tps.tps_cost(f.lin_ag, f.trans_g, f.w_ng, f.x_na, xtarg_nd, regs[i], wt_n=wt_n)/wt_n.mean()
    g._cost = tps.tps_cost(g.lin_ag, g.trans_g, g.w_ng, g.x_na, ytarg_md, regs[i], wt_n=wt_m)/wt_m.mean()
    return f,g

def tps_reg_cost(f):
    K_nn = tps.tps_kernel_matrix(f.x_na)
    cost = 0
    for w in f.w_ng.T:
        cost += w.dot(K_nn.dot(w))
    return cost
    
def logmap(m):
    "http://en.wikipedia.org/wiki/Axis_angle#Log_map_from_SO.283.29_to_so.283.29"
    theta = np.arccos(np.clip((np.trace(m) - 1)/2,-1,1))
    return (1/(2*np.sin(theta))) * np.array([[m[2,1] - m[1,2], m[0,2]-m[2,0], m[1,0]-m[0,1]]]), theta


def balance_matrix3(prob_nm, max_iter, p, q, r_n = None, rs_n = None, cs_m = None):
    
    n,m = prob_nm.shape
    
    if r_n is None: r_n = np.ones(n,'f4')
    if rs_n is None: rs_n = np.ones(n,'f4')
    if cs_m is None: cs_m = (float(n)/m)*np.ones(m,'f4')

    for _ in xrange(max_iter):
        c_m = cs_m/(r_n.dot(prob_nm) + p)
        r_n = rs_n/(prob_nm.dot(c_m) + q)

    return r_n, c_m

def balance_matrix(prob_nm, p, max_iter=20, ratio_err_tol=1e-3):
    n,m = prob_nm.shape
    pnoverm = (float(p)*float(n)/float(m))
    for _ in xrange(max_iter):
        colsums = pnoverm + prob_nm.sum(axis=0)        
        prob_nm /=  + colsums[None,:]
        rowsums = p + prob_nm.sum(axis=1)
        prob_nm /= rowsums[:,None]
        
        if ((rowsums-1).__abs__() < ratio_err_tol).all() and ((colsums-1).__abs__() < ratio_err_tol).all():
            break


    return prob_nm

def calc_correspondence_matrix(x_nd, y_md, r, p, max_iter=20):
    dist_nm = ssd.cdist(x_nd, y_md,'euclidean')
    
    
    prob_nm = np.exp(-dist_nm / r)
    # Seems to work better without **2
    # return balance_matrix(prob_nm, p=p, max_iter = max_iter, ratio_err_tol = ratio_err_tol)
    outlierfrac = 1e-1
    return balance_matrix3(prob_nm, max_iter, p, outlierfrac)


def nan2zero(x):
    np.putmask(x, np.isnan(x), 0)
    return x


def fit_score(src, targ, dist_param):
    "how good of a partial match is src to targ"
    sqdists = ssd.cdist(src, targ,'sqeuclidean')
    return -np.exp(-sqdists/dist_param**2).sum()

def orthogonalize3_cross(mats_n33):
    "turns each matrix into a rotation"

    x_n3 = mats_n33[:,:,0]
    # y_n3 = mats_n33[:,:,1]
    z_n3 = mats_n33[:,:,2]

    znew_n3 = math_utils.normr(z_n3)
    ynew_n3 = math_utils.normr(np.cross(znew_n3, x_n3))
    xnew_n3 = math_utils.normr(np.cross(ynew_n3, znew_n3))

    return np.concatenate([xnew_n3[:,:,None], ynew_n3[:,:,None], znew_n3[:,:,None]],2)

def orthogonalize3_svd(x_k33):
    u_k33, _s_k3, v_k33 = svds.svds(x_k33)
    return (u_k33[:,:,:,None] * v_k33[:,None,:,:]).sum(axis=2)

def orthogonalize3_qr(_x_k33):
    raise NotImplementedError

def orthogonalize_gripper(axis_0, axis_2):
    #axis_2 is the main axis
    #axis_0 is the secondary axis
    #assuming axis_0 cross axis_1 should be axis_2

    #axis_2 stays axis_2
    #axis_1 = axis_2 cross axis_0
    #axis_0 = axis_1 cross axis_2
    
    rotation_mat = np.zeros((3,3))
    rotation_mat[:,2] = axis_2
    axis_1 = math_utils.normalize(np.cross(axis_2, axis_0))
    rotation_mat[:,1] = axis_1
    rotation_mat[:,0] =  math_utils.normalize(np.cross(axis_1, axis_2))
    return rotation_mat
    