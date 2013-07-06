"""
Resample time serieses to reduce the number of datapoints
"""
from __future__ import division
import numpy as np
from rapprentice import log
import fastrapp
import math_utils as mu

def lerp(a, b, fracs):
    "linearly interpolate between a and b"
    fracs = fracs[:,None]
    return a*(1-fracs) + b*fracs

def adaptive_resample2(x, t = None, max_err = np.inf, max_dx = np.inf, max_dt = np.inf, normfunc = None):
    #return np.arange(0, len(x), 100)
    log.info("resampling a path of length %i"%len(x))
    x = np.asarray(x)
    
    if x.ndim == 1: x = x[:,None]
    else: assert x.ndim == 2
    
    if normfunc is None: normfunc = np.linalg.norm
    
    if t is None: t = np.arange(len(x))
    else: t = np.asarray(t)
    assert(t.ndim == 1)
    
    n = len(t)
    assert len(x) == n

    import networkx as nx
    g = nx.DiGraph()
    g.add_nodes_from(xrange(n))
    for i_src in xrange(n):
        for i_targ in xrange(i_src+1, n):
            normdx = normfunc(x[i_targ] - x[i_src])
            dt = t[i_targ] - t[i_src]
            errs = lerp(x[i_src], x[i_targ], (t[i_src:i_targ+1] - t[i_src])/dt) - x[i_src:i_targ+1]
            interp_err = errs.__abs__().max()#np.array([normfunc(err) for err in errs]).max()
            if normdx > max_dx or dt > max_dt or interp_err > max_err: break
            g.add_edge(i_src, i_targ)
    resample_inds = nx.shortest_path(g, 0, n-1)
    return resample_inds

def adaptive_resample(x, tol, max_change=None, min_steps=3):
    """
    resample original signal with a small number of waypoints so that the the sparsely sampled function, 
    when linearly interpolated, deviates from the original function by less than tol at every time

    input:
    x: 2D array in R^(t x k)  where t is the number of timesteps
    tol: tolerance. either a single scalar or a vector of length k
    max_change: max change in the sparsely sampled signal at each timestep
    min_steps: minimum number of timesteps in the new trajectory. (usually irrelevant)

    output:
    new_times, new_x

    assuming that the old signal has times 0,1,2,...,len(x)-1
    this gives the new times, and the new signal
    """
    x = np.asarray(x)
    assert x.ndim == 2

    if np.isscalar(tol): 
        tol = np.ones(x.shape[1])*tol
    else:
        tol = np.asarray(tol)
        assert tol.ndim == 1 and tol.shape[0] == x.shape[1]

    times = np.arange(x.shape[0])

    if max_change is None: 
        max_change = np.ones(x.shape[1]) * np.inf
    elif np.isscalar(max_change): 
        max_change = np.ones(x.shape[1]) * max_change
    else:
        max_change = np.asarray(max_change)
        assert max_change.ndim == 1 and max_change.shape[0] == x.shape[1]

    dl = mu.norms(x[1:] - x[:-1],1)
    l = np.cumsum(np.r_[0,dl])

    def bad_inds(x1, t1):
        ibad = np.flatnonzero( (np.abs(mu.interp2d(l, l1, x1) - x) > tol).any(axis=1) )
        jbad1 = np.flatnonzero((np.abs(x1[1:] - x1[:-1]) > max_change[None,:]).any(axis=1))
        if len(ibad) == 0 and len(jbad1) == 0: return []
        else:
            lbad = l[ibad]
            jbad = np.unique(np.searchsorted(l1, lbad)) - 1
            jbad = np.union1d(jbad, jbad1)
            return jbad

    l1 = np.linspace(0,l[-1],min_steps)
    for _ in xrange(20):
        x1 = mu.interp2d(l1, l, x)
        bi = bad_inds(x1, l1)
        if len(bi) == 0:
            return np.interp(l1, l, times), x1
        else:
            l1 = np.union1d(l1, (l1[bi] + l1[bi+1]) / 2 )


    raise Exception("couldn't subdivide enough. something funny is going on. check your input data")



def test_resample():
    x = [0,0,0,1,2,3,4,4,4]
    t = range(len(x))
    inds = adaptive_resample(x, max_err = 1e-5)
    assert inds == [0, 2, 6, 8]
    inds = adaptive_resample(x, t=t, max_err = 0)
    assert inds == [0, 2, 6, 8]
    print "success"


    inds1 = fastrapp.resample(np.array(x)[:,None], t, 0, np.inf, np.inf)
    print inds1
    assert inds1.tolist() == [0,2,6,8]

def test_resample_big():
    from time import time
    t = np.linspace(0,1,1000)
    x0 = np.sin(t)[:,None]
    x = x0 + np.random.randn(len(x0), 50)*.1
    tstart = time()
    inds0 = adaptive_resample(x, t=t, max_err = .05, max_dt = .1)
    print time() - tstart, "seconds"

    print "now doing cpp version"
    tstart = time()
    inds1 = fastrapp.resample(x, t, .05, np.inf, .1)
    print time() - tstart, "seconds"
    
    assert np.allclose(inds0, inds1)
if __name__ == "__main__":
    test_resample()
    test_resample_big()