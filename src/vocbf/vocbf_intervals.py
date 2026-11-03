"""Exact VOCBF union decomposition, exploiting zero angular coefficients.

For rows k_j*a+c_j >= 0 OR k'_j*a+c'_j >= 0, intersect the
unions of closed half-lines in a. No gain, objective, constraint, navigation,
or floating-point feasibility tolerance is changed.
"""
import itertools
import numpy as np
from .qp import quadratic,soft_clf


def solve_source_union(rows,A,b,clf_A,clf_c,config,previous,*,use_lower_bound=True,use_scalar_intervals=True):
    """Exact physical/CLF lower bound, then the original union when needed.

    Dropping obstacle constraints gives a superset and a lower bound on cost.
    If its globally minimizing physical/CLF command satisfies every original
    obstacle OR, it is feasible in the subset and attains that same lower bound:
    therefore it is a global minimizer of the original problem. Acceptance uses
    the existing moving-adapter strict nonnegative residual check, not a relaxed
    obstacle tolerance. Otherwise every original feasible region is retained.
    """
    arguments=(clf_A,clf_c,np.asarray(config["clf_weights"]),previous,
               np.asarray(config["physical_H_diagonal"]),np.asarray(config["physical_R_diagonal"]))
    if use_lower_bound:
        lower_u,lower_value=soft_clf([(A,b)],*arguments)
        if (lower_u is not None and np.all(b-A@lower_u>=0.)
            and all(np.max(row[:2].T@lower_u+row[2])>=0. for row in rows)):
            return lower_u,lower_value,dict(algorithm="physical_CLF_global_lower_bound_attained",
                    enumerated_side_assignments=0,original_OR_residuals=[(r[:2].T@lower_u+r[2]).tolist() for r in rows],
                    optimality_argument="superset optimum satisfies every original OR constraint; same feasible optimum and objective")
    regions,solver=source_union_regions(rows,A,b,use_scalar_intervals=use_scalar_intervals)
    u,value=soft_clf(regions,*arguments)
    return u,value,solver


def _merge(intervals):
    result=[]
    for lo,hi in sorted(intervals):
        if lo>hi:continue
        if result and lo<=result[-1][1]:
            result[-1]=(result[-1][0],max(result[-1][1],hi))
        else:result.append((lo,hi))
    return result


def scalar_intervals(rows,A,b):
    """Return the exact finite union in a; retain all original physical rows."""
    if any(np.any(np.asarray(row)[1]!=0.) for row in rows):
        raise ValueError("Scalar decomposition requires exactly zero angular coefficients")
    lo,hi=-np.inf,np.inf
    for normal,bound in zip(A,b):
        if normal[1]!=0.:continue
        if normal[0]>0.:hi=min(hi,float(bound/normal[0]))
        elif normal[0]<0.:lo=max(lo,float(bound/normal[0]))
        elif bound<0.:return []
    intervals=[] if lo>hi else [(lo,hi)]
    for row in rows:
        halves=[]
        for k,c in zip(row[0],row[2]):
            if k>0.:halves.append((float(-c/k),np.inf))
            elif k<0.:halves.append((-np.inf,float(-c/k)))
            elif c>=0.:halves.append((-np.inf,np.inf))
        halves=_merge(halves)
        intervals=_merge([(max(a,c),min(b,d)) for a,b in intervals for c,d in halves
                          if max(a,c)<=min(b,d)])
        if not intervals:break
    return intervals


def source_union_regions(rows,A,b,*,use_scalar_intervals=True):
    A=np.asarray(A,float);b=np.asarray(b,float)
    if use_scalar_intervals and all(np.all(np.asarray(row)[1]==0.) for row in rows):
        intervals=scalar_intervals(rows,A,b);regions=[]
        for lo,hi in intervals:
            normals=[];bounds=[]
            if np.isfinite(lo):normals.append([-1.,0.]);bounds.append(-lo)
            if np.isfinite(hi):normals.append([1.,0.]);bounds.append(hi)
            regions.append((np.vstack([A,np.array(normals).reshape(-1,2)]),np.r_[b,bounds]))
        return regions,dict(algorithm="exact_scalar_acceleration_interval_union",intervals=intervals,
            enumerated_side_assignments=0,interval_count=len(intervals),
            original_obstacle_count=len(rows),alpha_coefficients_exactly_zero=True)
    regions=[];assignments=[]
    for assignment in itertools.product((0,1),repeat=len(rows)):
        aa=np.vstack([A]+[-row[:2,j][None,:] for row,j in zip(rows,assignment)])
        bb=np.r_[b,[row[2,j] for row,j in zip(rows,assignment)]]
        feasible,_=quadratic(np.eye(2),np.zeros(2),aa,bb)
        if feasible is not None:regions.append((aa,bb));assignments.append(assignment)
    return regions,dict(algorithm="exhaustive_original_VOCBF_union",enumerated_side_assignments=2**len(rows),
                        feasible_side_assignments=assignments)
