"""Independent, closed-form VOCBF equations and explicit-slack Clarabel QPs.

No adapter symbolic differentiation, active-set solver or navigation is reused.
This is an equation implementation, not an unavailable author-code translation.
"""
from __future__ import annotations
import itertools
import numpy as np
from scipy import sparse
import clarabel
import casadi as ca

_QPOASES = {}


def frame(x, ell):
    n=np.array([np.cos(x[2]),np.sin(x[2])]); jn=np.array([-n[1],n[0]])
    return n,jn,np.asarray(x[:2])+ell*n,x[3]*n+ell*x[4]*jn


def clf_rows(x,goal,ell,cfg,chart="wrapped_atan2"):
    n,jn,p,vel=frame(x,ell); d=np.asarray(goal)-p; d2=float(d@d)
    acc0=-ell*x[4]**2*n+x[3]*x[4]*jn
    G=np.column_stack([n,ell*jn]); K=np.asarray(cfg["position_feedback"])
    C=np.asarray(cfg["position_coefficients"]); kd=cfg["desired_speed_gain"]
    e=p-goal+K*vel
    vals=[float(np.dot(C,e*e))]
    rows=[2*(C*e*K)@G]; constants=[2*(C*e)@(vel+K*acc0)]
    if d2<=1e-18:
        eh=0.; q=x[4]; qdot0=0.; qdotu=np.array([0.,1.])
    else:
        if chart=="literal_arctan":
            if d[0]==0: raise ValueError("Original arctan ratio is undefined at zero denominator")
            eh=x[2]-np.arctan(d[1]/d[0])
        elif chart=="wrapped_atan2":
            eh=np.arctan2(np.sin(x[2]-np.arctan2(d[1],d[0])),np.cos(x[2]-np.arctan2(d[1],d[0])))
        else:raise ValueError(chart)
        cross=lambda a,b:a[0]*b[1]-a[1]*b[0]
        cv=cross(d,vel); q=x[4]+cv/d2
        qdot0=cross(d,acc0)/d2+2*cv*float(d@vel)/d2**2
        qdotu=np.array([cross(d,G[:,0])/d2,1+cross(d,G[:,1])/d2])
    ebar=eh+cfg["heading_feedback"]*q
    vals.append(ebar**2);rows.append(2*ebar*cfg["heading_feedback"]*qdotu)
    constants.append(2*ebar*(q+cfg["heading_feedback"]*qdot0))
    distance=np.sqrt(max(d2,1e-18)); ev=x[3]-kd*distance
    vals.extend([ev**2,x[4]**2]);rows.extend([[2*ev,0.],[0.,2*x[4]]])
    constants.extend([0. if d2<=1e-18 else 2*ev*kd*float(d@vel)/distance,0.])
    vals=np.array(vals)
    return np.asarray(rows),np.asarray(constants)+cfg["clf_rate"]*vals,vals


def obstacle_rows(x,o,radius,ell,vo,gain,ao=(0.,0.)):
    n,jn,p,vel=frame(x,ell);rel=p-o;vr=vel-vo;d=float(np.linalg.norm(rel))
    if d<=radius:raise ValueError("VOCBF cone has no exterior geometry")
    acc0=-ell*x[4]**2*n+x[3]*x[4]*jn-np.asarray(ao)
    angle=np.arcsin(radius/d)-np.pi/2; columns=[]
    for sign in (1,-1):
        beta=sign*angle; c,s=np.cos(beta),np.sin(beta)
        R=np.array([[c,-s],[s,c]]);N=R@rel;JN=np.array([-N[1],N[0]])
        betadot=-sign*radius*float(rel@vr)/(d*d*np.sqrt(d*d-radius*radius))
        Ndot=R@vr+betadot*JN;value=float(vr@N)
        columns.append([float(n@N),float(ell*jn@N),float(acc0@N+vr@Ndot+gain*value),value])
    return np.asarray(columns).T


def physical_rows(x,case,cfg,previous,dt,held_bounds=True):
    amax=case["F_max"]/case["mass"];bmax=case["tau_max"]/case["inertia"]
    vmax=case["v_max"];wmax=case["omega_max"]
    A=np.array([[1.,0.],[-1.,0.],[0.,1.],[0.,-1.]])
    physical=np.array([amax,amax,bmax,bmax])
    if held_bounds:
        physical=np.minimum(physical,[(vmax-x[3])/dt,x[3]/dt,(wmax-x[4])/dt,(wmax+x[4])/dt])
    state=cfg["state_cbf_rate"]*np.array([vmax-x[3],x[3],wmax-x[4],wmax+x[4]])
    inc=np.asarray(cfg["input_increment_bounds"])
    increments=np.array([previous[0]+inc[0],-previous[0]+inc[0],previous[1]+inc[1],-previous[1]+inc[1]])
    return np.vstack([A,A,A]),np.r_[physical,state,increments]


def objective(u,clf_A,clf_c,cfg,previous):
    slack=np.maximum(0.,clf_A@u+clf_c)
    return float(.5*np.dot(cfg["physical_H_diagonal"],u*u)+.5*np.dot(cfg["physical_R_diagonal"],(u-previous)**2)+np.dot(cfg["clf_weights"],slack*slack))


def solve_explicit(rows,A,b,clf_A,clf_c,cfg,previous,backend="qpoases"):
    """Enumerate all original left/right choices; keep all four explicit slacks."""
    H=np.diag(np.r_[np.asarray(cfg["physical_H_diagonal"])+cfg["physical_R_diagonal"],2*np.asarray(cfg["clf_weights"])])
    q=np.r_[-np.asarray(cfg["physical_R_diagonal"])*previous,np.zeros(4)]
    settings=clarabel.DefaultSettings();settings.verbose=False
    settings.tol_gap_abs=1e-11;settings.tol_gap_rel=1e-11;settings.tol_feas=1e-11
    settings.max_iter=300
    best=None;best_cost=np.inf;info=None;statuses=[]
    for assignment in itertools.product((0,1),repeat=len(rows)):
        aa=np.vstack([A]+[-row[:2,j][None,:] for row,j in zip(rows,assignment)])
        bb=np.r_[b,[row[2,j] for row,j in zip(rows,assignment)]]
        matrix=np.vstack([np.c_[aa,np.zeros((len(bb),4))],np.c_[clf_A,-np.eye(4)]])
        rhs=np.r_[bb,-clf_c]
        # Positive row scaling changes no constraint and improves near-goal conditioning.
        scale=np.maximum(1.,np.max(abs(matrix),axis=1));scaled=matrix/scale[:,None]
        if backend=="clarabel":
            result=clarabel.DefaultSolver(sparse.csc_matrix(H),q,sparse.csc_matrix(scaled),rhs/scale,
                [clarabel.NonnegativeConeT(len(rhs))],settings).solve()
            status=str(result.status);statuses.append(status)
            if status not in ("Solved","AlmostSolved"):continue
            z=np.asarray(result.x);dual=np.asarray(result.z)/scale
        elif backend=="qpoases":
            dual_scale=np.ones(len(rhs))
            size=len(rhs)
            if size not in _QPOASES:
                _QPOASES[size]=ca.conic("source_explicit_qp_"+str(size),"qpoases",
                    {"h":ca.Sparsity.dense(6,6),"a":ca.Sparsity.dense(size,6)},
                    {"printLevel":"none","error_on_fail":False,"nWSR":1000})
            solver=_QPOASES[size]
            result=solver(h=H,g=q,a=matrix,uba=rhs,lba=np.full(size,-np.inf))
            status=solver.stats()["return_status"];statuses.append(status)
            if not solver.stats()["success"]:
                # The backend keeps a previous active set. Near the goal the
                # heading rows can change by many orders of magnitude; retry
                # the identical QP in a fresh solver rather than inferring
                # mathematical infeasibility from failed hot-start homotopy.
                solver=ca.conic("source_explicit_qp_cold_"+str(size),"qpoases",
                    {"h":ca.Sparsity.dense(6,6),"a":ca.Sparsity.dense(size,6)},
                    {"printLevel":"none","error_on_fail":False,"nWSR":1000})
                _QPOASES[size]=solver
                result=solver(h=H,g=q,a=matrix,uba=rhs,lba=np.full(size,-np.inf))
                status=solver.stats()["return_status"];statuses.append("cold_retry: "+status)
                if not solver.stats()["success"]:
                    # A positive row rescaling is the same feasible set.
                    dual_scale=np.maximum(1.,np.maximum(np.max(abs(matrix),axis=1),abs(rhs)))
                    solver=ca.conic("source_explicit_qp_scaled_"+str(size),"qpoases",
                        {"h":ca.Sparsity.dense(6,6),"a":ca.Sparsity.dense(size,6)},
                        {"printLevel":"none","error_on_fail":False,"nWSR":1000})
                    result=solver(h=H,g=q,a=matrix/dual_scale[:,None],uba=rhs/dual_scale,lba=np.full(size,-np.inf))
                    status=solver.stats()["return_status"];statuses.append("positive_row_scaling_retry: "+status)
                    if not solver.stats()["success"]:continue
            z=np.asarray(result["x"]).ravel();dual=np.asarray(result["lam_a"]).ravel()/dual_scale
        else:raise ValueError(backend)
        u=z[:2]
        slack=rhs-matrix@z;cost=objective(u,clf_A,clf_c,cfg,previous)
        if cost<best_cost:
            best=u;best_cost=cost;info=dict(status=status,backend=backend,assignment=list(assignment),
                primal_violation=max(0.,float(np.max(-slack))),
                relative_stationarity=float(np.max(abs(H@z+q+matrix.T@dual))/(1+np.max(abs(H@z+q)))),
                relative_complementarity=float(np.max(abs(dual*slack))/(1+abs(cost))),
                explicit_slacks=z[2:].tolist())
    return best,best_cost,dict(chosen=info,statuses=statuses)
