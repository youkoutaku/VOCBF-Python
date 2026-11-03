"""Deterministic numerical validation; no claims of theorem certification."""
import numpy as np
from .models import Model,KINDS
from .qp import quadratic

def common_checks():
    rows=[]
    for kind in KINDS:
        m=Model(kind);x=m.initial();u=np.array([.1,.15]);dt=.04
        numerical=m.integrate(x,u,dt,2)[-1];pred=np.asarray(m.symbolic_step(x,u,dt)).ravel();err=float(np.max(abs(numerical-pred)))
        rows.append({'test':'prediction_vs_independent_DOP853','model':kind,'error':err,'passed':err<1e-8})
    # A non-axis-aligned polygon with active constraints, independent Clarabel.
    import clarabel
    from scipy import sparse
    rng=np.random.default_rng(7)
    for i in range(20):
        A=np.vstack([np.eye(2),-np.eye(2),rng.normal(size=(5,2))]);b=np.r_[np.ones(4),rng.uniform(.05,1,5)]
        H=np.diag(rng.uniform(.2,3,2));q=rng.normal(size=2)*3
        u,_=quadratic(H,q,A,b);s=clarabel.DefaultSettings();s.verbose=False;s.tol_feas=1e-10;s.tol_gap_abs=1e-10;s.tol_gap_rel=1e-10
        ref=clarabel.DefaultSolver(sparse.csc_matrix(H),q,sparse.csc_matrix(A),b,[clarabel.NonnegativeConeT(len(b))],s).solve();error=float(np.max(abs(u-np.asarray(ref.x))))
        rows.append({'test':'active_set_vs_clarabel','case':i,'error':error,'passed':error<1e-5})
    u,_=quadratic(np.eye(2),np.zeros(2),np.array([[1.,0.],[-1.,0.]]),np.array([-1.,-1.]))
    rows.append({'test':'infeasible_problem_retains_no_control','passed':u is None})
    return rows

def verify():
    rows=common_checks()+method_checks()
    return {'passed':all(r['passed'] for r in rows),'checks':len(rows),'records':rows,'scope':'Equations, model contracts and independent numerical solver checks; not universal closed-loop safety or exact author figure reproduction.'}

def method_checks():
    from .controller import default_config
    from .paper_reference import clf_rows,obstacle_rows,physical_rows,solve_explicit
    from .symbolic_reference import clf_function,moving_vocbf_function
    from .vocbf_intervals import solve_source_union
    rows=[];rng=np.random.default_rng(20261103);cfg=default_config()
    for ell in [0.,.15]:
        symbolic=clf_function(ell,cfg);barrier=moving_vocbf_function(ell,cfg['gain'])
        for i in range(100):
            x=np.r_[rng.normal(size=2),rng.uniform(-2,2),rng.uniform(.1,1),rng.uniform(-.5,.5)];goal=x[:2]+[3.,2.];o=x[:2]+[2.,-.9];vo=np.array([-.2,.1])
            a,c,v=clf_rows(x,goal,ell,cfg,'literal_arctan');sa,sc,sv=symbolic(x,goal)
            err=max(np.max(abs(a-np.asarray(sa))),np.max(abs(c-np.asarray(sc).ravel())),np.max(abs(v-np.asarray(sv).ravel())))
            b=obstacle_rows(x,o,.4,ell,vo,cfg['gain']);s=np.asarray(barrier(x,o,.4,vo))[:4]
            berr=float(np.max(abs(b-s)));rows.append({'test':'closed_form_vs_symbolic_derivative','offset':ell,'case':i,'clf_error':float(err),'barrier_error':berr,'passed':err<1e-9 and berr<1e-10})
        for i in range(10):
            x=np.array([0.,0.,.4,.1+i*.03,.05]);goal=np.array([3.,1.]);case=dict(F_max=.9,mass=1.,tau_max=1.5,inertia=1.,v_max=1.5,omega_max=1.8)
            A,b=physical_rows(x,case,cfg,np.zeros(2),.05);a,c,_=clf_rows(x,goal,ell,cfg,'literal_arctan');ob=[obstacle_rows(x,np.array([2.,-.5]),.4,ell,np.zeros(2),cfg['gain'])]
            u,cost,_=solve_source_union(ob,A,b,a,c,cfg,np.zeros(2),use_scalar_intervals=ell==0.)
            ref,rcost,_=solve_explicit(ob,A,b,a,c,cfg,np.zeros(2),backend='clarabel')
            error=None if u is None or ref is None else float(np.max(abs(u-ref)));ce=None if error is None else abs(cost-rcost)/(1+abs(rcost))
            rows.append({'test':'eliminated_slacks_vs_explicit_Clarabel','offset':ell,'case':i,'control_error':error,'relative_cost_error':ce,'passed':(u is None and ref is None) or (error is not None and error<2e-4 and ce<1e-7)})
    # The literal source heading chart is not silently replaced with atan2.
    try:clf_rows(np.zeros(5),np.array([0.,1.]),0.,cfg,'literal_arctan');undefined=False
    except ValueError:undefined=True
    rows.append({'test':'literal_arctan_zero_denominator_visible','passed':undefined})
    return rows
