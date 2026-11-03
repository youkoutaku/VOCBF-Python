"""Paper-equation unicycle controller and explicitly labeled extensions."""
import numpy as np
from .models import Model,box
from .paper_reference import clf_rows,obstacle_rows,physical_rows
from .vocbf_intervals import solve_source_union


def default_config():
    return dict(gain=1.,clf_rate=1.,state_cbf_rate=1.,desired_speed_gain=1.,
        position_feedback=[1.,1.],position_coefficients=[1.,1.],heading_feedback=1.,
        input_increment_bounds=[6.,3.],physical_H_diagonal=[1.,1.],physical_R_diagonal=[.1,.1],
        clf_weights=[100.,100.,1.,1.],heading_chart='literal_arctan')


def planar_rows(x,o,radius,vo,gain):
    """Same VO half-space geometry on a Cartesian double integrator."""
    rel=x[:2]-o;vr=x[2:]-vo;d=np.linalg.norm(rel)
    if d<=radius:raise ValueError('VOCBF requires exterior obstacle geometry')
    angle=np.arcsin(radius/d)-np.pi/2;cols=[]
    for sign in (1,-1):
        beta=sign*angle;c,s=np.cos(beta),np.sin(beta);R=np.array([[c,-s],[s,c]])
        N=R@rel;betadot=-sign*radius*(rel@vr)/(d*d*np.sqrt(d*d-radius*radius));Ndot=R@vr+betadot*np.array([-N[1],N[0]])
        h=float(vr@N);cols.append([N[0],N[1],float(vr@Ndot+gain*h),h])
    return np.asarray(cols).T

class VOCBF:
    def __init__(self,model,obstacles,radii,config=None,obstacle_velocities=None,held_bounds=True):
        self.plant=model;self.lifted=model.kind in ('single_integrator','kinematic_unicycle')
        kind={'single_integrator':'double_integrator','kinematic_unicycle':'dynamic_unicycle'}.get(model.kind,model.kind)
        self.model=Model(kind,model.offset,model.vmax,model.wmax,model.amax,model.alphamax)
        self.virtual_velocity=np.zeros(2);self.previous=np.zeros(2);self.cfg=default_config()
        if config:self.cfg.update(config)
        self.obstacles=np.asarray(obstacles).reshape(-1,2);self.radii=np.asarray(radii)
        self.velocities=np.zeros_like(self.obstacles) if obstacle_velocities is None else np.asarray(obstacle_velocities).reshape(-1,2)
        self.held_bounds=held_bounds

    def extended(self,x):return np.r_[x,self.virtual_velocity] if self.lifted else np.asarray(x)

    def action(self,x,goal,dt,t=0.):
        z=self.extended(x);cfg=dict(self.cfg);obs=self.obstacles+t*self.velocities
        try:
            if self.model.kind=='dynamic_unicycle':
                case=dict(F_max=self.model.amax,mass=1.,tau_max=self.model.alphamax,inertia=1.,v_max=self.model.vmax,omega_max=self.model.wmax)
                A,b=physical_rows(z,case,cfg,self.previous,dt,held_bounds=self.held_bounds)
                clf_A,clf_c,V=clf_rows(z,np.asarray(goal),self.model.offset,cfg,cfg['heading_chart'])
                rows=[obstacle_rows(z,o,r,self.model.offset,vo,cfg['gain']) for o,r,vo in zip(obs,self.radii,self.velocities)]
            else:
                A,b=box(*self.model.bounds(z,dt))
                e=z[:2]-goal+z[2:];v=z[2:]
                # Disclosed Cartesian CLFs: no invented heading state or Eq14.
                V=np.array([e@e,v@v]);clf_A=np.array([2*e,2*v]);clf_c=np.array([2*e@v+cfg['clf_rate']*V[0],cfg['clf_rate']*V[1]])
                cfg['clf_weights']=[100.,1.]
                rows=[planar_rows(z,o,r,vo,cfg['gain']) for o,r,vo in zip(obs,self.radii,self.velocities)]
            u,cost,solver=solve_source_union(rows,A,b,clf_A,clf_c,cfg,self.previous,use_scalar_intervals=self.model.kind=='dynamic_unicycle' and self.model.offset==0.)
        except ValueError as e:return None,{'status':'source_domain_boundary','reason':str(e)}
        if u is not None:self.previous=u.copy()
        return u,{'status':'ok' if u is not None else 'qp_infeasible','objective':None if u is None else cost,'solver':solver,'clf_values':V.tolist(),'minimum_physical_residual':None if u is None else float(np.min(b-A@u)),'OR_residuals':None if u is None else [(r[:2].T@u+r[2]).tolist() for r in rows],'initial_VO_values':[r[3].tolist() for r in rows],'dynamic_extension':self.lifted,'input_semantics':'held acceleration, continuously ramped plant velocity command' if self.lifted else 'held physical acceleration','plant_command_start':self.virtual_velocity.tolist() if self.lifted else None}

    def advance(self,x,u,dt):
        sample=self.model.integrate(self.extended(x),u,dt)
        if self.lifted:
            self.virtual_velocity=sample[-1,-2:].copy()
            return sample[:,:-2]
        return sample

    def settled(self,x,u,goal):
        return self.model.settled(self.extended(x),u,goal)
