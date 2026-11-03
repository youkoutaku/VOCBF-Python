"""Four explicit planar control-affine models, in SI units.

'First/second order' describes position/input order, not state-vector length.
All four first-order state-space equations have form xdot=f(x)+g(x)u.
"""
from dataclasses import dataclass
import casadi as ca
import numpy as np
from scipy.integrate import solve_ivp

KINDS=('single_integrator','double_integrator','kinematic_unicycle','dynamic_unicycle')

@dataclass
class Model:
    kind: str
    offset: float = 0.15
    vmax: float = 1.5
    wmax: float = 1.8
    amax: float = 0.9
    alphamax: float = 1.5

    def __post_init__(self):
        if self.kind not in KINDS:raise ValueError(self.kind)
        self.nx=dict(zip(KINDS,[2,4,3,5]))[self.kind]
        self.nu=2
        x=ca.SX.sym('x',self.nx);u=ca.SX.sym('u',2)
        f,g=self.affine(x);self._rhs=ca.Function('rhs',[x,u],[f+g@u])

    def affine(self,x):
        f=ca.SX.zeros(self.nx);g=ca.SX.zeros(self.nx,2)
        if self.kind=='single_integrator':g[:2,:]=ca.SX.eye(2)
        elif self.kind=='double_integrator':f[:2]=x[2:4];g[2:4,:]=ca.SX.eye(2)
        elif self.kind=='kinematic_unicycle':
            g[0,0]=ca.cos(x[2]);g[1,0]=ca.sin(x[2]);g[2,1]=1
        else:
            f[0]=x[3]*ca.cos(x[2]);f[1]=x[3]*ca.sin(x[2]);f[2]=x[4]
            g[3,0]=1;g[4,1]=1
        return f,g

    def rhs(self,x,u):return np.asarray(self._rhs(x,u)).ravel()

    def center(self,x):
        if self.kind in ('kinematic_unicycle','dynamic_unicycle'):
            if isinstance(x,(ca.SX,ca.MX)):return x[:2]+self.offset*ca.vertcat(ca.cos(x[2]),ca.sin(x[2]))
            return np.asarray(x[:2])+self.offset*np.array([np.cos(x[2]),np.sin(x[2])])
        return x[:2]

    def symbolic_step(self,x,u,dt,substeps=8):
        # Exact integrators; RK4 unicycle prediction is explicitly a discretization.
        if self.kind=='single_integrator':return x+dt*u
        if self.kind=='double_integrator':return ca.vertcat(x[:2]+dt*x[2:]+.5*dt**2*u,x[2:]+dt*u)
        z=x
        for _ in range(substeps):
            h=dt/substeps;k1=self._rhs(z,u);k2=self._rhs(z+.5*h*k1,u);k3=self._rhs(z+.5*h*k2,u);k4=self._rhs(z+h*k3,u)
            z=z+h*(k1+2*k2+2*k3+k4)/6
        return z

    def integrate(self,x,u,dt,samples=21):
        sol=solve_ivp(lambda t,z:self.rhs(z,u),(0,dt),x,method='DOP853',rtol=1e-10,atol=1e-12,dense_output=True)
        if not sol.success:raise RuntimeError(sol.message)
        return sol.sol(np.linspace(0,dt,samples)).T

    def bounds(self,x,dt):
        if dt<=0:raise ValueError('dt must be positive')
        if self.kind=='single_integrator':lo=np.full(2,-self.vmax);hi=-lo
        elif self.kind=='kinematic_unicycle':lo=np.array([0.,-self.wmax]);hi=np.array([self.vmax,self.wmax])
        elif self.kind=='double_integrator':lo=np.maximum(-self.amax,(-self.vmax-x[2:])/dt);hi=np.minimum(self.amax,(self.vmax-x[2:])/dt)
        else:
            lo=np.maximum([-self.amax,-self.alphamax],[-x[3]/dt,(-self.wmax-x[4])/dt])
            hi=np.minimum([self.amax,self.alphamax],[(self.vmax-x[3])/dt,(self.wmax-x[4])/dt])
        return np.asarray(lo),np.asarray(hi)

    def nominal(self,x,goal,dt=.05):
        delta=np.asarray(goal)-self.center(x);distance=np.linalg.norm(delta)
        if self.kind=='single_integrator':u=.9*delta
        elif self.kind=='double_integrator':u=1.2*delta-2*x[2:]
        else:
            heading=np.arctan2(delta[1],delta[0]) if distance>1e-9 else x[2]
            error=np.arctan2(np.sin(heading-x[2]),np.cos(heading-x[2]))
            vref=min(self.vmax,.9*distance)*max(0.,np.cos(error));wref=np.clip(2.2*error,-self.wmax,self.wmax)
            u=np.array([vref,wref]) if self.kind=='kinematic_unicycle' else np.array([1.8*(vref-x[3]),2.5*(wref-x[4])])
        lo,hi=self.bounds(x,dt);return np.clip(u,lo,hi)

    def initial(self):
        heading=np.arctan2(2.,4.)
        if self.kind=='single_integrator':return np.zeros(2)
        if self.kind=='double_integrator':return np.zeros(4)
        axle=-self.offset*np.array([np.cos(heading),np.sin(heading)])
        return np.r_[axle,heading] if self.kind=='kinematic_unicycle' else np.r_[axle,heading,0.,0.]

    def settled(self,x,u,goal):
        if np.linalg.norm(self.center(x)-goal)>=.15:return False
        if self.kind in ('single_integrator','kinematic_unicycle'):return np.linalg.norm(u)<.08
        if self.kind=='double_integrator':return np.linalg.norm(x[2:])<.08
        return abs(x[3])<.05 and abs(x[4])<.05

def box(lo,hi):
    return np.array([[1.,0.],[-1.,0.],[0.,1.],[0.,-1.]]),np.array([hi[0],-lo[0],hi[1],-lo[1]])
