"""Numerical examples with explicit failure retention and no safety repair."""
from pathlib import Path
import json,platform,sys
import numpy as np
from .models import Model
METHOD='vocbf'

def jsonable(v):
    if isinstance(v,dict):return {k:jsonable(w) for k,w in v.items()}
    if isinstance(v,(list,tuple)):return [jsonable(w) for w in v]
    if isinstance(v,np.ndarray):return jsonable(v.tolist())
    if isinstance(v,np.generic):return jsonable(v.item())
    if isinstance(v,float) and not np.isfinite(v):return None
    return v

def run(kind,output,steps=400,dt=.05,offset=.15):
    from .controller import VOCBF as CONTROLLER
    model=Model(kind,offset=offset);goal=np.array([4.,2.]);obstacles=np.array([[2.,.4]]);radii=np.array([.45])
    controller=CONTROLLER(model,obstacles,radii,dt=dt) if METHOD=='mpc_cbf' else CONTROLLER(model,obstacles,radii)
    x=model.initial();states=[x.copy()];controls=[];times=[0.];events=[];minimum=np.inf;dwell=0.;outcome='timeout';initial=x.copy()
    for k in range(steps):
        u,info=controller.action(x,goal,dt,k*dt);events.append(jsonable(info))
        if u is None:outcome=info.get('status','no_control');break
        if not np.isfinite(u).all():outcome='nonfinite_control';break
        samples=controller.advance(x,u,dt) if hasattr(controller,'advance') else model.integrate(x,u,dt)
        centers=np.array([model.center(z) for z in samples]);clearance=np.linalg.norm(centers[:,None,:]-obstacles[None,:,:],axis=2)-radii
        minimum=min(minimum,float(clearance.min()));x=samples[-1];states.append(x.copy());controls.append(u.copy());times.append((k+1)*dt)
        if info.get('dynamic_extension'):events[-1]['plant_command_end']=controller.virtual_velocity.tolist()
        if minimum<-1e-6:outcome='sampled_collision';break
        settled=controller.settled(x,u,goal) if hasattr(controller,'settled') else model.settled(x,u,goal)
        dwell=dwell+dt if settled else 0.
        if dwell>=.5-1e-9:outcome='goal_complete';break
    out=Path(output)
    if out.exists() and any(out.iterdir()):raise FileExistsError('Refusing to overwrite '+str(out))
    out.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(out/'trajectory.npz',time=np.asarray(times),state=np.asarray(states),control=np.asarray(controls).reshape(-1,2))
    summary=dict(method=METHOD,model=kind,model_extension=True,offset=offset,initial=initial.tolist(),goal=goal.tolist(),obstacles=obstacles.tolist(),inflated_radii=radii.tolist(),dt=dt,requested_steps=steps,executed_steps=len(controls),outcome=outcome,final_goal_error=float(np.linalg.norm(model.center(x)-goal)),minimum_sampled_clearance=minimum,collision_sampled=minimum<-1e-6,solver_fallbacks=sum(e.get('fallback',False) for e in events),verification='21 independent DOP853 samples per hold; no interval enclosure or sampled-safety theorem claimed',python=sys.version.split()[0],platform=platform.platform(),input_semantics='held physical control',dynamic_extension=False)
    if METHOD=='vocbf' and controller.lifted:summary.update(dynamic_extension=True,input_semantics='control stores filter acceleration; plant commands are continuous velocity ramps recorded in events')
    (out/'summary.json').write_text(json.dumps(jsonable(summary),indent=2,allow_nan=False)+'\n');(out/'events.json').write_text(json.dumps(events,indent=2,allow_nan=False)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(6,4));p=np.array([model.center(z) for z in states]);ax.plot(p[:,0],p[:,1],label=METHOD.upper());ax.plot(*goal,'*',label='goal')
    for o,r in zip(obstacles,radii):ax.add_patch(plt.Circle(o,r,color='gray',alpha=.4))
    ax.set(xlabel='x [m]',ylabel='y [m]',title=f'{kind}: {outcome}',aspect='equal');ax.legend();fig.tight_layout();fig.savefig(out/'trajectory.png',dpi=150);plt.close(fig)
    return jsonable(summary)
