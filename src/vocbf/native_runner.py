"""Original-model equation mechanisms, not an unavailable author figure rerun."""
from pathlib import Path
import json
import numpy as np
from .models import Model
from .controller import VOCBF
from .simulation import jsonable

def original(output,suite='all'):
    cases=json.loads((Path(__file__).parent/'data/original_model_cases.json').read_text());out=Path(output)
    if out.exists() and any(out.iterdir()):raise FileExistsError(out)
    out.mkdir(parents=True,exist_ok=True);results=[]
    for name,case in cases.items():
        if suite!='all' and name!=suite:continue
        model=Model('dynamic_unicycle',offset=case['offset'],vmax=case['v_max'],wmax=case['omega_max'],amax=case['F_max'],alphamax=case['tau_max'])
        radii=np.array(case['obstacle_radii'])+case['robot_radius']+case['margin'];obs=np.array(case['obstacles']);vel=np.array(case['obstacle_velocities']);goal=np.array(case['goal']);x=np.array(case['initial']);dt=case['dt']
        controller=VOCBF(model,obs,radii,obstacle_velocities=vel,held_bounds=False);states=[x.copy()];controls=[];events=[];times=[0.];minimum=np.inf;outcome='timeout';dwell=0.
        for k in range(int(round(case['horizon']/dt))):
            u,info=controller.action(x,goal,dt,k*dt);events.append(jsonable(info))
            if u is None:outcome=info['status'];break
            sample=model.integrate(x,u,dt,samples=257);centers=np.array([model.center(s) for s in sample]);ts=k*dt+np.linspace(0,dt,len(sample))
            gap=np.linalg.norm(centers[:,None,:]-(obs[None,:,:]+ts[:,None,None]*vel[None,:,:]),axis=2)-radii;minimum=min(minimum,float(gap.min()));x=sample[-1];states.append(x.copy());controls.append(u.copy());times.append((k+1)*dt)
            events[-1]['minimum_sampled_design_clearance']=float(gap.min())
            if np.min(sample[:,3])<-1e-7 or np.max(sample[:,3])>model.vmax+1e-7 or np.max(abs(sample[:,4]))>model.wmax+1e-7:outcome='physical_bound_violation';break
            if gap.min()<-1e-7:outcome='design_clearance_violation';break
            dwell=dwell+dt if model.settled(x,u,goal) else 0.
            if dwell>=.5-1e-9:outcome='goal_complete';break
        folder=out/name;folder.mkdir();np.savez_compressed(folder/'trajectory.npz',time=times,state=states,acceleration=np.asarray(controls).reshape(-1,2));(folder/'events.json').write_text(json.dumps(events,indent=2)+'\n')
        result={'case':name,'outcome':outcome,'steps':len(controls),'minimum_sampled_design_clearance':minimum,'final_goal_error':float(np.linalg.norm(model.center(x)-goal)),'source_equations_implemented':True,'author_code_executed':False,'exact_author_figure':False,'offset':model.offset,'held_physical_bounds_added':False,'configuration':controller.cfg}
        (folder/'summary.json').write_text(json.dumps(jsonable(result),indent=2)+'\n');results.append(jsonable(result))
    result={'completed':True,'examples':results,'scope':'Original five-state model and paper equations; declared unspecified gains and mechanism geometry, not exact author figures.'};(out/'summary.json').write_text(json.dumps(result,indent=2)+'\n');return result
