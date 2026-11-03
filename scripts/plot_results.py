"""Render the bundled numerical example evidence; no controller is rerun."""
from pathlib import Path
import argparse,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
    root=Path(__file__).resolve().parents[1]
    ap=argparse.ArgumentParser();ap.add_argument('--data',type=Path,default=root/'evidence/examples');ap.add_argument('--output',type=Path,default=root/'evidence/example_trajectories.png');args=ap.parse_args()
    titles={'single_integrator':'Single integrator','double_integrator':'Double integrator','kinematic_unicycle':'Kinematic unicycle','dynamic_unicycle':'Dynamic unicycle'}
    fig,axes=plt.subplots(2,2,figsize=(11,7.4),layout='constrained')
    for ax,(kind,title) in zip(axes.flat,titles.items()):
        folder=args.data/kind;s=json.loads((folder/'summary.json').read_text());states=np.load(folder/'trajectory.npz',allow_pickle=False)['state'];position=states[:,:2].copy()
        if 'unicycle' in kind:position+=s['offset']*np.column_stack([np.cos(states[:,2]),np.sin(states[:,2])])
        color='#207b62' if s['outcome']=='goal_complete' else '#b36517'
        ax.plot(position[:,0],position[:,1],color=color,lw=2,label='Body center')
        ax.scatter(*position[0],s=35,color='#283347',marker='o',label='Start',zorder=3)
        ax.scatter(*position[-1],s=50,color=color,marker='s',label='Final state',zorder=4)
        ax.scatter(*s['goal'],s=110,color='#333333',marker='*',label='Goal',zorder=5)
        for o,r in zip(s['obstacles'],s['inflated_radii']):ax.add_patch(plt.Circle(o,r,color='#b8bcc2',alpha=.65))
        ax.set(xlim=(-.4,5.1),ylim=(-.7,3.1),xlabel='x [m]',ylabel='y [m]',aspect='equal')
        ax.set_title(title+' — '+s['outcome'].replace('_',' '),fontsize=11,color=color)
        ax.text(.02,.98,f"{s['executed_steps']*s['dt']:.2f} s | error {s['final_goal_error']:.3f} m | fallbacks {s['solver_fallbacks']}",transform=ax.transAxes,va='top',fontsize=9)
        ax.grid(alpha=.15)
    axes[0,0].legend(loc='lower right',fontsize=8)
    fig.suptitle(root.name+' | Illustrative model extensions\nSampled trajectories; no comparative ranking or continuous-time safety proof',fontsize=13)
    args.output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(args.output,dpi=160);plt.close(fig)
if __name__=='__main__':main()
