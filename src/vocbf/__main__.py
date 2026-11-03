import os
# One numerical thread per process, set before importing numerical packages.
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','VECLIB_MAXIMUM_THREADS','NUMEXPR_NUM_THREADS']:os.environ[key]='1'
import argparse,json
PACKAGE='vocbf'
SUITES=['all', 'static', 'moving']
from pathlib import Path

def main(argv=None):
    parser=argparse.ArgumentParser(description='VOCBF-Python: source reproduction and declared model extensions')
    sub=parser.add_subparsers(dest='command',required=True)
    run=sub.add_parser('run');run.add_argument('--model',choices=['single_integrator','double_integrator','kinematic_unicycle','dynamic_unicycle'],required=True);run.add_argument('--output',required=True);run.add_argument('--steps',type=int,default=400);run.add_argument('--dt',type=float,default=.05);run.add_argument('--offset',type=float,default=.15)
    demo=sub.add_parser('demo',help='Four-model straight-line destination / single-obstacle demo');demo.add_argument('--model',choices=['all','single_integrator','double_integrator','kinematic_unicycle','dynamic_unicycle'],default='all');demo.add_argument('--output',required=True);demo.add_argument('--steps',type=int,default=1200);demo.add_argument('--dt',type=float,default=.05);demo.add_argument('--guidance',choices=['waypoints','direct'],default='waypoints');demo.add_argument('--no-animation',action='store_true')
    verify=sub.add_parser('verify');verify.add_argument('--output')
    native=sub.add_parser('original');native.add_argument('--suite',choices=SUITES,default='all');native.add_argument('--output',required=True)
    if PACKAGE!='vocbf':sub.add_parser('fetch-original')
    args=parser.parse_args(argv)
    if args.command=='run':
        if args.steps<=0 or args.dt<=0:parser.error('steps and dt must be positive')
        from .simulation import run as simulate
        value=simulate(args.model,args.output,args.steps,args.dt,args.offset)
    elif args.command=='demo':
        if args.steps<=0 or not 0<args.dt<float('inf'):parser.error('steps and dt must be positive and finite')
        from .single_obstacle import run as demo_run
        value=demo_run(args.output,args.model,args.steps,args.dt,args.guidance,not args.no_animation)
    elif args.command=='verify':
        from .validation import verify as check
        from .simulation import jsonable
        value=jsonable(check())
        if args.output:
            p=Path(args.output)
            if p.exists():raise FileExistsError(p)
            p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
        print(json.dumps({k:v for k,v in value.items() if k!='records'},indent=2));return 0 if value['passed'] else 1
    elif args.command=='original':
        from .native_runner import original
        value=original(args.output,args.suite)
    else:
        from .upstream import fetch
        value=fetch()
    print(json.dumps(value,indent=2,allow_nan=False));return 0 if value.get('passed',True) else 1

if __name__=='__main__':raise SystemExit(main())
