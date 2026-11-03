"""Export a reviewable public source archive using an explicit allowlist."""
from pathlib import Path
import argparse,datetime,hashlib,json,zipfile
ROOT=Path(__file__).resolve().parents[1]
ROOT_FILES={'README.md','README_ZH.md','LICENSE','LICENSE_STATUS.md','THIRD_PARTY_NOTICES.md','CITATION.cff','pyproject.toml','requirements-tested.txt','requirements-original-tested.txt','uv.lock','MANIFEST.in','.gitignore'}
DIRECTORIES={'src','docs','examples','tests','scripts','evidence','.github','licenses'}

def files():
    result=[]
    for p in ROOT.rglob('*'):
        rel=p.relative_to(ROOT)
        if not p.is_file() or p.is_symlink():continue
        if any(s in {'__pycache__','.venv','.upstream','.local-reference','runs','dist','build','.git'} or s.endswith('.egg-info') for s in rel.parts):continue
        if len(rel.parts)==1 and rel.name in ROOT_FILES or len(rel.parts)>1 and rel.parts[0] in DIRECTORIES:
            if p.suffix not in {'.pyc','.pyo'}:result.append(p)
    return sorted(result)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,default=ROOT/'dist'/(ROOT.name+'-public.zip'));a=ap.parse_args()
    selected=files();entries=[]
    for p in selected:
        content=p.read_bytes()
        if p.suffix in {'.py','.md','.json','.toml','.yml','.cff','.txt'} and ((b'/' + b'Users/') in content or (b'/' + b'home/') in content):raise RuntimeError('Private absolute path in public file: '+str(p.relative_to(ROOT)))
        entries.append({'path':str(p.relative_to(ROOT)),'bytes':len(content),'sha256':hashlib.sha256(content).hexdigest()})
    manifest={'created_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'repo':ROOT.name,'files':entries,'excluded':['.git','.venv','.upstream','.local-reference','runs','dist','build'],'contains_author_unlicensed_source':False,'scope':'Only explicit source/documentation allowlist; numeric evidence is separate from author source.'}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    if a.output.exists():raise FileExistsError(a.output)
    with zipfile.ZipFile(a.output,'w',zipfile.ZIP_DEFLATED) as z:
        for p in selected:z.write(p,ROOT.name+'/'+str(p.relative_to(ROOT)))
        z.writestr(ROOT.name+'/PUBLIC_MANIFEST.json',json.dumps(manifest,indent=2)+'\n')
    a.output.with_suffix('.manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({'archive':str(a.output),'files':len(selected),'bytes':a.output.stat().st_size,'sha256':hashlib.sha256(a.output.read_bytes()).hexdigest()},indent=2))
if __name__=='__main__':main()
