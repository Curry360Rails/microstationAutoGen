"""Rank turnout cell candidates using OSM branch geometry and decoded DGN arc metadata.

Read-only. Requires pyproj; uses native arc records and nominal turnout number.
Results are hypotheses, not verified cell placements or surveyed turnout IDs.
"""
import argparse,csv,json,math,re
from collections import defaultdict
from pathlib import Path
from pyproj import Transformer

NUM=re.compile(r'^(\d+)\s+(LH|RH)')
def norm(a):return (a+math.pi)%(2*math.pi)-math.pi
def angle(a,b):return abs(norm(a-b))
def dist(a,b):return math.hypot(a[0]-b[0],a[1]-b[1])
def az(a,b):return math.atan2(b[1]-a[1],b[0]-a[0])
def walk(path,xy,reach):
    p=[xy[n] for n in path if n in xy]
    if len(p)<2:return None
    out=[p[0]]; walked=0.
    for a,b in zip(p,p[1:]):
        d=dist(a,b)
        if d<1e-7:continue
        if walked+d>=reach:
            t=(reach-walked)/d;out.append((a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])));break
        out.append(b);walked+=d
    return out if len(out)>1 else None
def at_length(points,length):
    remain=length
    for a,b in zip(points,points[1:]):
        d=dist(a,b)
        if remain<=d:return (a[0]+(b[0]-a[0])*remain/d,a[1]+(b[1]-a[1])*remain/d) if d else a
        remain-=d
    return points[-1]
def recursive_arcs(elems):
    for e in elems:
        g=e.get('geometry'); gs=g if isinstance(g,list) else [g] if isinstance(g,dict) else []
        for v in gs:
            if v.get('kind')=='arc' and v.get('primary_radius') and v.get('sweep_angle_rad'):
                yield v
        yield from recursive_arcs(e.get('children',[]))
def catalog(data,scale):
    models=[]
    for m in data['models']:
        match=NUM.match(m.get('model',''))
        if not match:continue
        n=int(match.group(1));hand=match.group(2)
        arcs=[]
        for a in recursive_arcs(m.get('elements',[])):
            radius=float(a['primary_radius'])*scale
            sweep=abs(float(a['sweep_angle_rad']))
            if .5<=radius<=2000 and .002<=sweep<=.8:
                arcs.append({'radius_m':radius,'sweep_deg':math.degrees(sweep),'arc_length_m':radius*sweep})
        models.append({'name':m['model'],'number':n,'hand':hand,'arcs':arcs})
    return models

def run(args):
    raw=json.loads(args.osm.read_text(encoding='utf8'))
    lib=json.loads(args.library.read_text(encoding='utf8'))
    model0=next(m for m in lib['models'] if NUM.match(m.get('model','')))
    units=model0['units']
    # Conversion is a hypothesis: raw coords assumed UOR and master units assumed US survey feet.
    inferred=float(units['meters_per_master_unit'])/float(units['uor_per_storage'])
    scale=args.meters_per_raw if args.meters_per_raw is not None else inferred
    models=catalog(lib,scale)
    tr=Transformer.from_crs('EPSG:4326','EPSG:26911',always_xy=True)
    nodes={}
    for e in raw['elements']:
        if e['type']=='node' and 'lat' in e and 'lon' in e:
            if e['id'] not in nodes or e.get('tags',{}).get('railway')=='switch':
                nodes[e['id']]=e
    xy={i:tr.transform(n['lon'],n['lat']) for i,n in nodes.items()}
    ways=[e for e in raw['elements'] if e['type']=='way' and e.get('tags',{}).get('railway') in ('rail','light_rail','narrow_gauge','industrial','tram')]
    adjacency=defaultdict(set)
    for w in ways:
        for a,b in zip(w.get('nodes',[]),w.get('nodes',[])[1:]):
            if a in xy and b in xy and a!=b:adjacency[a].add(b);adjacency[b].add(a)
    switches=[n for n in nodes.values() if n.get('tags',{}).get('railway')=='switch']
    stop={n['id'] for n in switches}
    def trace(root,neighbor):
        path=[root,neighbor];prev=root;cur=neighbor
        while len(path)<300 and cur not in stop and len(adjacency[cur])==2:
            nex=next((v for v in adjacency[cur] if v!=prev),None)
            if nex is None or nex in path:break
            path.append(nex);prev,cur=cur,nex
        return path
    out=[];features=[]
    for node in switches:
        nid=node['id'];arms=[]
        for nb in sorted(adjacency[nid]):
            line=walk(trace(nid,nb),xy,args.reach)
            if line and dist(line[0],line[-1])>2:
                bearing=az(line[0],line[-1]);
                if all(angle(bearing,a['bearing'])>math.radians(.3) for a in arms):arms.append({'points':line,'bearing':bearing})
        scored=[]
        for i in range(len(arms)):
            for j in range(i+1,len(arms)):
                opp=abs(math.pi-angle(arms[i]['bearing'],arms[j]['bearing']))
                if opp>math.radians(args.through_tolerance):continue
                for forward in (i,j):
                    for branch in range(len(arms)):
                        if branch in (i,j):continue
                        straight=arms[forward]['points']; diverge=arms[branch]['points']
                        direction=arms[forward]['bearing'];branchdir=arms[branch]['bearing']
                        delta=norm(branchdir-direction)
                        if abs(delta)>math.radians(args.max_branch_angle):continue
                        # Observed cross-track offset at multiple distances from the switch.
                        samples=[]
                        for reach in (10.,20.,30.,40.):
                            p=at_length(diverge,reach);s=at_length(straight,reach)
                            baseline=xy[nid]
                            along=(p[0]-baseline[0])*math.cos(direction)+(p[1]-baseline[1])*math.sin(direction)
                            lateral=-(p[0]-baseline[0])*math.sin(direction)+(p[1]-baseline[1])*math.cos(direction)
                            if along>=2:samples.append((along,lateral))
                        if len(samples)<2:continue
                        # Radius and frog-angle scoring. Unknown toe position is searched over a short range.
                        for m in models:
                            if not m['arcs']:continue
                            nominal=math.atan(1/m['number'])
                            # OSM side relative to the selected forward direction, not verified cell handedness.
                            side='LH' if delta>0 else 'RH'
                            hand_penalty=0 if side==m['hand'] else 1.0
                            best=None
                            for arc in m['arcs']:
                                r=arc['radius_m']; theta=math.radians(arc['sweep_deg'])
                                # Only test arcs with meaningful deflection and plausible frog-angle relationship.
                                if theta < nominal*.25 or theta>nominal*3:continue
                                for toe in (0.,5.,10.,15.,20.):
                                    residual=[]
                                    for along,lateral in samples:
                                        x=max(0.,along-toe)
                                        # Circular branch followed by tangent; approximate idealized centerline.
                                        arclen=r*theta
                                        if x<=arclen:
                                            predicted=r*(1-math.cos(x/r))
                                        else:
                                            predicted=r*(1-math.cos(theta))+(x-arclen)*math.sin(theta)
                                        residual.append((abs(lateral)-predicted)**2)
                                    rmse=math.sqrt(sum(residual)/len(residual))
                                    penalty=abs(abs(delta)-nominal)*args.angle_weight+hand_penalty
                                    score=rmse+penalty
                                    if best is None or score<best['score']:
                                        best={'score':score,'rmse_m':rmse,'toe_offset_m':toe,'radius_m':r,'arc_sweep_deg':math.degrees(theta),'hand':side,'observed_angle_deg':math.degrees(abs(delta)),'through_opposition_error_deg':math.degrees(opp)}
                            if best:scored.append((best['score'],m,best))
        scored.sort(key=lambda a:a[0]);top=[];seen=set()
        for score,m,fit in scored:
            if m['name'] in seen:continue
            seen.add(m['name']);top.append((m,fit))
            if len(top)>=args.top:break
        if not arms:status='no_connected_arms'
        elif len(arms)<3:status='insufficient_arms'
        elif not top:status='no_acceptable_template'
        else:status='ranked_unverified'
        row={'osm_node_id':nid,'longitude':node['lon'],'latitude':node['lat'],'arms':len(arms),'status':status,
             'best_model':top[0][0]['name'] if top else '',
             'best_number':top[0][0]['number'] if top else '',
             'best_hand':top[0][0]['hand'] if top else '',
             'best_rmse_m':round(top[0][1]['rmse_m'],3) if top else '',
             'best_score':round(top[0][1]['score'],3) if top else '',
             'best_radius_m':round(top[0][1]['radius_m'],3) if top else '',
             'best_toe_offset_m':top[0][1]['toe_offset_m'] if top else '',
             'candidates_json':json.dumps([{'model':m['name'],'number':m['number'],**{k:round(v,3) if isinstance(v,float) else v for k,v in fit.items()}} for m,fit in top]),
             'confidence':'unverified' }
        out.append(row);features.append({'type':'Feature','geometry':{'type':'Point','coordinates':[node['lon'],node['lat']]},'properties':row})
    args.out.mkdir(parents=True,exist_ok=True)
    with (args.out/'turnout_rankings.csv').open('w',newline='',encoding='utf8') as f:
        writer=csv.DictWriter(f,fieldnames=list(out[0]));writer.writeheader();writer.writerows(out)
    (args.out/'turnout_rankings.geojson').write_text(json.dumps({'type':'FeatureCollection','features':features},indent=2),encoding='utf8')
    summary={'switches':len(out),'ranked':sum(r['status']=='ranked_unverified' for r in out),'unresolved':sum(r['status']!='ranked_unverified' for r in out),'catalog_models':len(models),'scale_m_per_raw':scale,'scale_assumption':'raw coordinates interpreted as UOR; master unit from metadata; NOT verified','method':'local branch profile vs circular candidate, toe-offset search; approximate, not full CAD registration','projection':'EPSG:26911'}
    (args.out/'summary.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('osm',type=Path);p.add_argument('library',type=Path)
    p.add_argument('--out',type=Path,default=Path('turnout_rankings'))
    p.add_argument('--meters-per-raw',type=float,default=None)
    p.add_argument('--reach',type=float,default=60.)
    p.add_argument('--through-tolerance',type=float,default=35.)
    p.add_argument('--max-branch-angle',type=float,default=35.)
    p.add_argument('--angle-weight',type=float,default=30.)
    p.add_argument('--top',type=int,default=5)
    run(p.parse_args())
