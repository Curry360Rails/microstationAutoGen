"""Offline OSM track chaining and conservative circular arc approximation. Python stdlib only."""
import argparse, json, math
from collections import defaultdict
from pathlib import Path

def distance(a,b): return math.hypot(a[0]-b[0],a[1]-b[1])
def circle(a,b,c):
    ax,ay=a[:2];bx,by=b[:2];cx,cy=c[:2]
    d=2*(ax*(by-cy)+bx*(cy-ay)+cx*(ay-by))
    if abs(d)<1e-8:return None
    u=((ax*ax+ay*ay)*(by-cy)+(bx*bx+by*by)*(cy-ay)+(cx*cx+cy*cy)*(ay-by))/d
    v=((ax*ax+ay*ay)*(cx-bx)+(bx*bx+by*by)*(ax-cx)+(cx*cx+cy*cy)*(bx-ax))/d
    return [u,v],distance([u,v],a)
def radii_from_v14(path):
    if not path:return []
    data=json.loads(Path(path).read_text(encoding='utf8')); out=[]
    for model in data.get('models',[]):
        if model.get('model','').startswith(('Default','CN#')):continue
        for element in model.get('elements',[]):
            stack=[element]
            while stack:
                obj=stack.pop();stack.extend(obj.get('children',[]))
                geom=obj.get('geometry',[])
                if isinstance(geom,dict):geom=[geom]
                for g in geom:
                    if g.get('kind')=='arc' and g.get('status')=='native_2d_decoded':
                        r=g.get('primary_radius');unit=model.get('units',{})
                        # Raw geometry is in DGN UORs; 10000 UOR per storage unit.
                        # Source cell models have feet master units; use storage-unit conversion
                        # only with explicit calibrated meters-per-storage factor.
                        if isinstance(r,(float,int)) and r>0:out.append({'model':model['model'],'raw_radius':r,'uor_per_storage':unit.get('uor_per_storage')})
    return out

def prepare(plan, catalogue, meters_per_storage, tolerance=0.35, max_radius_rel_error=.06):
    # One undirected edge per adjacent pair of OSM nodes; duplicate geometry is de-duplicated.
    nodes={};edges={};adj=defaultdict(set); tags={}
    for sw in plan['switches']:
        if sw.get('tagged'):tags[str(sw['node_id'])]=True
    for w in plan['tracks']:
        ids=w.get('node_ids',[]);pts=w['points']
        if len(ids)!=len(pts):raise ValueError('Mismatched node IDs and coordinates in way '+str(w['way_id']))
        for nid,p in zip(ids,pts):nodes[str(nid)]=p[:2]
        for a,b in zip(ids,ids[1:]):
            a,b=str(a),str(b)
            if a==b or distance(nodes[a],nodes[b])<1e-7:continue
            key=tuple(sorted((a,b)))
            if key not in edges:edges[key]={'nodes':key,'way_ids':[w['way_id']]}
            else:edges[key]['way_ids'].append(w['way_id'])
            adj[a].add(key);adj[b].add(key)
    breaks={n for n in adj if len(adj[n])!=2 or n in tags}
    visited=set();chains=[]
    def walk(start,edge):
        path=[start];curr=start;key=edge
        while key not in visited:
            visited.add(key);a,b=key;nxt=b if curr==a else a;path.append(nxt);curr=nxt
            if curr in breaks:break
            remaining=[e for e in adj[curr] if e not in visited]
            if not remaining:break
            key=remaining[0]
        return path
    for n in sorted(breaks):
        for edge in sorted(adj[n]):
            if edge not in visited:chains.append(walk(n,edge))
    for edge in sorted(edges):
        if edge not in visited:chains.append(walk(edge[0],edge))
    radii=sorted(set(round(r['raw_radius']/r['uor_per_storage']*meters_per_storage,5)
                     for r in catalogue if r.get('uor_per_storage') and r['raw_radius']>0))
    records=[];curves=0
    for idx,path in enumerate(chains):
        points=[nodes[n] for n in path];pieces=[];i=0
        while i<len(points)-1:
            # Candidate arc from three consecutive points. Only replace the middle vertex
            # if it lies on a catalogue-radius arc within tolerance.
            if i+2<len(points) and radii:
                a,b,c=points[i:i+3];fit=circle(a,b,c)
                if fit:
                    center,r=fit;candidate=min(radii,key=lambda x:abs(x-r))
                    if abs(candidate-r)/r<=max_radius_rel_error:
                        # For a fixed radius, center is on perpendicular bisector of endpoints.
                        chord=distance(a,c)
                        if chord>1e-7 and chord<2*candidate:
                            mid=[(a[0]+c[0])/2,(a[1]+c[1])/2]
                            h=math.sqrt(max(0,candidate*candidate-(chord/2)**2))
                            normal=[-(c[1]-a[1])/chord,(c[0]-a[0])/chord]
                            centers=[[mid[0]+s*h*normal[0],mid[1]+s*h*normal[1]] for s in (-1,1)]
                            chosen=min(centers,key=lambda cc:abs(distance(cc,b)-candidate))
                            err=abs(distance(chosen,b)-candidate)
                            t0=math.atan2(a[1]-chosen[1],a[0]-chosen[0]);tm=math.atan2(b[1]-chosen[1],b[0]-chosen[0]);t1=math.atan2(c[1]-chosen[1],c[0]-chosen[0])
                            ccw=(t1-t0)%(2*math.pi); midccw=(tm-t0)%(2*math.pi)
                            sweep=ccw if midccw<=ccw else ccw-2*math.pi
                            if err<=tolerance and abs(sweep)<math.pi/2:
                                pieces.append({'type':'arc','start':a,'end':c,'center':chosen,'radius_m':candidate,'sweep_rad':sweep,'fit_radius_m':r,'midpoint_error_m':err})
                                curves+=1;i+=2;continue
            pieces.append({'type':'line','start':points[i],'end':points[i+1]});i+=1
        # Consecutive lines are grouped into a single MicroStation line string.
        grouped=[];linepoints=[]
        def flush():
            nonlocal linepoints
            if len(linepoints)>1:grouped.append({'type':'line_string','points':linepoints})
            linepoints=[]
        for piece in pieces:
            if piece['type']=='line':
                if not linepoints:linepoints=[piece['start']]
                linepoints.append(piece['end'])
            else:flush();grouped.append(piece)
        flush()
        records.append({'chain_id':idx,'node_ids':path,'start_node':path[0],'end_node':path[-1], 'pieces':grouped})
    return {'schema':'osm_microstation_v3','crs_epsg':26911,'chains':records,'switches':plan['switches'],
            'catalogue_radii_m':radii,'stats':{'original_ways':len(plan['tracks']),'unique_edges':len(edges),'chains':len(records),'fitted_arc_pieces':curves,'drawing_pieces':sum(len(c['pieces']) for c in records)},
            'notes':'Arc fitting is conservative and only uses 3-point windows. No turnout cell placement or trimming.'}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('plan');ap.add_argument('--v14');ap.add_argument('--meters-per-storage',type=float,default=1.0,help='Explicit conversion of v14 cell storage units to meters; verify with cell geometry before using')
    ap.add_argument('--out',default='railway_chains_v3.json');ap.add_argument('--fit-tolerance',type=float,default=.35);args=ap.parse_args()
    plan=json.loads(Path(args.plan).read_text(encoding='utf8'))
    if plan.get('crs_epsg')!=26911:raise ValueError('Plan must be EPSG:26911')
    cat=radii_from_v14(args.v14);result=prepare(plan,cat,args.meters_per_storage,args.fit_tolerance)
    Path(args.out).write_text(json.dumps(result,indent=2),encoding='utf8');print(result['stats']);print('Catalogue radii:',len(result['catalogue_radii_m']))
if __name__=='__main__':main()
