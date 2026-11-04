"""MicroStation 2025 turnout placement preparation and guarded native insertion.

Inputs: raw Overpass JSON, turnout_rankings.csv, and user-confirmed cell anchors.
No placement is attempted without explicit anchor data and --write.
"""
import argparse
import csv
import json
import math
from pathlib import Path


def read_switches(path):
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    nodes = {int(e['id']): e for e in data['elements'] if e['type'] == 'node'}
    ways = [e for e in data['elements'] if e['type'] == 'way' and e.get('tags', {}).get('railway') in ('rail', 'light_rail', 'industrial', 'disused', 'narrow_gauge')]
    return nodes, ways


def bearing(p, q):
    return math.atan2(q[1]-p[1], q[0]-p[0])


def wrap(a):
    return (a + math.pi) % (2*math.pi) - math.pi


def arms_at(node_id, nodes, ways, transformer, distance_m=25.0):
    """Find outward tangent arms by following OSM node sequences from the switch."""
    arms = []
    for w in ways:
        ids = w.get('nodes', [])
        for idx, nid in enumerate(ids):
            if nid != node_id:
                continue
            for direction in (-1, 1):
                j = idx + direction
                if not (0 <= j < len(ids)):
                    continue
                chain = [nid]
                while 0 <= j < len(ids) and ids[j] in nodes:
                    chain.append(ids[j])
                    p0 = nodes[nid]
                    pj = nodes[ids[j]]
                    x0,y0 = transformer.transform(p0['lon'],p0['lat'])
                    xj,yj = transformer.transform(pj['lon'],pj['lat'])
                    if math.hypot(xj-x0,yj-y0) >= distance_m:
                        break
                    j += direction
                if len(chain) < 2:
                    continue
                p0 = nodes[nid]; pj = nodes[chain[-1]]
                x0,y0 = transformer.transform(p0['lon'],p0['lat'])
                xj,yj = transformer.transform(pj['lon'],pj['lat'])
                if math.hypot(xj-x0,yj-y0) < 1:
                    continue
                arms.append({'way_id':w['id'], 'direction':direction, 'bearing_rad':bearing((x0,y0),(xj,yj)), 'end_node':chain[-1]})
    # Deduplicate coincident arms caused by shared OSM ways.
    unique=[]
    for a in arms:
        if all(abs(wrap(a['bearing_rad']-b['bearing_rad'])) > math.radians(2) for b in unique):
            unique.append(a)
    return unique


def choose_orientation(arms):
    if len(arms) < 3:
        return None
    # Through arms are the pair closest to 180 degrees apart.
    pairs=[(abs(math.pi-abs(wrap(a['bearing_rad']-b['bearing_rad']))),i,j)
           for i,a in enumerate(arms) for j,b in enumerate(arms) if i<j]
    error,i,j=min(pairs)
    remaining=[k for k in range(len(arms)) if k not in (i,j)]
    if not remaining:
        return None
    k=min(remaining,key=lambda z:min(abs(wrap(arms[z]['bearing_rad']-arms[i]['bearing_rad'])),abs(wrap(arms[z]['bearing_rad']-arms[j]['bearing_rad']))))
    # Choose the through tangent closest to the diverging arm, pointing outward.
    ti=min((i,j),key=lambda z:abs(wrap(arms[z]['bearing_rad']-arms[k]['bearing_rad'])))
    direction=arms[ti]['bearing_rad']
    signed=wrap(arms[k]['bearing_rad']-direction)
    return {'through_bearing_rad':direction,'branch_bearing_rad':arms[k]['bearing_rad'],
            'hand_observed': 'LH' if signed>0 else 'RH',
            'branch_angle_deg':abs(math.degrees(signed)),
            'through_opposition_error_deg':math.degrees(error)}


def load_anchors(path):
    """User-verified anchors: model -> local toe [x,y] in meters and axis angle in degrees."""
    data=json.loads(Path(path).read_text(encoding='utf-8'))
    return data.get('models',{})


def run(args):
    from pyproj import Transformer
    transformer=Transformer.from_crs(4326,26911,always_xy=True)
    nodes,ways=read_switches(args.osm)
    anchors=load_anchors(args.anchors) if args.anchors else {}
    with open(args.rankings,newline='',encoding='utf-8-sig') as f:
        rankings=list(csv.DictReader(f))
    if args.node:
        rankings=[r for r in rankings if str(r['osm_node_id'])==str(args.node)]
    if args.max_turnouts is not None:
        rankings=rankings[:args.max_turnouts]
    rows=[]
    for idx,r in enumerate(rankings,1):
        nid=int(r['osm_node_id']); n=nodes.get(nid)
        if not n:
            rows.append({'node_id':nid,'status':'missing_osm_node'});continue
        x,y=transformer.transform(n['lon'],n['lat'])
        orient=choose_orientation(arms_at(nid,nodes,ways,transformer))
        model=args.model or r.get('best_model','')
        row={'node_id':nid,'x_m':x,'y_m':y,'model':model,
             'ranked_confidence':r.get('confidence',''), 'ranked_rmse_m':r.get('best_rmse_m',''),
             'status':'unresolved_orientation' if orient is None else 'awaiting_verified_anchor'}
        if orient:
            row.update(orient)
            anchor=anchors.get(model)
            if anchor:
                toe=anchor['toe_xy_m']; local_angle=math.radians(float(anchor['through_bearing_deg']))
                rotation=wrap(orient['through_bearing_rad']-local_angle)
                # Rotate local toe point and translate to the OSM switch position.
                tx=x-(math.cos(rotation)*toe[0]-math.sin(rotation)*toe[1])
                ty=y-(math.sin(rotation)*toe[0]+math.cos(rotation)*toe[1])
                row.update({'rotation_rad':rotation,'translation_x_m':tx,'translation_y_m':ty,
                            'status':'placement_ready_unverified_fit'})
                hand=model.split(' ')[1][:2] if len(model.split(' '))>1 else ''
                if hand in ('LH','RH') and hand!=orient['hand_observed']:
                    row['status']='hand_mismatch'
        rows.append(row)
        print(f"[{idx}/{len(rankings)}] node={nid} model={model} {row['status']}",flush=True)
    out=Path(args.out);out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps({'epsg':26911,'source':str(args.osm),'placements':rows},indent=2),encoding='utf-8')
    print('Saved',out,flush=True)
    if args.write:
        ready=[r for r in rows if r['status']=='placement_ready_unverified_fit']
        if not ready:
            raise RuntimeError('No verified-anchor placements ready. No DGN elements created.')
        raise RuntimeError('Native cell insertion is intentionally disabled: the exact MicroStation 2025 cell-reference creation signature and source-cell model loading have not been verified. Placement transforms were saved for review; no DGN elements created.')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--osm',required=True,help='Raw Overpass JSON')
    p.add_argument('--rankings',required=True,help='turnout_rankings.csv')
    p.add_argument('--anchors',help='Verified per-cell toe/axis JSON (meters)')
    p.add_argument('--cel',help='Path to source .cel library; reserved for native insertion')
    p.add_argument('--node',type=int,help='Specific OSM switch node')
    p.add_argument('--model',help='Override cell model name')
    p.add_argument('--max-turnouts',type=int,default=1)
    p.add_argument(
        '--out',
        type=Path,
        default=Path(__file__).resolve().parent / 'turnout_placement_plan.json',
        help='Output JSON path (default: alongside this script)'
    )
    p.add_argument('--write',action='store_true',help='Require native placement; fails closed until writer verified')
    args=p.parse_args()
    run(args)

if __name__=='__main__':
    main()
