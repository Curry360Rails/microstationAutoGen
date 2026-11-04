"""OSM/GeoJSON railway centerlines -> native MicroStation lines and circular arcs.
Run within MicroStation Python Manager; use --dry-run outside Bentley.
Requires numpy; raw OSM lon/lat also requires pyproj.
"""
import argparse
import json
import math
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent

import numpy as np


def args_parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('source', type=Path, help='Overpass raw OSM JSON or railway GeoJSON')
    p.add_argument('-c', '--curves', action='store_true', help='Fit and draw native circular arcs')
    p.add_argument('-s', '--spirals', action='store_true', help='Allow explicit spiral records only (not inferred from OSM)')
    p.add_argument('--dry-run', action='store_true', help='Fit and report without Bentley imports or DGN writes')
    p.add_argument('--write', action='store_true', help='Explicitly permit writing to active DGN')
    p.add_argument('--epsg', type=int, default=6424, help='Target projected CRS (default: 6424, US survey feet)')
    p.add_argument('--source-epsg', type=int, help='GeoJSON source EPSG if projected; default 4326')
    p.add_argument('--max-deviation', type=float, default=1.5, help='Maximum radial residual, meters')
    p.add_argument('--min-arc-points', type=int, default=5)
    p.add_argument('--min-arc-angle', type=float, default=3.0, help='Minimum arc sweep, degrees')
    p.add_argument('--max-fit-points', type=int, default=250, help='Limit candidate circle-fit window for performance')
    p.add_argument('--min-radius', type=float, default=15.0)
    p.add_argument('--max-radius', type=float, default=20000.0)
    p.add_argument('--max-elements', type=int, default=None)
    p.add_argument('--origin', nargs=2, type=float, metavar=('X','Y'), help='Diagnostic local origin in TARGET CRS units (feet for EPSG:6424); changes placement')
    p.add_argument('--uor-per-meter', type=float, help='Override only after checking DGN working units')
    p.add_argument('--progress-every', type=int, default=10)
    p.add_argument('--report', type=Path, default=HERE / 'rail_fit_report.json',
                   help='Report filename or path; output is saved beside this script')
    return p


def transform_factory(source_epsg, target_epsg):
    if source_epsg == target_epsg:
        return lambda x, y: (float(x), float(y))
    try:
        from pyproj import Transformer
    except ImportError as exc:
        raise RuntimeError('Install pyproj to transform coordinates: python -m pip install pyproj') from exc
    t = Transformer.from_crs(source_epsg, target_epsg, always_xy=True)
    return t.transform


def target_unit_to_meter(epsg):
    """Meters per coordinate unit, from the target CRS definition."""
    try:
        from pyproj import CRS
    except ImportError as exc:
        raise RuntimeError('Install pyproj for CRS unit verification') from exc
    crs = CRS.from_epsg(epsg)
    if not crs.is_projected or len(crs.axis_info) < 2:
        raise ValueError(f'EPSG:{epsg} must be a projected CRS with linear axes')
    axes = crs.axis_info[:2]
    scales = [float(axis.unit_conversion_factor) for axis in axes]
    if not all(math.isfinite(v) and v > 0 for v in scales):
        raise ValueError(f'Invalid linear units for EPSG:{epsg}')
    if not math.isclose(scales[0], scales[1], rel_tol=1e-12):
        raise ValueError('Target CRS axes use different units')
    return scales[0], axes[0].unit_name


def read_tracks(path, epsg, source_epsg):
    obj = json.loads(path.read_text(encoding='utf-8'))
    tracks = []
    if obj.get('type') == 'FeatureCollection':
        src = source_epsg or 4326
        conv = transform_factory(src, epsg)
        for i, feature in enumerate(obj.get('features', [])):
            props = feature.get('properties') or {}
            if 'railway' in props and props['railway'] not in ('rail', 'light_rail', 'subway', 'tram', 'narrow_gauge', 'industrial', 'disused', 'construction'):
                continue
            geom = feature.get('geometry') or {}
            typ = geom.get('type')
            coords = geom.get('coordinates') or []
            paths = [coords] if typ == 'LineString' else coords if typ == 'MultiLineString' else []
            for j, line in enumerate(paths):
                pts = [list(conv(pt[0], pt[1])) for pt in line]
                if len(pts) >= 2:
                    tracks.append({'id': str(props.get('id', feature.get('id', i))) + ':' + str(j), 'points': pts})
    elif 'elements' in obj:
        conv = transform_factory(4326, epsg)
        nodes = {e['id']: (e['lon'], e['lat']) for e in obj['elements'] if e.get('type') == 'node' and 'lat' in e and 'lon' in e}
        for way in obj['elements']:
            if way.get('type') != 'way' or 'railway' not in way.get('tags', {}):
                continue
            tags = way['tags']
            if tags['railway'] not in ('rail', 'light_rail', 'subway', 'tram', 'narrow_gauge', 'industrial', 'disused', 'construction'):
                continue
            raw = [nodes[n] for n in way.get('nodes', []) if n in nodes]
            if len(raw) < 2 and way.get('geometry'):
                raw = [(v['lon'], v['lat']) for v in way['geometry']]
            if len(raw) >= 2:
                tracks.append({'id': str(way['id']), 'points': [list(conv(*pt)) for pt in raw]})
    else:
        raise ValueError('Expected Overpass raw JSON with elements or GeoJSON FeatureCollection')
    if not tracks:
        raise ValueError('No railway line features found')
    return tracks


def clean_points(pts):
    out = []
    for p in pts:
        if not out or math.dist(p, out[-1]) > 0.001:
            out.append(p)
    return out


def join_ways(tracks, tolerance=0.25):
    """Join degree-2 endpoints only; never traverse junctions of degree >=3."""
    # Endpoint clusters by quantized XY. Typical OSM shared nodes are exact.
    def key(p):
        return (round(p[0]/tolerance), round(p[1]/tolerance))
    tracks = [{'id': t['id'], 'points': clean_points(t['points'])} for t in tracks]
    tracks = [t for t in tracks if len(t['points']) >= 2]
    adj = {}
    for i,t in enumerate(tracks):
        for side in (0,-1):
            adj.setdefault(key(t['points'][side]), []).append((i,side))
    visited=set(); chains=[]
    def walk(i, reverse):
        ids=[]; pts=[]
        while i not in visited:
            visited.add(i)
            t=tracks[i]
            segment=list(reversed(t['points'])) if reverse else t['points']
            pts.extend(segment if not pts else segment[1:]); ids.append(t['id'])
            endkey=key(segment[-1]); connections=adj.get(endkey, [])
            if len(connections)!=2: break
            nexts=[(j,side) for j,side in connections if j not in visited]
            if len(nexts)!=1: break
            i,side=nexts[0]
            reverse=(side == -1)
        return {'ids':ids, 'points':clean_points(pts)}
    # Start at non-degree-2 nodes to keep junctions as boundaries.
    for i,t in enumerate(tracks):
        if i in visited:continue
        a,b=key(t['points'][0]),key(t['points'][-1])
        if len(adj[a])!=2 or len(adj[b])!=2:
            chains.append(walk(i,len(adj[a])==2 and len(adj[b])!=2))
    for i in range(len(tracks)):
        if i not in visited:chains.append(walk(i,False))
    return chains


def fit_circle(points):
    a=np.asarray(points,dtype=float)
    origin=a.mean(axis=0)
    xy=a-origin
    mat=np.column_stack((2*xy[:,0],2*xy[:,1],np.ones(len(xy))))
    rhs=np.sum(xy*xy,axis=1)
    sol,_,rank,_=np.linalg.lstsq(mat,rhs,rcond=None)
    if rank<3:return None
    center=sol[:2]+origin
    radius2=float(np.dot(sol[:2],sol[:2])+sol[2])
    if radius2<=0:return None
    r=math.sqrt(radius2)
    residual=np.linalg.norm(a-center,axis=1)-r
    ang=np.unwrap(np.arctan2(a[:,1]-center[1],a[:,0]-center[0]))
    sweep=float(ang[-1]-ang[0])
    if abs(sweep)>math.pi*1.9:return None
    dif=np.diff(ang)
    if not (np.all(dif>=-1e-4) or np.all(dif<=1e-4)):return None
    return {'center':center.tolist(),'radius_m':r,'start_angle':float(ang[0]),'sweep_rad':sweep,
            'max_error_m':float(np.max(np.abs(residual))), 'rms_error_m':float(np.sqrt(np.mean(residual**2)))}


def arc_valid(fit, a, opts):
    return bool(fit and opts.min_radius<=fit['radius_m']<=opts.max_radius
                and fit['max_error_m']<=opts.max_deviation
                and abs(math.degrees(fit['sweep_rad']))>=opts.min_arc_angle
                and math.dist(a[0],a[-1])>0.01)


def detect_pieces(points, opts):
    """Longest-first arc extraction; absorb small intermediate straight fragments."""
    n=len(points)
    if not opts.curves or n<opts.min_arc_points:
        return [{'type':'line','points':points}]
    candidates=[]
    # Try long spans first; avoids greedy local fragmentation.
    for i in range(n-opts.min_arc_points+1):
        for j in range(min(n, i+opts.max_fit_points), i+opts.min_arc_points-1,-1):
            if j-i<opts.min_arc_points:break
            fit=fit_circle(points[i:j])
            if arc_valid(fit,points[i:j],opts):
                candidates.append((i,j,fit))
                break
    candidates.sort(key=lambda x:(-(x[1]-x[0]),x[0]))
    used=np.zeros(n-1,dtype=bool)
    chosen=[]
    for i,j,fit in candidates:
        if used[i:j-1].any():continue
        used[i:j-1]=True
        chosen.append((i,j,fit))
    chosen.sort()
    pieces=[];pos=0
    for i,j,fit in chosen:
        if i>pos:
            pieces.append({'type':'line','points':points[pos:i+1]})
        pieces.append({'type':'arc',**fit,'start':points[i],'end':points[j-1], 'n_fit_points':j-i})
        pos=j-1
    if pos<n-1:
        pieces.append({'type':'line','points':points[pos:]})
    if not pieces:pieces=[{'type':'line','points':points}]
    return pieces


def main():
    opts=args_parser().parse_args()
    # All generated files go beside this script, even when a relative or
    # absolute path is supplied to --report.
    opts.report = HERE / opts.report.name
    if opts.spirals:
        raise ValueError('-s requires fitted spiral records, which raw OSM/GeoJSON do not provide. Spiral inference is not implemented; no invented spirals will be drawn.')
    unit_to_m, unit_name = target_unit_to_meter(opts.epsg)
    tracks = read_tracks(opts.source, opts.epsg, opts.source_epsg)
    # Fit in meters, so all fit tolerances and radii retain their advertised units.
    # The projection's axis directions and origin are preserved; only units change.
    for track in tracks:
        track['points'] = [[v * unit_to_m for v in xy] for xy in track['points']]
    chains = join_ways(tracks, tolerance=0.25)
    print(f'Target EPSG:{opts.epsg}; source coordinate unit: {unit_name}; '
          f'meters per coordinate unit: {unit_to_m:.12f}', flush=True)
    print(f'Loaded {len(tracks)} railway ways; joined into {len(chains)} chains',flush=True)
    start=time.monotonic()
    output=[]; counts={'line':0,'arc':0}
    for k,chain in enumerate(chains,1):
        pieces=detect_pieces(chain['points'],opts)
        output.append({'ids':chain['ids'],'pieces':pieces})
        for piece in pieces:counts[piece['type']]+=1
        if k%max(1,opts.progress_every)==0 or k==len(chains):
            elapsed=time.monotonic()-start
            print(f'Fitting {k}/{len(chains)} ({k/len(chains):.0%}), elapsed {elapsed:.1f}s',flush=True)
    opts.report.parent.mkdir(parents=True, exist_ok=True)
    opts.report.write_text(json.dumps({'source':str(opts.source),'epsg':opts.epsg,'geometry_coordinate_unit':'meters (projected coordinates scaled by target CRS unit-to-meter factor)','target_crs_unit':unit_name,'meters_per_target_unit':unit_to_m,'counts':counts,'chains':output},indent=2),encoding='utf-8')
    print('Fit report:',opts.report,'counts:',counts,flush=True)
    if opts.dry_run or not opts.write:
        print('DRY RUN: no MicroStation elements inserted. Use --write to insert.',flush=True)
        return
    
    from MSPyBentleyGeom import DPoint3d, DPoint3dArray
    from MSPyDgnPlatform import EditElementHandle, LineStringHandler, ArcHandler
    from MSPyMstnPlatform import ISessionMgr
    ref=ISessionMgr.GetActiveDgnModelRef()
    if ref is None or ref.GetDgnModel() is None:raise RuntimeError('Open a writable destination DGN')
    model=ref.GetDgnModel()
    factor=opts.uor_per_meter or float(model.GetModelInfo().GetUorPerMeter())
    if not math.isfinite(factor) or factor <= 0:
        raise RuntimeError('Invalid destination UOR-per-meter')
    print('Active model:',model.GetModelName(),'UOR per meter (verify):',factor,flush=True)
    print('WARNING: confirm destination DGN GCS matches EPSG:',opts.epsg,flush=True)
    print('Geometry is stored in meters after target-CRS unit conversion.',flush=True)
    ox,oy=tuple(v * unit_to_m for v in opts.origin) if opts.origin else (0.0,0.0)
    def pt(p):return DPoint3d((p[0]-ox)*factor,(p[1]-oy)*factor,0.0)
    def success(status):
        if status is None:
            return False
        if str(status) in ('0', 'BentleyStatus.eSUCCESS', 'eSUCCESS'):
            return True
        try:
            return int(status) == 0
        except (TypeError, ValueError):
            return False
    def write_piece(piece):
        eeh=EditElementHandle()
        if piece['type']=='arc':
            radius=piece['radius_m']*factor
            status=ArcHandler.CreateArcElement(eeh,None,pt(piece['center']),radius,radius,0.0,
                                               piece['start_angle'],piece['sweep_rad'],ref.Is3d(),ref)
        else:
            arr=DPoint3dArray()
            for p in piece['points']:arr.append(pt(p))
            status=LineStringHandler.CreateLineStringElement(eeh,None,arr,ref.Is3d(),ref)
        if not success(status):raise RuntimeError('Create failed: '+str(status))
        result=eeh.AddToModel()
        if result is not None and not success(result):raise RuntimeError('AddToModel failed: '+str(result))
    # Split line strings to stay within the conservative 100-vertex limit.
    for chain in output:
        expanded=[]
        for piece in chain['pieces']:
            if piece['type']=='line' and len(piece['points'])>100:
                pts=piece['points']
                expanded.extend({'type':'line','points':pts[i:i+100]} for i in range(0,len(pts)-1,99))
            else:expanded.append(piece)
        chain['pieces']=expanded
    # Write separate native elements; no false claim that this is a complex chain.
    made=0; total=sum(len(c['pieces']) for c in output)
    if opts.max_elements is not None:total=min(total,opts.max_elements)
    started=time.monotonic()
    for ci,chain in enumerate(output):
        for piece in chain['pieces']:
            if opts.max_elements is not None and made>=opts.max_elements:break
            try:write_piece(piece)
            except Exception as exc:raise RuntimeError(f'Failed chain {ci} after {made} writes. Do not rerun in same DGN.') from exc
            made+=1
            if made%max(1,opts.progress_every)==0 or made==total:
                elapsed=time.monotonic()-started
                eta=elapsed/made*(total-made) if made else 0
                print(f'Drawing [{"#"*round(30*made/total):<30}] {made}/{total} {made/total:.0%} ETA {eta:.1f}s',flush=True)
        if opts.max_elements is not None and made>=opts.max_elements:break
    print(f'Inserted {made} native line/arc elements. No turnout cells or spirals.',flush=True)

if __name__=='__main__':main()
