"""Turnout placement planning and guarded native cell placement for MicroStation 2025.

Reads raw Overpass JSON, turnout ranking CSV, and extracted turnout_geometry_v14.json.
Automatically estimates anchors from extracted geometry. Native placement uses --write; no anchor confirmation flags.
"""
import argparse
import csv
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent

def angle(a): return (a + math.pi) % (2 * math.pi) - math.pi

def distance(a, b): return math.hypot(a[0]-b[0], a[1]-b[1])

def transform_point(p, scale, rotation, translation, anchor):
    x=(p[0]-anchor[0])*scale; y=(p[1]-anchor[1])*scale
    c=math.cos(rotation);s=math.sin(rotation)
    return (translation[0]+c*x-s*y, translation[1]+s*x+c*y)

def osm_arms(node_id, nodes, ways, transformer, reach, tangent_window=12.0):
    """Follow connected OSM ways and fit outward tangents near the requested reach.

    The OSM junction is the estimated point of switch. Bearings are measured
    from a distal window, rather than the chord from the junction itself.
    """
    from collections import defaultdict
    adjacency = defaultdict(list)
    for way in ways:
        ids = way.get('nodes', [])
        for a, b in zip(ids, ids[1:]):
            if a in nodes and b in nodes and a != b:
                adjacency[a].append((b, way['id']))
                adjacency[b].append((a, way['id']))
    if node_id not in nodes:
        return []
    xy_cache = {}
    def xy(nid):
        if nid not in xy_cache:
            n = nodes[nid]
            xy_cache[nid] = transformer.transform(n['lon'], n['lat'])
        return xy_cache[nid]
    arms = []
    for neighbor, way_id in adjacency[node_id]:
        route = [node_id, neighbor]
        visited = {node_id, neighbor}
        length = distance(xy(node_id), xy(neighbor))
        previous, current = node_id, neighbor
        while length < reach:
            options = [(n, wid) for n, wid in adjacency[current]
                       if n != previous and n not in visited]
            # Stop at another switch or ambiguous branch rather than following
            # an arbitrary route across a second turnout.
            if len(options) != 1:
                break
            nxt, _ = options[0]
            length += distance(xy(current), xy(nxt))
            route.append(nxt)
            visited.add(nxt)
            previous, current = current, nxt
        coords = [xy(n) for n in route]
        cumulative = [0.0]
        for a, b in zip(coords, coords[1:]):
            cumulative.append(cumulative[-1] + distance(a, b))
        if cumulative[-1] < 1.0:
            continue
        end_dist = min(reach, cumulative[-1])
        start_dist = max(0.0, end_dist - tangent_window)
        def at(d):
            for j in range(1, len(cumulative)):
                if d <= cumulative[j] or j == len(cumulative)-1:
                    span = cumulative[j]-cumulative[j-1]
                    t = (d-cumulative[j-1])/span if span else 0.0
                    return (coords[j-1][0]+t*(coords[j][0]-coords[j-1][0]),
                            coords[j-1][1]+t*(coords[j][1]-coords[j-1][1]))
            return coords[-1]
        start_pt, end_pt = at(start_dist), at(end_dist)
        if distance(start_pt, end_pt) < 0.5:
            continue
        bearing = math.atan2(end_pt[1]-start_pt[1], end_pt[0]-start_pt[0])
        if all(abs(angle(bearing - other['bearing'])) > math.radians(1.5)
               for other in arms):
            arms.append({'bearing':bearing, 'way_id':way_id,
                         'end':list(end_pt), 'measured_reach':end_dist,
                         'tangent_window':end_dist-start_dist,
                         'node_path':route})
    return arms


def orientation(arms):
    if len(arms) < 3:
        return None
    pairs = sorted((abs(math.pi-abs(angle(a['bearing']-b['bearing']))),i,j)
                   for i,a in enumerate(arms) for j,b in enumerate(arms) if i<j)
    err,i,j = pairs[0]
    other = [k for k in range(len(arms)) if k not in (i,j)]
    k = min(other, key=lambda z: min(abs(angle(arms[z]['bearing']-arms[i]['bearing'])),
                                     abs(angle(arms[z]['bearing']-arms[j]['bearing']))))
    through = min((i,j), key=lambda z: abs(angle(arms[k]['bearing']-arms[z]['bearing'])))
    divergence = angle(arms[k]['bearing']-arms[through]['bearing'])
    return {'bearing_rad':arms[through]['bearing'],
            'branch_bearing_rad':arms[k]['bearing'],
            'branch_angle_deg':abs(math.degrees(divergence)),
            'hand':'LH' if divergence>0 else 'RH',
            'opposition_error_deg':math.degrees(err),
            'arm_count':len(arms),
            'angle_method':'distal_tangent_window_from_osm_split',
            'through_arm':arms[through], 'branch_arm':arms[k]}


def angle_annotation(placement, meters_per_crs_unit, length_m=18.0, offset_m=5.0):
    """Display distal fitted tangents at their actual measurement locations.

    The lines are not falsely anchored at the OSM switch node. Their infinite
    extensions may intersect near the PS; this intersection is reported only.
    """
    rays = []
    for key in ('through_arm', 'branch_arm'):
        arm = placement[key]
        bearing = arm['bearing']
        end = arm['end']
        window = arm['tangent_window']
        start = [end[0] - window * math.cos(bearing),
                 end[1] - window * math.sin(bearing)]
        # The exact line used to measure the angle, not a displaced ray.
        rays.append([start, list(end)])
    a = placement['bearing_rad']
    b = placement['branch_bearing_rad']
    mid = a + angle(b - a) / 2.0
    xy = placement['target_xy']
    offset = (length_m + offset_m) / meters_per_crs_unit
    label_xy = [xy[0] + offset * math.cos(mid),
                xy[1] + offset * math.sin(mid)]
    observed = placement['branch_angle_deg']
    estimated = (1.0 / (2.0 * math.tan(math.radians(observed) / 2.0))
                 if 0.05 < observed < 90 else None)
    nearest = round(estimated) if estimated is not None else None
    label = (f'No. {nearest} (est.) | {observed:.2f} deg {placement["hand"]}'
             if nearest is not None else f'{observed:.2f} deg {placement["hand"]}')
    return {'rays': rays, 'label_xy': label_xy, 'label': label,
            'estimated_frog_number': estimated, 'nearest_frog_number': nearest,
            'measurement_method': 'actual_distal_tangent_windows'}


def write_native_annotations(args, placements, meters_per_crs_unit):
    """Write labels first, then the measured tangent segments; log each insertion."""
    import traceback
    import MSPyBentleyGeom
    import MSPyDgnPlatform
    import MSPyMstnPlatform
    from MSPyMstnPlatform import ISessionMgr
    from MSPyDgnPlatform import EditElementHandle, LineHandler, TextBlock, TextElemHandler
    from MSPyBentleyGeom import DPoint3d, DSegment3d

    ref = ISessionMgr.GetActiveDgnModelRef()
    if ref is None:
        raise RuntimeError('No active DGN model reference')
    model = ref.GetDgnModel()
    if model is None:
        raise RuntimeError('No active DGN model')
    factor = float(model.GetModelInfo().GetUorPerMeter()) * meters_per_crs_unit
    if not math.isfinite(factor) or factor <= 0:
        raise RuntimeError('Invalid destination coordinate scale')
    print('Annotation destination:', model.GetModelName(), 'EPSG:', args.epsg, flush=True)
    print('Verify destination DGN GCS matches --epsg; no automatic GCS change.', flush=True)

    def point(xy):
        return DPoint3d(float(xy[0])*factor, float(xy[1])*factor, 0.0)

    def inserted_id(handle):
        try:
            return str(handle.GetElementId())
        except (AttributeError, TypeError) as exc:
            return 'unavailable (' + str(exc) + ')'

    def add_line(a, b, kind):
        handle = EditElementHandle()
        status = LineHandler.CreateLineElement(
            handle, None, DSegment3d(point(a), point(b)), ref.Is3d(), ref)
        if not success(status):
            raise RuntimeError(kind + ' CreateLineElement failed: ' + str(status))
        added = handle.AddToModel()
        if added is not None and not success(added):
            raise RuntimeError(kind + ' AddToModel failed: ' + str(added))
        print(kind, 'AddToModel=', added, 'element_id=', inserted_id(handle),
              'start=', a, 'end=', b, flush=True)
        return inserted_id(handle)

    def add_text(label, xy):
        # TextBlock's constructor requires a DgnModel, not a DgnModelRef.
        block = TextBlock(model)
        block.SetUserOrigin(point(xy))
        # Set text size explicitly if the installed binding exposes these
        # Bentley text-run APIs. Otherwise report the inherited style.
        text_height_uor = float(args.text_height) * float(model.GetModelInfo().GetUorPerMeter())
        try:
            from MSPyBentleyGeom import DPoint2d
            props = block.GetRunPropertiesForAdd()
            props.SetFontSize(DPoint2d(text_height_uor, text_height_uor))
            print('Text height set to', args.text_height, 'meters', flush=True)
        except (AttributeError, TypeError) as exc:
            print('WARNING: explicit text height not supported by this binding:',
                  repr(exc), '; using active text style', flush=True)
        block.AppendText(label)
        handle = EditElementHandle()
        creation = TextElemHandler.CreateElement(handle, None, block)
        print('TextElemHandler.CreateElement returned:', repr(creation), flush=True)
        if creation is None:
            raise RuntimeError('TextElemHandler.CreateElement returned None')
        # The return type is TextBlockToElementResult, NOT BentleyStatus.
        # Avoid guessing its enum values; AddToModel is checked separately.
        added = handle.AddToModel()
        if added is not None and not success(added):
            raise RuntimeError('Text AddToModel failed: ' + str(added))
        eid = inserted_id(handle)
        print('TEXT AddToModel=', added, 'element_id=', eid,
              'location=', xy, 'label=', label, flush=True)
        return eid

    completed = 0
    for placement in placements:
        ann = placement.get('angle_annotation')
        if ann is None:
            continue
        stage = 'text'
        node_id = placement['node_id']
        try:
            print('Node', node_id, 'creating label first:', ann['label'], flush=True)
            text_id = add_text(ann['label'], ann['label_xy'])
            line_ids = []
            for i, (start, end) in enumerate(ann['rays'], 1):
                stage = 'tangent line ' + str(i)
                line_ids.append(add_line(start, end, stage))
            placement['annotation_element_ids'] = {'text': text_id, 'lines': line_ids}
            completed += 1
            print('Annotation complete:', completed, 'node:', node_id, flush=True)
        except Exception as exc:
            print('ANNOTATION FAILED node=', node_id, 'stage=', stage,
                  'error=', type(exc).__name__, str(exc), flush=True)
            traceback.print_exc()
            raise RuntimeError(
                'Annotation failed at node %s during %s: %s: %s. '
                'Some elements may have been inserted; remove them before rerunning.'
                % (node_id, stage, type(exc).__name__, exc)) from exc
    print('Native angle annotations completed:', completed, flush=True)

def geometry_iter(obj):
    if isinstance(obj,dict):
        g=obj.get('geometry')
        if isinstance(g,list):
            for item in g: yield from geometry_iter({'geometry':item})
        elif isinstance(g,dict) and 'kind' in g:yield g
        for child in obj.get('children',[]):yield from geometry_iter(child)

def geometry_stats(model):
    lines=[];arcs=[];samples=[]
    for el in model.get('elements',[]):
        for g in geometry_iter(el):
            kind=g.get('kind')
            if kind=='line' and len(g.get('points',[]))>=2:
                pts=g['points']; lines.append(pts)
                samples.extend(pts)
            elif kind in ('arc','ellipse'):
                arcs.append(g)
                samples.extend(g.get('sampled_xyz',[]))
    return lines,arcs,samples

def _cross2(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _intersection(a, b, c, d):
    """Intersection of infinite AB and CD, including segment parameters."""
    u = (b[0]-a[0], b[1]-a[1])
    v = (d[0]-c[0], d[1]-c[1])
    den = _cross2(u, v)
    if abs(den) < 1e-10 * max(1.0, math.hypot(*u)*math.hypot(*v)):
        return None
    delta = (c[0]-a[0], c[1]-a[1])
    t = _cross2(delta, v) / den
    w = _cross2(delta, u) / den
    return (a[0]+t*u[0], a[1]+t*u[1], t, w)


def estimate_anchor(model):
    """Anchor at the symbol's perpendicular stem/through-axis intersection.

    Circle: stem bisects the circle. Triangle: stem is the vertical side.
    Work in raw extracted cell coordinates; no assumed real-world scale.
    """
    def descendants(element):
        yield element
        for child in element.get('children', []):
            yield from descendants(child)

    segments, circles, triangles = [], [], []
    for root in model.get('elements', []):
        for el in descendants(root):
            geometries = el.get('geometry', [])
            if isinstance(geometries, dict):
                geometries = [geometries]
            for g in geometries:
                kind = g.get('kind')
                pts = g.get('points', [])
                if kind in ('line', 'line_string') and len(pts) >= 2:
                    for a, b in zip(pts, pts[1:]):
                        if distance(a, b) > 1e-8:
                            segments.append({'a': a[:2], 'b': b[:2],
                                             'id': el.get('id'), 'kind': kind,
                                             'length': distance(a, b)})
                    if kind == 'line_string' and len(pts) >= 4 and distance(pts[0], pts[-1]) < 1e-5:
                        triangles.append({'points': [q[:2] for q in pts], 'id': el.get('id')})
                elif kind in ('ellipse', 'arc') and g.get('center') is not None:
                    rx = g.get('primary_radius', g.get('radius'))
                    ry = g.get('secondary_radius', rx)
                    if rx is not None and ry is not None and abs(float(rx)-float(ry)) < max(1e-6, float(rx)*0.03):
                        circles.append({'center': g['center'][:2], 'radius': float(rx),
                                        'id': el.get('id')})

    if not segments:
        return None
    # Reference axis: the long, approximately horizontal baseline through y=0.
    # For this cell library, that axis is also supported by numerous endpoints.
    all_x = [v[0] for seg in segments for v in (seg['a'], seg['b'])]
    span = max(all_x)-min(all_x)
    tol = max(1e-6, span*1e-5)
    baseline = [seg for seg in segments
                if abs(seg['a'][1]-seg['b'][1]) <= tol and seg['length'] >= span*0.1]
    zero_support = sum(abs(v[1]) <= tol for seg in segments for v in (seg['a'], seg['b']))
    if zero_support >= 3:
        axis_y = 0.0
    elif baseline:
        axis_y = min(((seg['a'][1]+seg['b'][1])*0.5 for seg in baseline), key=abs)
    else:
        # Use dominant endpoint elevation, preferring the cell reference y=0.
        ys = [v[1] for seg in segments for v in (seg['a'], seg['b'])]
        axis_y = min(ys, key=lambda y: (sum(abs(y-z) > tol for z in ys), abs(y)))

    candidates = []
    for stem in segments:
        a, b = stem['a'], stem['b']
        if abs(a[0]-b[0]) > max(tol, stem['length']*0.035):
            continue
        if abs(a[1]-b[1]) < tol:
            continue
        x = (a[0]+b[0])*0.5
        # Must touch the axis, or end very close to it.
        gap = max(0.0, min(a[1],b[1])-axis_y, axis_y-max(a[1],b[1]))
        if gap > max(tol*2, stem['length']*0.08):
            continue
        for circle in circles:
            cx, cy = circle['center']
            if abs(cx-x) <= max(tol, circle['radius']*0.04) and abs(cy-axis_y) > tol:
                if min(a[1],b[1])-tol <= cy <= max(a[1],b[1])+tol:
                    candidates.append((100.0-gap/max(stem['length'],1), x, stem,
                                       'circle_bisector', circle['id']))
        for tri in triangles:
            vertices = tri['points']
            for u,v in zip(vertices,vertices[1:]):
                if abs(u[0]-v[0]) <= max(tol,stem['length']*0.035) and abs(u[0]-x) <= max(tol,stem['length']*0.035):
                    if abs(u[1]-v[1]) >= stem['length']*0.8:
                        candidates.append((90.0-gap/max(stem['length'],1),x,stem,
                                           'triangle_vertical_side',tri['id']))
                        break
    if candidates:
        # Prefer the approach-end marker (leftmost in the raw library drawings).
        candidates.sort(key=lambda c: (-c[0], c[1]))
        score,x,stem,kind,marker_id=candidates[0]
        return {'toe_xy_raw':[float(x),float(axis_y)],
                'through_bearing_deg':0.0,
                'method':'symbol_perpendicular_intersection',
                'marker_kind':kind, 'confidence':'estimated',
                'leader_element_id':stem['id'], 'marker_element_id':marker_id,
                'through_axis_y_raw':float(axis_y),
                'candidate_count':len(candidates)}

    longest=max(segments,key=lambda seg:seg['length'])
    return {'toe_xy_raw':[float(longest['a'][0]),float(longest['a'][1])],
            'through_bearing_deg':math.degrees(math.atan2(
                longest['b'][1]-longest['a'][1],longest['b'][0]-longest['a'][0])),
            'method':'longest_line_endpoint_FALLBACK_UNVERIFIED',
            'confidence':'low','candidate_count':0}


def make_preview_dxf(path, placements, models, scales, meters_per_crs_unit):
    try:import ezdxf
    except ImportError as exc:raise RuntimeError('DXF preview needs: python -m pip install ezdxf') from exc
    doc=ezdxf.new('R2013');msp=doc.modelspace()
    doc.layers.new('OSM_SWITCH',dxfattribs={'color':1})
    doc.layers.new('CELL_LINES',dxfattribs={'color':3})
    doc.layers.new('CELL_CURVES_PREVIEW',dxfattribs={'color':5})
    for p in placements:
        if p['status']!='preview_ready':continue
        origin=p['target_xy'];rot=p['rotation_rad'];anchor=p['anchor_raw']
        scale=scales[p['model']]['meters_per_raw'] / meters_per_crs_unit
        msp.add_circle(origin,1.0 / meters_per_crs_unit,dxfattribs={'layer':'OSM_SWITCH'})
        model=models[p['model']]
        lines,arcs,_=geometry_stats(model)
        for pts in lines:
            xy=[transform_point(q,scale,rot,origin,anchor) for q in pts]
            if len(xy)==2:msp.add_line(*xy,dxfattribs={'layer':'CELL_LINES'})
            elif len(xy)>2:msp.add_lwpolyline(xy,dxfattribs={'layer':'CELL_LINES'})
        for g in arcs:
            # Preserve source geometry as a *preview* polyline only; this is not a cell insertion.
            pts=g.get('sampled_xyz',[])
            if len(pts)>1:
                xy=[transform_point(q,scale,rot,origin,anchor) for q in pts]
                msp.add_lwpolyline(xy,dxfattribs={'layer':'CELL_CURVES_PREVIEW'})
    path.parent.mkdir(parents=True,exist_ok=True);doc.saveas(path)

def run(args):
    from pyproj import CRS, Transformer
    target_crs = CRS.from_epsg(args.epsg)
    if not target_crs.is_projected:
        raise ValueError('--epsg must specify a projected CRS with linear units')
    axes = target_crs.axis_info[:2]
    if len(axes) != 2 or not all(a.unit_conversion_factor for a in axes):
        raise ValueError('Cannot determine target CRS linear units')
    meters_per_crs_unit = float(axes[0].unit_conversion_factor)
    if not math.isclose(meters_per_crs_unit, float(axes[1].unit_conversion_factor), rel_tol=1e-10):
        raise ValueError('Target CRS axes use different linear units')
    if not math.isfinite(meters_per_crs_unit) or meters_per_crs_unit <= 0:
        raise ValueError('Invalid target CRS linear-unit conversion')
    print(f'Target EPSG:{args.epsg}; meters per CRS unit: {meters_per_crs_unit}', flush=True)
    osm=json.loads(args.osm.read_text(encoding='utf-8'))
    if 'elements' not in osm:raise ValueError('Expected raw Overpass JSON with elements')
    nodes={int(e['id']):e for e in osm['elements'] if e.get('type')=='node' and 'lon' in e and 'lat' in e}
    ways=[e for e in osm['elements'] if e.get('type')=='way' and e.get('tags',{}).get('railway') in
          ('rail','light_rail','industrial','disused','narrow_gauge')]
    lib=json.loads(args.geometry.read_text(encoding='utf-8'))
    models={m['model']:m for m in lib['models']}
    with args.rankings.open(newline='',encoding='utf-8-sig') as f:rankings=list(csv.DictReader(f))
    if args.node is not None:rankings=[r for r in rankings if int(r['osm_node_id'])==args.node]
    if args.max_turnouts is not None:rankings=rankings[:args.max_turnouts]
    if args.anchors is None:
        overrides = {name: estimate_anchor(model) for name, model in models.items()}
        overrides = {name: anchor for name, anchor in overrides.items() if anchor}
        generated = HERE / 'anchors_candidates.json'
        generated.write_text(json.dumps({'verified': False, 'models': overrides}, indent=2), encoding='utf-8')
        print('Automatically estimated anchors:', generated, flush=True)
    else:
        overrides = json.loads(args.anchors.read_text(encoding='utf-8')).get('models', {})
    proj=Transformer.from_crs('EPSG:4326',target_crs,always_xy=True)
    placements=[]
    for i,r in enumerate(rankings,1):
        nid=int(r['osm_node_id']);node=nodes.get(nid);name=args.model or r.get('best_model','')
        p={'node_id':nid,'model':name,'status':'unresolved','ranking_confidence':r.get('confidence',''),
           'ranking_rmse_m':r.get('best_rmse_m','')}
        if node is None:p['status']='missing_osm_node'
        elif name not in models:p['status']='missing_cell_model'
        else:
            xy=proj.transform(node['lon'],node['lat']);p['target_xy']=list(xy)
            orient=orientation(osm_arms(nid,nodes,ways,proj,
                args.arm_reach / meters_per_crs_unit,
                args.tangent_window / meters_per_crs_unit))
            if orient is None:p['status']='insufficient_osm_arms'
            else:
                p.update(orient)
                p['estimated_ps_xy'] = list(xy)
                p['angle_annotation'] = angle_annotation(p, meters_per_crs_unit, args.angle_line_length, args.label_offset)
                p['observed_angle_label'] = (
                    f"SW {nid} | {orient['branch_angle_deg']:.2f} deg {orient['hand']}")
                cell_hand='LH' if 'LH' in name.split() else ('RH' if 'RH' in name.split() else None)
                if cell_hand and cell_hand!=orient['hand']:p['status']='hand_mismatch'
                else:
                    anchor=overrides.get(name)
                    if anchor is None:anchor=estimate_anchor(models[name])
                    if anchor is None:p['status']='needs_cell_anchor'
                    else:
                        raw=anchor.get('toe_xy_raw')
                        if raw is None:p['status']='anchor_missing_toe_xy_raw'
                        else:
                            rotation=angle(orient['bearing_rad']-math.radians(float(anchor['through_bearing_deg'])))
                            p.update({'anchor_raw':raw,'anchor_method':anchor.get('method','manual'),
                                       'anchor_confidence':anchor.get('confidence','unknown'),
                                      'rotation_rad':rotation,'rotation_deg':math.degrees(rotation),
                                      'raw_coordinate_scale_m':None,
                                      'status':'preview_ready'})
        placements.append(p)
        print(f'[{i}/{len(rankings)}] OSM {nid}: {name}: {p["status"]}',flush=True)
    report={'schema':'turnout_placement_v2','epsg':args.epsg,'meters_per_crs_unit':meters_per_crs_unit,'source_osm':str(args.osm),
            'source_geometry':str(args.geometry),'angle_note':'OSM split approximates PS; angle measured between distal branch and through tangents, not the frog angle.', 'scale_note':'Scale resolved from source cell model at native placement time; source raw geometry assumed UOR.',
            'placements':placements}
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('Saved plan:',args.out,flush=True)
    if args.preview_dxf:
        if args.cel is None:raise RuntimeError('--preview-dxf requires --cel to read source model units')
        scales = resolve_scales(args.cel, {p['model'] for p in placements if p['status']=='preview_ready'}, args.raw_coordinate_kind)
        make_preview_dxf(args.preview_dxf,placements,models,scales,meters_per_crs_unit)
        print('Saved DXF preview:',args.preview_dxf,flush=True)
    if args.annotate:
        write_native_annotations(args, placements, meters_per_crs_unit)
    if args.write:
        native_place(args, placements, meters_per_crs_unit)
    return report


def success(status):
    if status is None:
        return False
    if str(status) in ('0', 'BentleyStatus.eSUCCESS', 'eSUCCESS'):
        return True
    try:
        return int(status) == 0
    except (TypeError, ValueError):
        return False


def rotation_matrix(theta):
    """Bentley RotMatrix from nine row-major entries (confirm binding in runtime)."""
    from MSPyBentleyGeom import RotMatrix
    c, s = math.cos(theta), math.sin(theta)
    return RotMatrix.FromRowValues(c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0)


def _model_scale(model, raw_kind):
    """Convert source-model UOR or storage coordinates to meters.

    Bentley ModelInfo.GetUorPerMeter() is documented and already incorporates
    the source model's storage unit definition. No hard-coded unit assumptions.
    """
    info = model.GetModelInfo()
    uor_per_meter = float(info.GetUorPerMeter())
    uor_per_storage = float(info.GetUorPerStorage())
    if not all(math.isfinite(v) and v > 0 for v in (uor_per_meter, uor_per_storage)):
        raise RuntimeError('Invalid UOR resolution for model ' + model.GetModelName())
    meters_per_storage = uor_per_storage / uor_per_meter
    meters_per_raw = 1.0 / uor_per_meter if raw_kind == 'uor' else meters_per_storage
    unit = info.GetStorageUnit()
    if not unit.IsValid():
        raise RuntimeError('Invalid storage unit for model ' + model.GetModelName())
    return {
        'uor_per_meter': uor_per_meter,
        'uor_per_storage': uor_per_storage,
        'meters_per_storage': meters_per_storage,
        'meters_per_raw': meters_per_raw,
        'storage_unit_label': str("placeholder"),
        'raw_kind': raw_kind,
        'unit_method': 'ModelInfo.GetUorPerMeter',
    }


def _source_model_for_cell(library, name):
    model_id = library.FindModelIdByName(name)
    model = library.LoadModelById(model_id)
    if model is None:
        raise RuntimeError('Could not load source cell model ' + name)
    return model


def _attach_and_find_library(path):
    # Load type registrations before using session/model reference APIs.
    import MSPyBentleyGeom
    import MSPyDgnPlatform
    import MSPyMstnPlatform
    from MSPyBentley import BeFileName
    from MSPyMstnPlatform import Cell
    output_name = BeFileName()
    input_name = BeFileName(str(path.resolve()))
    status = Cell.AttachLibrary(output_name, input_name, '', 1)
    if not success(status):
        raise RuntimeError('Cell.AttachLibrary failed: ' + str(status))
    return Cell


def _library_for_cell(Cell, name):
    """Use documented Cell.FindCell tuple to obtain the actual source DgnFile."""
    result = Cell.FindCell(None, name, 0)
    if not isinstance(result, tuple):
        raise RuntimeError('Unexpected Cell.FindCell result for ' + name + ': ' + repr(result))
    libraries = [v for v in result if hasattr(v, 'FindModelIdByName') and hasattr(v, 'LoadModelById')]
    if len(libraries) != 1:
        raise RuntimeError('Cannot resolve source DgnFile for ' + name + ': ' + repr(result))
    return libraries[0]


def resolve_scales(cel, names, raw_kind):
    if not cel.is_file():
        raise RuntimeError('Cell library does not exist: ' + str(cel))
    Cell = _attach_and_find_library(cel)
    scales = {}
    for name in sorted(names):
        if not Cell.ExistsInLibrary(name):
            raise RuntimeError('Cell not found in library: ' + name)
        library = _library_for_cell(Cell, name)
        model = _source_model_for_cell(library, name)
        scales[name] = _model_scale(model, raw_kind)
        print('SOURCE UNITS:', name, scales[name], flush=True)
    return scales


def native_place(args, placements, meters_per_crs_unit):
    """Place native cells using per-cell source units and Bentley trueScale."""
    if args.cel is None or not args.cel.is_file():
        raise RuntimeError('--write requires an existing --cel library')
    if args.local_origin is not None:
        raise RuntimeError('Local origin requires matching DGN global-origin configuration')
    ready = [p for p in placements if p['status'] == 'preview_ready' and
             (p.get('anchor_method') in ('leader_throughline_intersection', 'symbol_perpendicular_intersection') or
              args.anchors is not None or args.allow_fallback_anchor)]
    if not ready:
        raise RuntimeError('No placeable switches; see turnout_placement_plan.json')
    import MSPyBentleyGeom
    import MSPyDgnPlatform
    import MSPyMstnPlatform
    from MSPyBentleyGeom import DPoint3d
    from MSPyMstnPlatform import ISessionMgr
    ref = ISessionMgr.GetActiveDgnModelRef()
    if ref is None or ref.GetDgnModel() is None:
        raise RuntimeError('Open a writable DGN design model in MicroStation Python Manager')
    model = ref.GetDgnModel()
    dest_info = model.GetModelInfo()
    dest_uor = float(dest_info.GetUorPerMeter())
    # Convert projected CRS coordinates to meters before applying destination UOR/meter.
    dest_unit = _model_scale(model, 'storage')
    dest_meters_per_storage = dest_unit['meters_per_storage']
    print('DESTINATION:', model.GetModelName(), dest_unit, flush=True)
    print('Destination meters per storage unit:', dest_meters_per_storage, flush=True)
    print(f'WARNING: Verify destination GCS EPSG:{args.epsg} and global origin; this script does not change either.', flush=True)
    scales = resolve_scales(args.cel, {p['model'] for p in ready}, args.raw_coordinate_kind)
    print('WARNING: leader/through-line anchors are geometry estimates; review before production use.', flush=True)
    print('Eligible cell placements:', len(ready), flush=True)
    from MSPyMstnPlatform import Cell
    for index, p in enumerate(ready, 1):
        name = p['model']
        scale = scales[name]['meters_per_raw']
        toe = p['anchor_raw']
        theta = p['rotation_rad']
        c, sn = math.cos(theta), math.sin(theta)
        ax, ay = float(toe[0]) * scale, float(toe[1]) * scale
        x, y = p['target_xy']
        ox = x * meters_per_crs_unit - (c * ax - sn * ay)
        oy = y * meters_per_crs_unit - (sn * ax + c * ay)
        # trueScale handles source/destination unit conversion. Offset is converted
        # from extracted source units once, into destination meters.
        origin = DPoint3d(ox * dest_uor, oy * dest_uor, 0.0)
        rot = rotation_matrix(theta)
        print(f'[{index}/{len(ready)}] placing {name} at ({ox:.3f}, {oy:.3f})', flush=True)
        position = Cell.PlaceCell(origin, DPoint3d(1.0, 1.0, 1.0), True,
                                  rot, 0, 0, False, 0, 0, name, None)
        if int(position) == 0:
            raise RuntimeError(f'Cell.PlaceCell failed for {name}, node {p["node_id"]}; previous cells may be inserted')
        p.update(status='native_cell_inserted', file_position=int(position),
                 source_scale=scales[name], insertion_origin_m=[ox, oy])
        args.out.write_text(json.dumps({'schema':'turnout_placement_v4',
              'epsg':args.epsg, 'meters_per_crs_unit':meters_per_crs_unit, 'placements':placements}, indent=2), encoding='utf-8')
        print('Placed node:', p['node_id'], 'file position:', position, flush=True)
    print('Finished. Check cell origin, geometry, GCS and anchors; track centerlines are not trimmed.', flush=True)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--epsg',type=int,default=6424,help='Target projected CRS EPSG code (default: 6424)')
    p.add_argument('--osm',type=Path,default=HERE/'railway_raw_osm.json')
    p.add_argument('--rankings',type=Path,default=HERE/'turnout_rankings.csv')
    p.add_argument('--geometry',type=Path,default=HERE/'turnout_geometry_v14.json')
    p.add_argument('--anchors',type=Path,help='Optional anchor overrides; otherwise estimated automatically')
    p.add_argument('--cel',type=Path,help='Original .cel library for native insertion')
    p.add_argument('--node',type=int)
    p.add_argument('--model',help='Override ranked model')
    p.add_argument('--max-turnouts',type=int,default=1)
    p.add_argument('--arm-reach',type=float,default=25.0,help='Distance from estimated PS to tangent measurement (meters)')
    p.add_argument('--tangent-window',type=float,default=12.0,help='Length of distal tangent fitting window (meters)')
    p.add_argument('--raw-coordinate-kind', choices=('uor','storage'), default='uor',
                   help='Interpret extracted JSON coordinates as source UOR (default) or source storage units')
    p.add_argument('--out',type=Path,default=HERE/'turnout_placement_plan.json')
    p.add_argument('--preview-dxf',type=Path,help='Create transformed cell-geometry preview DXF')
    p.add_argument('--annotate',action='store_true',help='Write two measured tangents and a native text label')
    p.add_argument('--text-height',type=float,default=2.0,help='Text height in meters (default: 2)')
    p.add_argument('--angle-line-length',type=float,default=18.0,help='Length of displayed angle rays in meters')
    p.add_argument('--label-offset',type=float,default=5.0,help='Additional label offset beyond rays in meters')
    p.add_argument('--allow-fallback-anchor',action='store_true',
                   help='Allow native placement with unverified longest-line fallback anchors')
    p.add_argument('--write',action='store_true',help='Insert native cells into current MicroStation DGN')
    p.add_argument('--local-origin',type=float,nargs=2,help='Diagnostic only; native placement blocked')
    a=p.parse_args()
    # All generated outputs stay beside this script, even with custom filenames.
    a.out = HERE / a.out.name
    if a.preview_dxf is not None:
        a.preview_dxf = HERE / a.preview_dxf.name
    if a.max_turnouts is not None and a.max_turnouts<1:p.error('--max-turnouts must be >=1')
    if a.angle_line_length <= 0 or a.label_offset < 0:p.error('annotation length must be positive and label offset nonnegative')
    if a.arm_reach <= 0 or a.tangent_window <= 0:p.error('arm reach and tangent window must be positive')
    if a.text_height <= 0:p.error('--text-height must be positive')
    run(a)

if __name__=='__main__':main()
