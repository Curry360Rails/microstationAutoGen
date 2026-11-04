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

def osm_arms(node_id, nodes, ways, transformer, reach):
    arms=[]
    for way in ways:
        ids=way.get('nodes',[])
        for idx, nid in enumerate(ids):
            if nid!=node_id: continue
            for direction in (-1,1):
                j=idx+direction
                if j<0 or j>=len(ids):continue
                start=nodes[node_id]
                origin=transformer.transform(start['lon'],start['lat'])
                last=None
                while 0<=j<len(ids) and ids[j] in nodes:
                    node=nodes[ids[j]]
                    point=transformer.transform(node['lon'],node['lat'])
                    if distance(origin,point)>=0.5: last=point
                    if distance(origin,point)>=reach:break
                    j+=direction
                if last is not None:
                    a=math.atan2(last[1]-origin[1],last[0]-origin[0])
                    if all(abs(angle(a-other['bearing']))>math.radians(1.5) for other in arms):
                        arms.append({'bearing':a,'way_id':way['id'],'end':last})
    return arms

def orientation(arms):
    if len(arms)<3:return None
    pairs=sorted((abs(math.pi-abs(angle(a['bearing']-b['bearing']))),i,j)
                 for i,a in enumerate(arms) for j,b in enumerate(arms) if i<j)
    err,i,j=pairs[0]
    other=[k for k in range(len(arms)) if k not in (i,j)]
    k=min(other,key=lambda z:min(abs(angle(arms[z]['bearing']-arms[i]['bearing'])),abs(angle(arms[z]['bearing']-arms[j]['bearing']))))
    through=min((i,j),key=lambda z:abs(angle(arms[k]['bearing']-arms[z]['bearing'])))
    divergence=angle(arms[k]['bearing']-arms[through]['bearing'])
    return {'bearing_rad':arms[through]['bearing'], 'branch_angle_deg':abs(math.degrees(divergence)),
            'hand':'LH' if divergence>0 else 'RH','opposition_error_deg':math.degrees(err),
            'arm_count':len(arms)}

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

def estimate_anchor(model):
    """Heuristic only: midpoint of longest extracted straight line, NOT verified PS."""
    lines,_,_=geometry_stats(model)
    segments=[]
    for pts in lines:
        for a,b in zip(pts,pts[1:]):
            length=distance(a,b)
            if length>0:segments.append((length,a,b))
    if not segments:return None
    _,a,b=max(segments,key=lambda x:x[0])
    return {'toe_xy_raw':[float(a[0]),float(a[1])],
            'through_bearing_deg':math.degrees(math.atan2(b[1]-a[1],b[0]-a[0])),
            'method':'longest_line_endpoint_HEURISTIC_NOT_VERIFIED'}

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
            orient=orientation(osm_arms(nid,nodes,ways,proj,args.arm_reach / meters_per_crs_unit))
            if orient is None:p['status']='insufficient_osm_arms'
            else:
                p.update(orient)
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
                                      'rotation_rad':rotation,'rotation_deg':math.degrees(rotation),
                                      'raw_coordinate_scale_m':None,
                                      'status':'preview_ready'})
        placements.append(p)
        print(f'[{i}/{len(rankings)}] OSM {nid}: {name}: {p["status"]}',flush=True)
    report={'schema':'turnout_placement_v2','epsg':args.epsg,'meters_per_crs_unit':meters_per_crs_unit,'source_osm':str(args.osm),
            'source_geometry':str(args.geometry),'scale_note':'Scale resolved from source cell model at native placement time; source raw geometry assumed UOR.',
            'placements':placements}
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('Saved plan:',args.out,flush=True)
    if args.preview_dxf:
        if args.cel is None:raise RuntimeError('--preview-dxf requires --cel to read source model units')
        scales = resolve_scales(args.cel, {p['model'] for p in placements if p['status']=='preview_ready'}, args.raw_coordinate_kind)
        make_preview_dxf(args.preview_dxf,placements,models,scales,meters_per_crs_unit)
        print('Saved DXF preview:',args.preview_dxf,flush=True)
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
    ready = [p for p in placements if p['status'] == 'preview_ready']
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
    print('WARNING: automatic anchors are geometric estimates, not verified switch points.', flush=True)
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
    p.add_argument('--arm-reach',type=float,default=25.0)
    p.add_argument('--raw-coordinate-kind', choices=('uor','storage'), default='uor',
                   help='Interpret extracted JSON coordinates as source UOR (default) or source storage units')
    p.add_argument('--out',type=Path,default=HERE/'turnout_placement_plan.json')
    p.add_argument('--preview-dxf',type=Path,help='Create transformed cell-geometry preview DXF')
    p.add_argument('--write',action='store_true',help='Insert native cells into current MicroStation DGN')
    p.add_argument('--local-origin',type=float,nargs=2,help='Diagnostic only; native placement blocked')
    a=p.parse_args()
    # All generated outputs stay beside this script, even with custom filenames.
    a.out = HERE / a.out.name
    if a.preview_dxf is not None:
        a.preview_dxf = HERE / a.preview_dxf.name
    if a.max_turnouts is not None and a.max_turnouts<1:p.error('--max-turnouts must be >=1')
    run(a)

if __name__=='__main__':main()
