"""MicroStation 2025 read-only compact turnout cell geometry exporter v14.
Run with TURNOUT .cel open, inside MicroStation Python Manager.
"""
import csv
import json
import math
import traceback
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent
TARGET = '10 LHIIUP'
MAX_MODELS = 250
MAX_DEPTH = 30
MAX_CHILDREN = 5000
SAMPLE_INTERVALS = 16
# Do not convert raw geometry to meters until its coordinate basis is verified.
OUTPUT = HERE / 'turnout_geometry_v14.json'
SUMMARY = HERE / 'turnout_geometry_v14_summary.csv'
LOG = HERE / 'turnout_geometry_v14.log'


def point(p):
    return [float(p.x), float(p.y), float(getattr(p, 'z', 0.0))]


def finite(values):
    return all(math.isfinite(v) for v in values)


def arc_point(center, a, b, rotation, theta):
    u = rotation + theta
    return [center[0] + a * math.cos(rotation) * math.cos(theta) - b * math.sin(rotation) * math.sin(theta),
            center[1] + a * math.sin(rotation) * math.cos(theta) + b * math.cos(rotation) * math.sin(theta),
            center[2]]


def native_curve(el, kind, dimensionality):
    # Union members other than the matching element type/dimensionality are invalid.
    member = ('arc_' if kind == 'arc' else 'ellipse_') + ('3d' if dimensionality else '2d')
    raw = getattr(el, member)
    if dimensionality:
        # 3D quaternion orientation needs explicit verification; do not fabricate axes.
        return {'kind': kind, 'native_representation': member,
                'status': '3d_orientation_not_decoded',
                'origin': point(raw.origin), 'primary': float(raw.primary),
                'secondary': float(raw.secondary),
                'quaternion': [float(v) for v in raw.quat],
                **({'start_angle_rad': float(raw.startAngle),
                    'sweep_angle_rad': float(raw.sweepAngle)} if kind == 'arc' else {})}
    center = point(raw.origin)
    a, b, rot = float(raw.primary), float(raw.secondary), float(raw.rotationAngle)
    if not finite(center + [a, b, rot]) or min(a, b) <= 0:
        raise ValueError('Nonfinite or nonpositive ellipse geometry')
    result = {'kind': kind, 'native_representation': member,
              'status': 'native_2d_decoded', 'center': center,
              'primary_radius': a, 'secondary_radius': b,
              'rotation_rad': rot, 'rotation_deg': math.degrees(rot)}
    if kind == 'arc':
        start, sweep = float(raw.startAngle), float(raw.sweepAngle)
        if not finite([start, sweep]):
            raise ValueError('Invalid arc angles')
        result.update(start_angle_rad=start, sweep_angle_rad=sweep,
                      start_angle_deg=math.degrees(start), sweep_angle_deg=math.degrees(sweep),
                      start=arc_point(center, a, b, rot, start),
                      end=arc_point(center, a, b, rot, start + sweep))
        result['sampled_xyz'] = [arc_point(center, a, b, rot, start + sweep * i / SAMPLE_INTERVALS)
                                 for i in range(SAMPLE_INTERVALS + 1)]
    else:
        result['sampled_xyz'] = [arc_point(center, a, b, rot, 2 * math.pi * i / (SAMPLE_INTERVALS * 4))
                                 for i in range(SAMPLE_INTERVALS * 4 + 1)]
    return result


def primitive_geometry(eh, dgn):
    et = int(eh.GetElementType())
    if et not in (3, 4, 6):
        return None
    query = getattr(dgn, 'ICurvePathQuery', None)
    if query is None:
        raise RuntimeError('ICurvePathQuery unavailable')
    curves = query.ElementToCurveVector(eh)
    if curves is None:
        return None
    parts = []
    for p in curves:
        # Use GetLine / GetLineString with validated results, no guessing.
        try:
            seg = p.GetLine()
            if seg is not None:
                coords = [point(x) for x in seg.point]
                if len(coords) == 2 and all(finite(q) for q in coords):
                    parts.append({'kind': 'line', 'points': coords})
                    continue
        except Exception:
            pass
        try:
            pts = p.GetLineString()
            if pts is not None:
                coords = [point(x) for x in pts]
                if len(coords) >= 2 and all(finite(q) for q in coords):
                    parts.append({'kind': 'line_string', 'points': coords})
        except Exception:
            pass
    return parts or None


def is_3d(model):
    v = getattr(model, 'Is3d', None)
    if v is None:
        v = getattr(model, 'Is3D', None)
    if v is None:
        return None
    return bool(v() if callable(v) else v)


def unit_info(model, dgn):
    info = model.GetModelInfo()
    master = info.GetMasterUnit()
    out = {'uor_per_storage': float(info.GetUorPerStorage()),
           'master_unit_numerator': float(master.GetNumerator()),
           'master_unit_denominator': float(master.GetDenominator()),
           'coordinate_basis': 'raw DGN coordinates; UOR vs master-unit scale unverified',
           'meters_per_raw_coordinate': None}
    try:
        meter = dgn.UnitDefinition.GetStandardUnitByName('Meter')
        status, factor = meter.GetConversionFactorFrom(master)
        out['master_to_meter_status'] = str(status)
        if int(status) == 0 and math.isfinite(float(factor)) and float(factor) > 0:
            out['meters_per_master_unit'] = float(factor)
    except Exception as ex:
        out['conversion_error'] = repr(ex)
    return out


def child_items(eh, dgn):
    it = dgn.ChildElemIter(eh)
    result = []
    for _ in range(MAX_CHILDREN):
        if not it.IsValid():
            return result
        result.append(it.GetElement())
        nxt = it.ToNext()
        if nxt is not None and hasattr(nxt, 'IsValid'):
            it = nxt
    raise RuntimeError('MAX_CHILDREN exceeded')


def extract(ref, model, dgn, dimension, depth=0, seen=None):
    if seen is None:
        seen = set()
    if depth > MAX_DEPTH:
        return {'error': 'MAX_DEPTH exceeded'}
    eh = ref if isinstance(ref, dgn.ElementHandle) else dgn.ElementHandle(ref, model)
    eid, et = int(eh.GetElementId()), int(eh.GetElementType())
    row = {'id': eid, 'element_type': et}
    if (eid, et) in seen:
        row['warning'] = 'duplicate element ID'
        return row
    seen.add((eid, et))
    try:
        if et in (15, 16):
            if dimension is None:
                row['geometry_error'] = 'Model dimensionality unknown; refusing to interpret union'
            else:
                row['geometry'] = native_curve(eh.GetElement(), 'arc' if et == 16 else 'ellipse', dimension)
        else:
            data = primitive_geometry(eh, dgn)
            if data:
                row['geometry'] = data
    except Exception as ex:
        row['geometry_error'] = repr(ex)
    try:
        kids = child_items(eh, dgn)
        if kids:
            row['children'] = [extract(c, model, dgn, dimension, depth + 1, seen) for c in kids]
    except Exception as ex:
        row['traversal_error'] = repr(ex)
    return row


def flatten(rows):
    for row in rows:
        yield row
        yield from flatten(row.get('children', []))


def process(file_obj, name, dgn):
    row = {'model': name}
    try:
        mid = file_obj.FindModelIdByName(name)
        model = file_obj.LoadModelById(mid)
        row['fill_status'] = str(file_obj.FillSectionsInModel(model, dgn.DgnModelSections.eModel))
        row['is_3d'] = is_3d(model)
        row['units'] = unit_info(model, dgn)
        refs = list(model.GetGraphicElements())
        row['elements'] = [extract(ref, model, dgn, row['is_3d']) for ref in refs]
        flat = list(flatten(row['elements']))
        row['element_count'] = len(flat)
        row['type_counts'] = dict(Counter(str(x.get('element_type')) for x in flat))
        row['geometry_count'] = sum(bool(x.get('geometry')) for x in flat)
        row['arc_count'] = sum(x.get('element_type') == 16 and x.get('geometry', {}).get('status') == 'native_2d_decoded' for x in flat if isinstance(x.get('geometry'), dict))
        row['ellipse_count'] = sum(x.get('element_type') == 15 and x.get('geometry', {}).get('status') == 'native_2d_decoded' for x in flat if isinstance(x.get('geometry'), dict))
        row['errors'] = [dict(id=x.get('id'), error=x.get('geometry_error') or x.get('traversal_error')) for x in flat if x.get('geometry_error') or x.get('traversal_error')]
    except Exception:
        row['error'] = traceback.format_exc()[-3000:]
    return row


def main():
    report = {'version': 14, 'timestamp_utc': datetime.now(timezone.utc).isoformat(),
              'read_only': True, 'target': TARGET, 'models': [], 'notes': [
                  'Cell geometry is not automatically classified as track centerlines.',
                  'Raw DGN coordinates are not converted to meters.',
                  '2D arcs use rotation + parameter angle; verify visually before insertion.']}
    log = []
    try:
        import MSPyBentleyGeom  # noqa: F401 - register geometry bindings first
        import MSPyDgnPlatform as dgn
        from MSPyMstnPlatform import ISessionMgr
        file_obj = ISessionMgr.ActiveDgnModelRef.GetDgnFile()
        names = [str(x.GetName()) for x in file_obj.GetModelIndex()]
        if TARGET not in names:
            raise RuntimeError('Open TURNOUT .cel: target model not found')
        first = process(file_obj, TARGET, dgn)
        report['models'].append(first)
        # Native decoding is the batch gate, not a CurveVector flag.
        if first.get('arc_count', 0) < 1 or first.get('ellipse_count', 0) < 1:
            log.append('Target native arc/ellipse gate failed; full batch skipped.')
        else:
            for name in names[:MAX_MODELS]:
                if name != TARGET:
                    report['models'].append(process(file_obj, name, dgn))
    except Exception:
        log.append(traceback.format_exc())
    finally:
        report['model_count'] = len(report['models'])
        OUTPUT.write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
        with SUMMARY.open('w', newline='', encoding='utf-8') as fh:
            writer = csv.DictWriter(fh, fieldnames=['model', 'is_3d', 'element_count', 'geometry_count', 'arc_count', 'ellipse_count', 'error_count', 'error'])
            writer.writeheader()
            for m in report['models']:
                writer.writerow({**{k: m.get(k) for k in ('model', 'is_3d', 'element_count', 'geometry_count', 'arc_count', 'ellipse_count')},
                                 'error_count': len(m.get('errors', [])), 'error': m.get('error', '')})
        LOG.write_text('\n'.join(log) + ('\n' if log else ''), encoding='utf-8')
        print('Wrote', OUTPUT, SUMMARY, LOG)


if __name__ == '__main__':
    main()
