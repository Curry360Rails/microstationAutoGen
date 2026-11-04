"""MicroStation 2025 Python Manager: connected OSM chains and sampled circular bends.

Input: railway_chains_v3.json beside this script, EPSG:26911 meters.
Diagnostic default: insert only ONE line-string element into a NEW writable DGN.
No turnout cells are inserted and no centerlines are trimmed.
"""
import json
import math
import time
from pathlib import Path

PLAN = Path(__file__).with_name('railway_chains_v3.json')
DRY_RUN = False
CONFIRM_METERS_AND_EPSG_26911 = True
MAX_PIECES = None
DRAW_SWITCH_MARKERS = False
ARC_MAX_CHORD_METERS = 2.0
LOCAL_ORIGIN_METERS = None
PROGRESS_EVERY = 1
VERBOSE_API_LOG = True
PROGRESS_BAR_WIDTH = 30


def sample_arc(piece, max_chord):
    center = piece['center']
    radius = float(piece['radius_m'])
    start = piece['start']
    sweep = float(piece['sweep_rad'])
    if max_chord <= 0 or radius <= 0:
        raise ValueError('Arc radius and maximum chord must be positive')
    angle = math.atan2(start[1] - center[1], start[0] - center[0])
    count = max(2, int(math.ceil(abs(sweep) * radius / max_chord)))
    points = [[center[0] + radius * math.cos(angle + sweep * i / count),
               center[1] + radius * math.sin(angle + sweep * i / count)]
              for i in range(count + 1)]
    points[0] = start
    points[-1] = piece['end']
    return points


def progress(done, total, start_time, label='Drawing', force=False):
    if not force and done != total and done % max(1, PROGRESS_EVERY):
        return
    elapsed = time.monotonic() - start_time
    fraction = done / total if total else 1.0
    filled = round(PROGRESS_BAR_WIDTH * fraction)
    bar = '#' * filled + '-' * (PROGRESS_BAR_WIDTH - filled)
    eta = elapsed / done * (total - done) if done else 0.0
    print(f'{label} [{bar}] {done}/{total} ({fraction:.1%}) '
          f'elapsed={elapsed:.1f}s ETA={eta:.1f}s', flush=True)


def status_is_success(status):
    """Compare Bentley's enum directly; support a numeric success result as fallback."""
    try:
        from MSPyBentley import BentleyStatus
        return status == BentleyStatus.eSUCCESS
    except ImportError:
        # Do not depend on int(enum) working on all Bentley bindings.
        return str(status) in ('BentleyStatus.eSUCCESS', '0')


def run():
    from MSPyMstnPlatform import ISessionMgr
    from MSPyDgnPlatform import EditElementHandle, LineStringHandler
    from MSPyBentleyGeom import DPoint3d, DPoint3dArray

    data = json.loads(PLAN.read_text(encoding='utf-8'))
    if data.get('schema') != 'osm_microstation_v3' or data.get('crs_epsg') != 26911:
        raise ValueError('Expected osm_microstation_v3 EPSG:26911 plan')

    print('Before GetActiveDgnModelRef', flush=True)
    try:
        ref = ISessionMgr.GetActiveDgnModelRef()
    except Exception as exc:
        print(
            "GetActiveDgnModelRef failed:",
            type(exc).__name__,
            str(exc),
            flush=True
        )
        raise
    print('After GetActiveDgnModelRef', flush=True)
    if ref is None:
        raise RuntimeError('No active DGN model reference')

    print('Before GetDgnModel', flush=True)
    model = ref.GetDgnModel()
    print('After GetDgnModel', flush=True)
    if model is None:
        raise RuntimeError('No active DGN model')
    info = model.GetModelInfo()
    factor = float(info.GetUorPerStorage())
    print('Model:', model.GetModelName(), flush=True)
    print('UOR per storage:', factor, 'chains:', data['stats']['chains'],
          'pieces:', data['stats']['drawing_pieces'], flush=True)
    print('Sampled circular pieces:', data['stats']['fitted_arc_pieces'], flush=True)
    print('Origin offset:', LOCAL_ORIGIN_METERS, flush=True)
    if DRY_RUN:
        print('DRY RUN: no geometry created', flush=True)
        return
    if not CONFIRM_METERS_AND_EPSG_26911:
        raise RuntimeError('Confirm meter storage units and destination GCS first')

    def create_line_string(points):
        if len(points) < 2:
            return
        arr = DPoint3dArray()
        ox, oy = LOCAL_ORIGIN_METERS if LOCAL_ORIGIN_METERS is not None else (0.0, 0.0)
        for p in points:
            arr.append(DPoint3d((float(p[0]) - ox) * factor,
                                (float(p[1]) - oy) * factor, 0.0))
        handle = EditElementHandle()
        if VERBOSE_API_LOG:
            print('Before CreateLineStringElement', len(points), 'points', flush=True)
        status = LineStringHandler.CreateLineStringElement(handle, None, arr, ref.Is3d(), ref)
        if VERBOSE_API_LOG:
            print('After CreateLineStringElement', status, flush=True)
        if not status_is_success(status):
            raise RuntimeError('LineString creation failed: ' + str(status))
        if VERBOSE_API_LOG:
            print('Before AddToModel', flush=True)
        result = handle.AddToModel()
        if VERBOSE_API_LOG:
            print('After AddToModel', result, flush=True)
        if result is not None and result != 0:
            raise RuntimeError('AddToModel failed: ' + str(result))

    total_track_elements = 0
    for chain in data['chains']:
        for piece in chain['pieces']:
            n = (len(piece['points']) if piece['type'] == 'line_string'
                 else len(sample_arc(piece, ARC_MAX_CHORD_METERS)))
            if n >= 2:
                total_track_elements += (n - 2) // 99 + 1
    total_markers = (2 * sum(bool(sw.get('tagged')) for sw in data['switches'])
                     if DRAW_SWITCH_MARKERS else 0)
    total_elements = total_track_elements + total_markers
    if MAX_PIECES is not None:
        total_elements = min(total_elements, MAX_PIECES)
    print('Planned MicroStation elements:', total_elements, flush=True)
    started = time.monotonic()
    made = 0
    progress(0, total_elements, started, force=True)

    for chain in data['chains']:
        for piece in chain['pieces']:
            pts = (piece['points'] if piece['type'] == 'line_string'
                   else sample_arc(piece, ARC_MAX_CHORD_METERS))
            for i in range(0, len(pts) - 1, 99):
                chunk = pts[i:i + 100]
                if len(chunk) < 2:
                    continue
                try:
                    create_line_string(chunk)
                except Exception as exc:
                    raise RuntimeError('Chain %s, piece %s failed after %s elements; '
                                       'do not rerun in same DGN: %s'
                                       % (chain['chain_id'], piece['type'], made, exc)) from exc
                made += 1
                progress(made, total_elements, started)
                if MAX_PIECES is not None and made >= MAX_PIECES:
                    print('Stopped at MAX_PIECES:', made, flush=True)
                    return

    if DRAW_SWITCH_MARKERS:
        for sw in data['switches']:
            if not sw.get('tagged'):
                continue
            x, y = sw['point'][:2]
            s = 2.0
            for marker in ([[x - s, y - s], [x + s, y + s]],
                           [[x - s, y + s], [x + s, y - s]]):
                create_line_string(marker)
                made += 1
                progress(made, total_elements, started)
                if MAX_PIECES is not None and made >= MAX_PIECES:
                    print('Stopped at MAX_PIECES:', made, flush=True)
                    return
    progress(made, total_elements, started, force=True)
    print('Completed line-string elements:', made, flush=True)
    print('No turnout cells placed; sampled arcs are line strings, not native ARC entities.', flush=True)


if __name__ == '__main__':
    run()
