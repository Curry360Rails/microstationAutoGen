"""MicroStation 2025 Python Manager: import prepared OSM chains.

Options: -c native circular arcs, -s prepared clothoid spirals, -c -s both.
Without flags, only straight line-string pieces are drawn (arc/spiral pieces
are represented by their endpoint chords). Spiral pieces must have fitted
start bearing, start/end curvature, length, and placement frame metadata.

Run in a NEW DGN. Verify model GCS EPSG:26911 and meter storage units.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

PLAN = Path(__file__).with_name('railway_chains_v3.json')
DRY_RUN = False
CONFIRM_METERS_AND_EPSG_26911 = True
MAX_PIECES = None
PROGRESS_EVERY = 10


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('-c', '--curves', action='store_true', help='Write native circular ARC primitives')
    p.add_argument('-s', '--spirals', action='store_true', help='Write native fitted clothoid primitives')
    p.add_argument('--plan', type=Path, default=PLAN)
    p.add_argument('--write', action='store_true', help='Write to active DGN (requires confirmation constant)')
    p.add_argument('--max-pieces', type=int, default=MAX_PIECES)
    # Python Manager may inject arguments: parse only known args.
    args, unknown = p.parse_known_args()
    if unknown:
        print('Ignored unknown Python Manager arguments:', unknown, flush=True)
    return args


def success(value):
    if value is None:
        return True
    return str(value) in ('0', 'BentleyStatus.eSUCCESS', 'eSUCCESS') or value == 0


def run():
    args = parse_args()
    data = json.loads(args.plan.read_text(encoding='utf-8'))
    if data.get('schema') != 'osm_microstation_v3' or data.get('crs_epsg') != 26911:
        raise ValueError('Expected osm_microstation_v3 EPSG:26911 plan')
    pieces = [(chain.get('chain_id'), piece) for chain in data['chains'] for piece in chain['pieces']]
    if args.max_pieces is not None:
        if args.max_pieces < 1:
            raise ValueError('--max-pieces must be positive')
        pieces = pieces[:args.max_pieces]
    counts = {'line_string': 0, 'arc': 0, 'spiral': 0}
    for _, piece in pieces:
        counts[piece['type']] = counts.get(piece['type'], 0) + 1
    print('Plan:', args.plan, 'Pieces:', len(pieces), 'Types:', counts, flush=True)
    print('Native arcs:', args.curves, 'Native spirals:', args.spirals, flush=True)
    if DRY_RUN or not args.write:
        print('DRY RUN: no geometry created. Set DRY_RUN=False, confirmation=True and use --write.', flush=True)
        return
    if not CONFIRM_METERS_AND_EPSG_26911:
        raise RuntimeError('Confirm DGN storage units are meters and GCS is EPSG:26911 in script first')

    # Import all Bentley modules before accessing model ref; Python Manager
    # bindings sometimes fail converting DgnModelRef without the type registered.
    import MSPyDgnPlatform
    import MSPyBentleyGeom
    from MSPyMstnPlatform import ISessionMgr
    from MSPyDgnPlatform import DraftingElementSchema, EditElementHandle
    from MSPyBentleyGeom import DPoint3d, DSegment3d, DEllipse3d, ICurvePrimitive

    ref = ISessionMgr.GetActiveDgnModelRef()
    if ref is None or ref.GetDgnModel() is None:
        raise RuntimeError('Open a writable destination DGN in MicroStation Python Manager')
    model = ref.GetDgnModel()
    factor = float(model.GetModelInfo().GetUorPerStorage())
    print('Model:', model.GetModelName(), 'UOR/storage:', factor, flush=True)

    def pt(p):
        return DPoint3d(float(p[0]) * factor, float(p[1]) * factor, 0.0)

    def line_primitive(a, b):
        return ICurvePrimitive.CreateLine(DSegment3d(pt(a), pt(b)))

    def arc_primitive(piece):
        # Use documented ArcHandler directly for native ARC_ELM, rather than
        # relying on undocumented DEllipse3d constructors.
        from MSPyDgnPlatform import ArcHandler
        center = pt(piece['center'])
        radius = float(piece['radius_m']) * factor
        start = piece['start']
        angle = math.atan2(start[1] - piece['center'][1], start[0] - piece['center'][0])
        sweep = float(piece['sweep_rad'])
        eeh = EditElementHandle()
        status = ArcHandler.CreateArcElement(eeh, None, center, radius, radius,
                                             0.0, angle, sweep, ref.Is3d(), ref)
        if not success(status):
            raise RuntimeError('CreateArcElement failed: ' + str(status))
        result = eeh.AddToModel()
        if not success(result):
            raise RuntimeError('AddToModel failed: ' + str(result))

    def write_primitive(primitive):
        if primitive is None:
            raise RuntimeError('Curve primitive factory returned None')
        eeh = EditElementHandle()
        status = DraftingElementSchema.ToElement(eeh, primitive, None, ref.Is3d(), ref)
        if not success(status):
            raise RuntimeError('DraftingElementSchema.ToElement failed: ' + str(status))
        result = eeh.AddToModel()
        if not success(result):
            raise RuntimeError('AddToModel failed: ' + str(result))

    def spiral_primitive(piece):
        # Required prepared metadata: a transform placing the spiral origin,
        # plus signed curvature, bearing and length. Do not infer these from
        # circular arc center/sweep: that would shift the track alignment.
        required = ('start_bearing_rad', 'start_curvature_per_m',
                    'end_curvature_per_m', 'length_m', 'origin')
        missing = [key for key in required if key not in piece]
        if missing:
            raise ValueError('Spiral piece missing fitted parameters: ' + ', '.join(missing))
        from MSPyBentleyGeom import Transform, DSpiral2dClothoid
        # Bentley Transform constructor/placement overload must be verified
        # before using this path. Fail explicitly instead of writing an
        # unpositioned native spiral at (0,0).
        raise NotImplementedError(
            'Native spiral placement frame is not verified for this MicroStation '
            'build. No spiral written; supply documented Transform placement overload.')

    started = time.monotonic()
    made = 0
    for chain_id, piece in pieces:
        kind = piece['type']
        try:
            if kind == 'line_string':
                pts = piece['points']
                # Straight-only mode preserves each original polyline segment.
                # Bentley CreateLine uses a native LINE primitive.
                for a, b in zip(pts, pts[1:]):
                    if math.dist(a[:2], b[:2]) > 1e-8:
                        write_primitive(line_primitive(a, b))
                        made += 1
            elif kind in ('arc', 'circular_arc'):
                if args.curves:
                    arc_primitive(piece)
                else:
                    write_primitive(line_primitive(piece['start'], piece['end']))
                made += 1
            elif kind == 'spiral':
                if args.spirals:
                    write_primitive(spiral_primitive(piece))
                else:
                    write_primitive(line_primitive(piece['start'], piece['end']))
                made += 1
            else:
                raise ValueError('Unknown piece type: ' + str(kind))
        except Exception as exc:
            raise RuntimeError(f'Chain {chain_id} piece {kind} failed after {made} elements. '
                               'Do not rerun in same DGN (duplicates).') from exc
        done = pieces.index((chain_id, piece)) + 1
        if done % PROGRESS_EVERY == 0 or done == len(pieces):
            elapsed = time.monotonic() - started
            eta = elapsed / done * (len(pieces) - done)
            print(f'[{done}/{len(pieces)}] {done/len(pieces):.1%} '
                  f'created={made} elapsed={elapsed:.1f}s ETA={eta:.1f}s', flush=True)
    print('Finished. Elements created:', made, flush=True)


if __name__ == '__main__':
    run()
