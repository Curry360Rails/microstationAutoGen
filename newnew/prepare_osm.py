"""Prepare raw Overpass railway JSON for MicroStation. Run with ordinary Python.
Requires: python -m pip install pyproj
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path

RAIL_TYPES = {'rail', 'light_rail', 'tram', 'narrow_gauge', 'subway'}


def prepare(source, epsg=26911):
    from pyproj import Transformer
    data = json.loads(Path(source).read_text(encoding='utf-8'))
    elements = data.get('elements')
    if not isinstance(elements, list):
        raise ValueError('Expected raw Overpass JSON with elements array')
    # Overpass exports can repeat nodes: untagged geometry copies must not
    # overwrite earlier tagged switch records. Merge tags by node ID.
    nodes = {}
    for e in elements:
        if e.get('type') != 'node' or 'lon' not in e or 'lat' not in e:
            continue
        nid = int(e['id'])
        if nid in nodes:
            merged = dict(e)
            merged['tags'] = {**nodes[nid].get('tags', {}), **e.get('tags', {})}
            nodes[nid] = merged
        else:
            nodes[nid] = e
    ways = [e for e in elements if e.get('type') == 'way' and e.get('tags', {}).get('railway') in RAIL_TYPES]
    transform = Transformer.from_crs('EPSG:4326', f'EPSG:{epsg}', always_xy=True)
    coords = {nid: list(transform.transform(n['lon'], n['lat'])) for nid, n in nodes.items()}
    neighbors = defaultdict(set)
    track_ways = []
    missing = []
    for way in ways:
        ids = [int(i) for i in way.get('nodes', [])]
        # Do not bridge missing nodes: split into contiguous runs.
        runs = []
        current = []
        for nid in ids:
            if nid in coords:
                current.append(nid)
            else:
                missing.append({'way_id': way['id'], 'node_id': nid})
                if len(current) > 1: runs.append(current)
                current = []
        if len(current) > 1: runs.append(current)
        for run in runs:
            for a, b in zip(run, run[1:]):
                if a != b:
                    neighbors[a].add(b)
                    neighbors[b].add(a)
            track_ways.append({'way_id': way['id'], 'node_ids': run, 'points': [coords[n] for n in run], 'tags': way.get('tags', {})})
    switch_ids = {nid for nid, n in nodes.items() if n.get('tags', {}).get('railway') == 'switch'}
    switches = []
    for nid in sorted(switch_ids | {n for n, adj in neighbors.items() if len(adj) >= 3}):
        tagged = nid in switch_ids
        degree = len(neighbors.get(nid, ()))
        switches.append({'node_id': nid, 'point': coords[nid], 'tagged': tagged, 'degree': degree,
                         'status': 'tagged_connected' if tagged and degree >= 3 else
                                   'tagged_review' if tagged else 'inferred_branch',
                         'neighbor_ids': sorted(neighbors.get(nid, ())),
                         'tags': nodes[nid].get('tags', {})})
    return {'schema': 'osm_microstation_v2', 'crs_epsg': epsg,
            'source': str(Path(source).resolve()), 'tracks': track_ways,
            'switches': switches, 'missing_nodes': missing,
            'counts': {'railway_ways': len(ways), 'track_runs': len(track_ways),
                       'tagged_switches': len(switch_ids),
                       'tagged_switches_on_tracks': sum(n in neighbors for n in switch_ids),
                       'inferred_branches': sum(s['status'] == 'inferred_branch' for s in switches)}}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('osm_json', type=Path)
    p.add_argument('--out', type=Path, default=Path('railway_microstation_plan.json'))
    p.add_argument('--epsg', type=int, default=26911)
    a = p.parse_args()
    plan = prepare(a.osm_json, a.epsg)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(plan, indent=2), encoding='utf-8')
    print(json.dumps(plan['counts'], indent=2))
    print('Output:', a.out.resolve())
    if plan['missing_nodes']: print('WARNING: missing node references:', len(plan['missing_nodes']))

if __name__ == '__main__': main()
