#!/usr/bin/env python3
"""Inspect Bentley V8 cell-library models using ezdgn (read-only).

This is a best-effort extractor: ezdgn's available entity properties vary by
version and entity type. Unrecognized entities are recorded, not discarded.
"""
import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path

SCALAR_FIELDS = ('name', 'type', 'entity_type', 'element_type', 'description',
                 'radius', 'start_angle', 'end_angle', 'sweep_angle', 'length',
                 'rotation', 'angle', 'is_closed', 'closed', 'level', 'color',
                 'line_style', 'line_weight', 'class_name', 'id')
POINT_FIELDS = ('origin', 'center', 'start', 'end', 'start_point', 'end_point',
                'point', 'normal', 'direction', 'translation', 'scale')
ARRAY_FIELDS = ('vertices', 'points', 'control_points', 'poles', 'knots', 'weights')
CHILD_FIELDS = ('children', 'entities', 'elements', 'components', 'geometry')


def plain(value, depth=0):
    """Convert a known property to JSON without executing arbitrary methods."""
    if depth > 6:
        return None
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else str(value)
    if isinstance(value, (list, tuple)):
        return [plain(v, depth + 1) for v in value[:100000]]
    if isinstance(value, dict):
        return {str(k): plain(v, depth + 1) for k, v in value.items()}
    if all(hasattr(value, k) for k in ('x', 'y')):
        try:
            return {k: plain(getattr(value, k), depth + 1)
                    for k in ('x', 'y', 'z') if hasattr(value, k)}
        except Exception:
            pass
    if hasattr(value, 'tolist') and callable(value.tolist):
        try:
            return plain(value.tolist(), depth + 1)
        except Exception:
            pass
    return None


def get_property(obj, key):
    try:
        val = getattr(obj, key)
        return None if callable(val) else val
    except Exception:
        return None


def get_children(obj):
    """Return first usable child collection, avoiding strings and dicts."""
    for field in CHILD_FIELDS:
        val = get_property(obj, field)
        if val is None or isinstance(val, (str, bytes, dict)):
            continue
        try:
            items = list(val)
        except (TypeError, ValueError, RuntimeError):
            continue
        if items and all(isinstance(v, (int, float, str, bytes)) for v in items):
            continue
        if items:
            return field, items
    return None, []


def inspect_entity(entity, seen, stats, max_depth=20, depth=0):
    typename = type(entity).__name__
    stats['types'][typename] += 1
    node = {'python_type': typename, 'properties': {}, 'children': []}
    if id(entity) in seen:
        node['warning'] = 'Repeated reference / cycle'
        stats['warnings'] += 1
        return node
    if depth >= max_depth:
        node['warning'] = 'Maximum nesting depth reached'
        stats['warnings'] += 1
        return node
    seen.add(id(entity))
    for field in SCALAR_FIELDS + POINT_FIELDS + ARRAY_FIELDS:
        val = get_property(entity, field)
        if val is None:
            continue
        result = plain(val)
        if result is not None:
            node['properties'][field] = result
    child_field, children = get_children(entity)
    if children:
        node['child_collection'] = child_field
        node['children'] = [inspect_entity(c, seen, stats, max_depth, depth + 1)
                            for c in children]
    geom_fields = set(POINT_FIELDS + ARRAY_FIELDS + ('radius', 'start_angle',
                                                     'end_angle', 'sweep_angle', 'length'))
    if any(k in node['properties'] for k in geom_fields):
        stats['with_geometry'] += 1
    elif not children:
        stats['unresolved'] += 1
        node['warning'] = 'No supported geometry properties or nested entities exposed'
    return node


def export(library, output_prefix, max_depth):
    try:
        import ezdgn
    except ImportError as exc:
        raise SystemExit('Install dependency: python -m pip install ezdgn') from exc
    doc = ezdgn.open_document(str(library))
    if not isinstance(doc, ezdgn.V8Document):
        raise SystemExit('Input is not recognized as a V8 DGN cell library.')
    rows, models = [], []
    for model in doc.models:
        meta = model.metadata
        entities = list(model.entities)
        stats = {'types': Counter(), 'with_geometry': 0, 'unresolved': 0, 'warnings': 0}
        seen = set()
        tree = [inspect_entity(e, seen, stats, max_depth) for e in entities]
        name = str(get_property(meta, 'name') or '')
        unit = str(get_property(meta, 'master_unit') or '')
        dimension = str(get_property(meta, 'dimension') or '')
        rows.append({
            'library': library.name, 'model_name': name, 'dimension': dimension,
            'master_unit': unit, 'entity_count': len(entities),
            'element_types': '; '.join(f'{k}:{v}' for k, v in sorted(stats['types'].items())),
            'elements_with_geometry_properties': stats['with_geometry'],
            'unresolved_leaf_elements': stats['unresolved'],
            'warnings': stats['warnings'],
        })
        models.append({'name': name, 'dimension': dimension, 'master_unit': unit,
                       'entities': tree})
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    csv_path = output_prefix.with_name(output_prefix.name + '_definitions.csv')
    json_path = output_prefix.with_name(output_prefix.name + '_geometry.json')
    with csv_path.open('w', newline='', encoding='utf-8-sig') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]) if rows else [
            'library', 'model_name', 'dimension', 'master_unit', 'entity_count',
            'element_types', 'elements_with_geometry_properties',
            'unresolved_leaf_elements', 'warnings'])
        writer.writeheader()
        writer.writerows(rows)
    payload = {
        'source': str(library),
        'reader': 'ezdgn',
        'coordinate_note': 'Coordinates are emitted as exposed by ezdgn; no origin, transform, or unit conversion has been applied.',
        'limitations': 'Best-effort property introspection; absence of properties does not mean geometry is absent. Nested transforms and complex primitives may require Bentley APIs.',
        'models': models,
    }
    with json_path.open('w', encoding='utf-8') as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
    print(f'Exported {len(rows)} models to:\n  {csv_path}\n  {json_path}')
    print(f"Geometry properties on {sum(r['elements_with_geometry_properties'] for r in rows)} elements; "
          f"unresolved leaves: {sum(r['unresolved_leaf_elements'] for r in rows)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('library', type=Path, help='Path to Bentley V8 .cel library')
    parser.add_argument('-o', '--output-prefix', type=Path, help='Prefix for output CSV/JSON')
    parser.add_argument('--max-depth', type=int, default=20)
    args = parser.parse_args()
    if not args.library.is_file():
        parser.error(f'File not found: {args.library}')
    if args.max_depth < 1:
        parser.error('--max-depth must be >= 1')
    export(args.library, args.output_prefix or args.library.with_suffix(''), args.max_depth)


if __name__ == '__main__':
    main()
