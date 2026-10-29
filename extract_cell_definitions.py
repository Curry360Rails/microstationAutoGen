#!/usr/bin/env python3
"""Inventory cell-library models (not geometry) in V8 .cel files via ezdgn."""
import argparse
import csv
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('library', type=Path, help='Path to a Bentley V8 .cel file')
    parser.add_argument('-o', '--output', type=Path, help='Output CSV path')
    args = parser.parse_args()
    if not args.library.is_file():
        parser.error(f'File not found: {args.library}')

    try:
        import ezdgn
    except ImportError as exc:
        raise SystemExit('Install dependency: python -m pip install ezdgn') from exc

    document = ezdgn.open_document(str(args.library))
    if not isinstance(document, ezdgn.V8Document):
        raise SystemExit('Input is not recognized as a V8 DGN cell library.')

    output = args.output or args.library.with_name(args.library.stem + '_definitions.csv')
    output.parent.mkdir(parents=True, exist_ok=True)
    fields = ['library', 'model_name', 'dimension', 'master_unit', 'entity_count']
    rows = []
    for model in document.models:
        meta = model.metadata
        rows.append({
            'library': args.library.name,
            'model_name': str(getattr(meta, 'name', '')),
            'dimension': str(getattr(meta, 'dimension', '')),
            'master_unit': str(getattr(meta, 'master_unit', '')),
            'entity_count': len(model.entities),
        })

    with output.open('w', newline='', encoding='utf-8-sig') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(f'Exported {len(rows)} models to {output}')
    print('Note: models are candidates for cell definitions; this is not a geometry export.')


if __name__ == '__main__':
    main()
