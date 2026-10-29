Cell library model and geometry inspector (v2)

Requirements:
  python -m pip install ezdgn

Usage (Git Bash / Windows):
  python extract_cell_definitions.py "C:\\path\\to\\TURNOUT .cel"

Optional output prefix:
  python extract_cell_definitions.py "C:\\path\\to\\TURNOUT .cel" -o turnout

Outputs:
  turnout_definitions.csv   - model inventory, entity types, geometry coverage
  turnout_geometry.json    - recursive entity tree and exposed properties

Notes:
- Read-only: does not modify the CEL library.
- Coordinates are raw values exposed by ezdgn. No transforms are applied.
- Geometry detection is best-effort and does not guarantee a complete shape.
- Unresolved leaves are flagged in JSON and counted in CSV.
- This version has been syntax-checked but not run against ezdgn in this environment.
- If all elements remain unresolved, the Bentley MicroStation Python API is
  likely necessary for reliable extraction of DGN complex geometry.
