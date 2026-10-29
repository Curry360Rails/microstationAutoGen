Cell library definition inventory (initial version)

This script uses the third-party ezdgn V8 reader, not MicroStation's embedded Python API.
It can run with system Python, including Python 3.14, subject to ezdgn installation.

Install:
  python -m pip install ezdgn

Run:
  python extract_cell_definitions.py "C:\path\to\TURNOUT .cel"

Optional output:
  python extract_cell_definitions.py "C:\path\to\TURNOUT .cel" -o turnout_definitions.csv

The CSV inventories model names, dimensions, master units, and entity counts.
A cell library can also contain non-cell models. The script does not validate
cell types, export geometry, or modify source files.

The third-party reader is early-stage software; verify results by comparing
model names with MicroStation 2025's cell library interface.
