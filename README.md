# MicroStation DXF Export v4 — safe orchestration scaffold

Run `export_turnouts_v4.py` using MicroStation 2025 Python Manager.

Configure absolute `source_cel`, `seed_dgn`, and `output_dir` in `export_config.json`. No active model name is assumed. The source CEL is never modified. The default is a one-model dry run.

## Important limitation

The installed Python probe confirmed `PyCadInputQueue.SendKeyin()` and model lookup, but did not establish documented APIs/key-ins for creating a temporary DGN, placing a cell, saving it as DXF, or restoring the original session. Consequently, this version intentionally does not execute those unverified operations. Changing `mode` does not bypass this safety gate. It produces a JSON and CSV report beside the script. No DXFs are created yet.

The next validation is to capture the exact native Save As key-in and file-creation/placement workflow from MicroStation, then implement and verify a single-model export before batch processing.
