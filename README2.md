# MicroStation 2025 native DXF export capability probe

## Status

MicroStation supports native DXF export, but the exact Python-callable command and automated per-model activation/export procedure have not been verified for this installation. This package intentionally does **not** claim to perform batch export yet. The probe is read-only.

## Run

1. Open `TURNOUT.cel` in MicroStation 2025.
2. Run `probe_dxf_api.py` from **Python Manager**, not system Bash Python.
3. Inspect or share `dxf_api_probe.json`, written beside the script.
4. In MicroStation, activate `10 LHIIUP` and manually export it using File > Export > DXF (or File > Save As > DXF, if available). Check that the output contains ARC and CIRCLE/ELLIPSE entities rather than only polyline approximations.
5. If you want to automate the UI command, record a MicroStation macro while doing that export and provide the recorded key-ins and export settings. Those can be combined with the model-activation APIs verified in the probe. Never assume a key-in from another MicroStation release is valid.

## Batch exporter requirements after verification

- Work from a copy of the source CEL.
- Enumerate all 150 models; skip empty `Default`.
- Activate one model and verify active model name before every export.
- Export to a temporary filename; verify existence and DXF header, then rename.
- Keep per-model success/error CSV and stop on any dialog or conversion error.
- Compare DXF ARC centers, radii, start/end angles and units against v14 geometry.
- Never generate DXFs from v14 JSON for this independent validation.

## References

- https://www.bentley.com/products/microstation
- https://blog.bentley.com/software/cad-file-formats-supported-by-microstation/
- https://bentleysystems.service-now.com/community?id=kb_article&sysparm_article=KB0040990
