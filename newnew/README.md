# OSM-to-MicroStation Railway v3

## What's new

- Builds a topology graph from the existing EPSG:26911 `railway_microstation_plan.json`.
- Splits at tagged switches, nodes with degree other than two, and closed loops; joins across original OSM way boundaries where degree is two.
- Creates 188 chains from 2,763 unique edges (actual dataset).
- Uses circular radius candidates extracted from `turnout_geometry_v14.json` to replace short three-point bends **only when** the catalogue radius is within 6% of the measured radius and the intermediate point deviates no more than 0.35 m. These are *approximations*, not proof of turnout dimensions.
- Samples accepted circles at up to 2 m chord spacing; MicroStation writes **line strings**, not native arcs. Straight runs become line strings as well. Long strings are split into 100-point chunks.
- Preserves OSM node IDs, chain endpoints and switch metadata in `railway_chains_v3.json`.

## Offline preparation

```bash
python prepare_chains.py railway_microstation_plan.json --v14 turnout_geometry_v14.json --meters-per-storage 1.0 --out railway_chains_v3.json
```

The supplied plan is already prepared. **Important:** `--meters-per-storage 1.0` is an assumption for the raw cell geometry scale; the cell library has feet master units, and its actual radius calibration should be verified against a known dimension before treating selected radii as physically meaningful. The script retains measured fit radii and errors for auditing. To use OSM line strings without any catalogue approximation, omit `--v14`.

## MicroStation

Copy `build_microstation.py` and `railway_chains_v3.json` together. Open a NEW destination DGN in MicroStation 2025, with meter storage units and the appropriate EPSG:26911 GCS. Run from Python Manager.

1. First run `DRY_RUN=True` to check the summary.
2. For the first write, set `DRY_RUN=False`, `CONFIRM_METERS_AND_EPSG_26911=True`, `MAX_PIECES=1`.
3. Inspect measured length, location, and `After AddToModel` output. Use a fresh DGN for each test.
4. Set `MAX_PIECES=None` to create the full linework in another clean DGN.

**Coordinate caveat:** MicroStation's geometry handler uses UORs in this workflow. This script multiplies meters by `GetUorPerStorage()` based on the earlier successful 1/10,000 scale diagnosis. Full UTM coordinates produce large UOR values; this has previously caused MicroStation to become unresponsive in your setup. Do not run a full import until one full-coordinate test succeeds. `LOCAL_ORIGIN_METERS=(474000,3828000)` may help diagnose this, but then the output is *not georeferenced* unless the DGN coordinate system is configured to account for the offset.

**Bentley API compatibility:** The `LineStringHandler.CreateLineStringElement` signature and `DPoint3dArray` availability have not been executed inside MicroStation 2025 in this environment. The script logs before and after both geometry creation and insertion. If the call fails, share the exact exception; no silent fallback to thousands of separate lines is performed.

## Not yet implemented

- True native ARC / complex-chain elements with alternating line and arc primitives (this v3 creates line-string pieces and sampled arcs).
- Insertion or orientation of TURNOUT.cel cells, trimming rails to cell connections, or verifying turnout number/handedness.
- Independently verified georeferencing or precise rail design geometry.

## Progress bar (v3.1)

`build_microstation.py` now prints a throttled ASCII progress bar with the number of MicroStation elements inserted, percentage, elapsed time, and estimated remaining time. The denominator includes split line-string chunks and optional switch-marker elements. `PROGRESS_EVERY = 25` controls update frequency; use `1` for a one-element test. The output uses separate lines because MicroStation Python Manager may not handle carriage-return redraws reliably. `VERBOSE_API_LOG = True` enables per-element Bentley API diagnostics if a call stalls. The progress count increments only after `AddToModel()` returns successfully. No external packages are required.
