# Turnout candidate matching v2 (read-only)

Run:

```bash
python -m pip install pyproj
python match_turnouts_v2.py railway_raw_osm.json turnout_geometry_v14.json --out refined_results
```

Outputs: `turnout_rankings.csv`, `turnout_rankings.geojson`, `summary.json`.

## What is compared

- Reconstructs graph-connected OSM switch arms, extending along degree-2 nodes.
- Identifies near-opposing through arms and measures the diverging arm's lateral offset from the through direction at several distances.
- Extracts decoded native circular arc radius and sweep from the MicroStation cell library and converts raw coordinates using the **unverified** UOR/master-unit interpretation.
- Tests several possible toe offsets and ranks models by lateral profile residual plus nominal frog-angle and handedness penalties.

## Critical limitations

This is **not** a verified full cell registration. The source library does not label which graphic primitives are centerlines, toe points, or frog points. The script therefore cannot safely align the entire cell geometry or distinguish variants with identical arc metadata. It also cannot verify left/right orientation or whether OSM depicts switch toe vs frog. The inferred raw-coordinate scale is **not calibrated** against a known physical dimension. All candidates remain `unverified`, regardless of score; do not use for automatic cell placement.

The comparison extrapolates sparse OSM branches as needed. OSM geometry and tagging can be incomplete. Rank differences are not probabilities.

For a true rigid registration stage, first identify and label the stock and diverging centerline primitives and a known anchor in each cell model, and calibrate the library units. Then align both routes to OSM using a rigid 2D transform, evaluate spatial and tangent residuals, and validate the top candidates visually.
