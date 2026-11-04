# Turnout placement preparation v1

This is a **safe first-stage generator**, not a working native cell inserter. It reads actual OSM topology and previously ranked turnout candidates, calculates approximate placement transforms, and writes a report. It does not alter a DGN. The `--write` option fails explicitly until the native Bentley cell API and cell anchor calibration have been verified.

Requires `pyproj` (`python -m pip install pyproj`) for EPSG:4326 -> EPSG:26911.

Example:

```
python place_turnouts.py --osm railway_raw_osm.json --rankings turnout_rankings.csv --max-turnouts 1 --out turnout_placement_plan.json
```

To enable a transform calculation, supply `--anchors anchors.json`, mapping exact model names to confirmed toe coordinates **in meters** and the local through-track bearing in degrees. The included example deliberately does not match any real cell. Original extracted cell geometry contains uncalibrated raw DGN coordinates, so converting those to meters requires explicit verification.

The highest-ranked turnout number is a candidate, not a verified identification. Through/branch directions inferred from 25 m OSM chords are preliminary; source cell origins, handedness, lead distances, and actual DGN GCS need verification before placement. No trimming is performed.

The script does not currently read GeoJSON; it uses raw Overpass JSON to preserve OSM node connectivity. `--cel` records the intended input but native insertion is not yet implemented.
