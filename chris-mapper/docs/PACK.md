# Map packs (`.rsmap`)

What a map pack holds and its file format: the data the map box reads (`chris_mapper/rsmap.py`: the reader
`Pack` and the writer `build_pack`; the builder tool that reads OpenStreetMap extracts lives in chris-roadsense,
[`tools/roadsense_maps.py`](https://github.com/PiotrTopa/chris-roadsense/blob/main/tools/roadsense_maps.py), [`scripts/roadsense-maps`](https://github.com/PiotrTopa/chris-roadsense/blob/main/scripts/roadsense-maps)). Moved verbatim from chris-roadsense's [`docs/MAPS.md`](https://github.com/PiotrTopa/chris-roadsense/blob/main/docs/MAPS.md) (lane L43); "mapd" in it names
the legacy daemon that read packs before the map box, "roadsense 2.0" the release that brought format 2.

**Attribution.** Map data © OpenStreetMap contributors, available under the Open Database License (ODbL)
1.0, https://www.openstreetmap.org/copyright. A pack is a derived database under the same licence; its
manifest carries the attribution and the date of the OSM data, and the status page prints it. Anything
that shows roadsense's map-derived output to the public has to show that attribution; a pack that is
passed on has to stay under the ODbL. Extracts come from Geofabrik (https://download.geofabrik.de).

## What a pack holds

* **Roads:** `highway` = motorway, trunk, primary, secondary, tertiary (and their `_link`s),
  unclassified, residential, living_street, service (`track` only with `--tracks`); `area=yes` is left
  out. Ways are split into graph edges at every node shared by two ways and at built-up / forest changes.
  `oneway=-1` ways are reversed so that an edge's geometry order is its direction of travel.
* **Per edge:** class, name, ref, one-way, roundabout, tunnel, bridge, covered, lit (yes / no / unknown),
  motorroad, link, dual carriageway (tagged, or a one-way major road), toll, restricted access, parking
  aisle, layer, surface, lanes (total, forward, backward), `maxspeed` per direction with its qualifier
  (from a sign, implicit zone value, variable), built-up flag and whether a tag states it, forest flag.
* **Turn lanes** (format 2): `turn:lanes` of a one-way way, `turn:lanes:forward` / `turn:lanes:backward` of a
  two-way one, per direction of travel, lanes from the left separated by `|`, the arrows of a lane by `;`
  (`left`, `slight_left`, `sharp_left`, `through`, `right`, `slight_right`, `sharp_right`, `reverse`,
  `merge_to_left`, `merge_to_right`; `none` = no arrow painted). A plain `turn:lanes` on a two-way way is
  ambiguous and left out, as are values with more than 12 lanes or without any arrow. Where the lane count is
  not tagged, the arrows count the lanes of their direction. In the Poland extract of 2026-11-14: 67,779 ways
  (13 % of tertiary and above), 75,813 of the 5.5 million edges of the pack.
* **Built-up area.** From tags on the way when they exist (`maxspeed` / `zone:traffic` / `zone:maxspeed`
  / `maxspeed:type` / `source:maxspeed` = `PL:urban`, `PL:rural`); otherwise **inferred**: the way is
  sampled every 40 m against a raster (1/2048 degree, about 54 m x 33 m) of `landuse` = residential,
  commercial, retail, industrial polygons, dilated by two cells; runs shorter than 200 m are merged.
  Motorways and expressways are never inferred built-up; living streets always are. The legal built-up
  area is defined by the D-42 / D-43 signs, which the pack does not hold: the inferred flag is a proxy
  (confidence 0.5 in the `map` message, source `landuse`).
* **Forest:** the same sampling against `landuse=forest` / `natural=wood`, dilated by one cell (a road
  runs in the gap between two forest polygons).
* **Point features** on the roads: traffic signals, pedestrian crossings (with and without signals),
  railway and tram level crossings, stop and give-way signs, mini roundabouts, speed bumps, toll booths,
  motorway junctions, and (format 2) speed cameras (`highway=speed_camera`) where the node is part of the
  road: 117 of the 643 in the extract; a camera mapped beside the road is not in the pack. Each with the
  direction it applies to when tagged.
* **Not in a pack:** turn restrictions, lane-level geometry, `destination` signs, traffic-sign nodes
  (`traffic_sign=city_limit` etc.), speed cameras that are not a node of the road, fuel stations and other
  POIs, elevation.

## Format (`.rsmap`, version 2; version 1 is still read)

Reference implementation: `chris_mapper/rsmap.py` (writer `build_pack`, reader `Pack`).
Little-endian.

```
header   64 bytes   "RSMAP001", u32 version = 2, u32 manifest_len, u64 manifest_off, u64 index_off, u64 index_count
manifest            UTF-8 JSON: region, source, osm_timestamp, built, attribution, licence, counts, enum tables
tiles               one zlib-compressed blob per non-empty tile
index               index_count x {u32 key, u64 offset, u32 compressed_len, u32 raw_len}, sorted by key
```

A tile is a 1/32 degree cell (about 3.5 km x 2.1 km in Poland), `key = ty << 16 | tx`,
`tx = floor((lon + 180) * 32)`, `ty = floor((lat + 90) * 32)`. The reader maps the file, finds a tile by
binary search in the index, decompresses it on first use and keeps the last 24 tiles. A tile blob:

```
header     10 x u32: strings_bytes, nodes, adjacency, edges, points, features, foreign, grid_n (16), grid_items, 0
strings    u16 length + UTF-8, repeated (names and refs; index 0 = none), padded to 4 bytes
nodes      {i32 x, i32 y, u32 adj_start, u16 adj_count, u16 flags}        graph nodes inside this tile
adjacency  {u32 tile, u16 edge, u16 end}                                  edges at a node; end 0 = the node is the edge's start
edges      48 bytes each: start node (tile, index), end node (tile, index), point range, feature range,
           length (dm), flags u16, name, ref, class, flags2, maxspeed forward / backward (km/h, 0 = none,
           255 = "none"), lanes total / forward / backward, layer, surface, a pad byte, and (version 2) the turn
           lanes forward / backward as u16 indices into the tile's strings (0 = none). Version 1: 44 bytes,
           the same record without the last two fields
points     {i32 dx, i32 dy} micro-degrees: an edge's first point relative to the tile's south-west corner,
           the following ones relative to their predecessor
features   {u16 point index, u8 kind, u8 direction (0 both, 1 forward, 2 backward)}
foreign    {u32 tile, u16 edge}: edges of other tiles that pass through this one
grid       257 x u32 starts + u16 items: per 1/512 degree cell the edges touching it (index < edges: own
           edge, otherwise foreign[index - edges])
```

An edge lives in the tile of its first point; a spatial query reads the grid cells around a point and
follows foreign references, so an edge is found from every tile it crosses. The enum tables (classes,
surfaces, features, flag bits) are in `rsmap.py` and in the manifest; they are part of the format.
**Versions.** 2 (2026-11-18, roadsense 2.0): turn lanes per edge, the feature kind `speed_camera`. 1: without
them. The reader of roadsense 2.0 reads both (`Pack.version`; a version 1 pack simply has no turn lanes, and
`map.pack.format` says which is loaded) and refuses any other version with an error that names it; mapd then
logs it and runs without a map layer (`reason: "no_pack"`). The reader of roadsense 1.x refuses a version 2
pack the same way, so on a device roadsense is updated before the pack.
