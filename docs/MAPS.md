# Map packs

mapd (docs/ARCHITECTURE.md "mapd") reads the road network from a **map pack**: one `.rsmap` file built
from an OpenStreetMap extract. Packs and extracts are data, not source: they are **never committed**
(`.gitignore` blocks `*.rsmap`, `*.osm.pbf`, `*.pbf`), they live outside every repository in
`~/roadsense-maps` on the dev box and under `/var/lib/roadsense/maps` on the device.

**Attribution.** Map data © OpenStreetMap contributors, available under the Open Database License (ODbL)
1.0, https://www.openstreetmap.org/copyright. A pack is a derived database under the same licence; its
manifest carries the attribution and the date of the OSM data, and the status page prints it. Anything
that shows roadsense's map-derived output to the public has to show that attribution; a pack that is
passed on has to stay under the ODbL. Extracts come from Geofabrik (https://download.geofabrik.de).

## Tool: `scripts/roadsense-maps`

```sh
python3 -m venv ~/roadsense-maps/.venv && ~/roadsense-maps/.venv/bin/pip install osmium numpy pillow   # once, dev box
export PYTHON=~/roadsense-maps/.venv/bin/python
scripts/roadsense-maps fetch                    # Geofabrik europe/poland -> ~/roadsense-maps/src/geofabrik/ (md5-checked)
systemd-run --user --wait -p MemoryMax=24G -p CPUQuota=300% \
    $PYTHON tools/roadsense_maps.py build       # -> ~/roadsense-maps/packs/poland-<osm date>.rsmap
scripts/roadsense-maps info   ~/roadsense-maps/packs/poland-YYYYMMDD.rsmap     # manifest
scripts/roadsense-maps verify ~/roadsense-maps/packs/poland-YYYYMMDD.rsmap     # decodes every tile (--deep: every cross reference)
scripts/roadsense-maps install ~/roadsense-maps/packs/poland-YYYYMMDD.rsmap --host user@phone   # copy + checksum, prints the activation step
```

`--region europe/poland/opolskie` (any Geofabrik path) builds a smaller pack; `--dir` or
`ROADSENSE_MAPS_DIR` moves the data directory; `build --tracks` adds `highway=track`;
`build --no-landuse` skips the built-up / forest inference. `build` needs pyosmium, numpy and Pillow and
runs on the dev box; the device needs numpy only.

**Install on the device.** `install` copies the pack with rsync (resumable) to `~/roadsense-maps/` of
the ssh user, compares the sha256 there, renames it into place and prints the one root step that makes
it current:

```sh
sudo install -d /var/lib/roadsense/maps
sudo ln -sfn /home/user/roadsense-maps/poland-YYYYMMDD.rsmap /var/lib/roadsense/maps/current.rsmap
```

mapd checks the pack path every 10 s and loads a new file without a restart; nothing else has to be
touched. Old packs are deleted by hand. `install --dry-run` prints the commands only.

## The Poland pack (built 2026-11-15 on the dev box)

| | |
|---|---|
| source | Geofabrik `europe/poland-latest.osm.pbf`, 2.12 GB, OSM data of 2026-11-14T21:20:41Z |
| pack | `~/roadsense-maps/packs/poland-20261114.rsmap`, **262.2 MB** (262,210,224 bytes) |
| content | 41,894 tiles, 5,518,936 edges, 4,886,175 nodes, 24,712,702 points; 607,379 km of road (service 174,364, residential 163,311, unclassified 104,171, tertiary 95,015, secondary 31,916, primary 15,323, motorway 10,996, living street 7,559, trunk 2,109, links 2,616) |
| build | 27 min wall, 30 min CPU, 8 GB peak (`systemd-run --user -p MemoryMax=24G -p CPUQuota=300%`; i7-8850H): roads 330 s, landuse 283 s, graph 593 s, write 414 s |
| verify | `roadsense-maps verify`: every tile decoded, 0 broken references, 4.5 min |
| mapd on it | 43 MB resident, 0.6 % of one laptop core with GNSS at 1 Hz, motion at 20 Hz and 2 `map` messages per second (docs/lanes/L13-mapd.md); not measured on the phone |

## What a pack holds

* **Roads:** `highway` = motorway, trunk, primary, secondary, tertiary (and their `_link`s),
  unclassified, residential, living_street, service (`track` only with `--tracks`); `area=yes` is left
  out. Ways are split into graph edges at every node shared by two ways and at built-up / forest changes.
  `oneway=-1` ways are reversed so that an edge's geometry order is its direction of travel.
* **Per edge:** class, name, ref, one-way, roundabout, tunnel, bridge, covered, lit (yes / no / unknown),
  motorroad, link, dual carriageway (tagged, or a one-way major road), toll, restricted access, parking
  aisle, layer, surface, lanes (total, forward, backward), `maxspeed` per direction with its qualifier
  (from a sign, implicit zone value, variable), built-up flag and whether a tag states it, forest flag.
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
  motorway junctions, each with the direction it applies to when tagged.
* **Not in a pack:** turn restrictions, lane-level geometry and turn lanes, traffic-sign nodes
  (`traffic_sign=city_limit` etc.), speed cameras, fuel stations and other POIs, elevation.

## Format (`.rsmap`, version 1)

Reference implementation: `mapd/roadsense_mapd/rsmap.py` (writer `build_pack`, reader `Pack`).
Little-endian.

```
header   64 bytes   "RSMAP001", u32 version = 1, u32 manifest_len, u64 manifest_off, u64 index_off, u64 index_count
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
edges      44 bytes each: start node (tile, index), end node (tile, index), point range, feature range,
           length (dm), flags u16, name, ref, class, flags2, maxspeed forward / backward (km/h, 0 = none,
           255 = "none"), lanes total / forward / backward, layer, surface
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
A reader refuses any other version.
