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
| verify | `roadsense-maps verify`: every tile decoded, 0 broken references, 4.5 min (phone, one big core: 9.5 min, 317 MB peak) |
| mapd on it | 43 MB resident, 0.6 % of one laptop core with GNSS at 1 Hz, motion at 20 Hz and 2 `map` messages per second (docs/lanes/L13-mapd.md); phone (2026-11-15, little cores, Nice 10): idle 0.35 %, synthetic drives 8-10 % of one core, 46-52 MB RSS, step 15-22 ms; tile load 10.6 ms cold / 4.4 ms warm (p50) |

## What a pack holds; the format

Moved to chris-mapper's `docs/PACK.md` (`chris-mapper/docs/PACK.md`, lane L43): the roads, attributes, turn lanes,
built-up and forest flags and point features a pack holds, and the `.rsmap` file format (versions 1 and 2).
The reader is `chris_mapper/rsmap.py`; the builder is this repository's `tools/roadsense_maps.py`.
