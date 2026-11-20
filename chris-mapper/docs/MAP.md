# The `map` topic of chris-mapper

The message the map box puts out, and how it is made. In: the integrator's `ego_state` (pose with uncertainty
and time); out: `map` (the box's contract: `README.md`). The text below was moved verbatim from chris-roadsense's
`docs/ARCHITECTURE.md`, section "mapd" (lane L43), where the map message was described while the legacy
daemon roadsense-mapd produced it; "mapd" in it names that daemon, `matcher.py` and `context.py` are
`chris_mapper/matcher.py` and `chris_mapper/context.py`, and `mapd/roadsense_mapd/limits/pl.json` is
`chris_mapper/limits/pl.json`. In the chris stack `map.position` repeats the pose the box was given
(`motion_source: "integrator"`, `input.pose: "ego_state"`): `state` is `good` while the estimator has an absolute
fix within its freshness window, `dead_reckoning` while it coasts, `unknown` when its dead reckoning ran out,
`no_source` before any fix; the box holds no position filter. The pack format: `docs/PACK.md`. Other paths in the moved text are those of the
chris-roadsense repository at the time.

**Map matching** (`matcher.py`): every 0.45 s the edges within 35..120 m are scored by distance, by the
heading against the edge bearing (one-way edges only in their direction), a class prior (service and
restricted roads lose a tie) and a transition term: no penalty for the current edge and for edges reachable
from it along the graph within the distance travelled, a penalty for everything else. A road that is not
connected to the current one wins only when it fits far better at once or clearly better over 250 m, so a
parallel road does not capture the match during a multipath excursion and a wrong match corrects itself. The class prior of
service, restricted and parking roads grows with speed (x1 at walking pace up to x4 from 10 m/s,
`class_speed`, `class_speed_mps`, `parking_prior`; lane L20: on the recorded drives the matcher put the car on
a service road in 20-35 % of the moving time while the frames showed a multi-lane road).
At a fork the continuation of the same road (same class and ref or name) is followed freely, a branch
(an exit ramp, a side street) has to be clearly nearer; the alternatives of the last fork stay reachable
for 150 m.
`match.confidence` is the winner's share among the candidate roads, scaled by how well the winner fits.
Since lane L20 it is the confidence of the **road**: candidate edges that are pieces of the matched road
(connected, same class and the same `ref` or `name`) are not rivals (`road_confidence`), so it no longer dips
where the road is split into edges at a side road (median while moving on the recorded drives 0.45 -> 0.56).

**Message** (2 per second, `--hz`; the last one is sent to a client when it connects):

```
"api": 1,
"state": "good", "available": true, "reason": null,
"pack": {"region":"poland","osm_timestamp":"2026-11-14T21:20:41Z","built":"...","format":2},      // null without a pack; format 1 packs have no turn lanes
"input": {"gnss": {"source":"headunit","fix":"3d","age_s":0.4,"hacc_m":4.0,"sats":14,"hdop":0.8,
                   "used":812,"ignored":0,"rejected":2,"resets":0,
                   "hacc_scale":1.0},                                                    // since L20: factor on the receiver's hacc                                // null before the first gnss message
          "motion": {"source":"vehicle|vision|none","tag":"car-can","age_s":0.03},
          "errors": 0, "last_error": null,
          "synthetic": false,                                   // the position was fed by input tagged synthetic since the last reset
          "resets": 0, "last_reset": null},                     // or {"reason":"...","age_s":12.3}: `reset` messages and source changes
"position": {"lat":52.1512345,"lon":21.0456789,"heading_deg":93.2,"heading_acc_deg":1.4,"speed_mps":22.31,
             "hacc_m":2.1,                 // RMS radial error of the filter, m
             "confidence":0.96,"source":"gnss|dead_reckoning","fix_age_s":0.4,"dr_distance_m":0.0,
             "motion_source":"vehicle","odometer_m":18234.5},                                      // null in no_source / unknown
"match": {"matched":true,"confidence":0.97,"ambiguous":false,"distance_m":1.8,"heading_diff_deg":0.6,
          "lat":..,"lon":..,"bearing_deg":92.6,"edge":"11bb1a2c:531","dir":1,"offset_m":212.4,"candidates":3},
"road": {"class":"primary",               // motorway, motorway_link, trunk, trunk_link, primary .. tertiary(_link), unclassified, residential, living_street, service
         "name":"Wrocławska","ref":"DK46","oneway":false,
         "lanes":{"total":2,"forward":1,"backward":1,"source":"tagged|tagged_total|assumed"},     // forward = the car's direction
         "turn_lanes":[["left"],["through"],["through","right"]],   // painted arrows of the own direction, lane by lane from the left ([] = a lane without one); null = not tagged (pack format 2, below)
         "tunnel":false,"bridge":false,"covered":false,"lit":true,                                 // lit: true / false / null (not tagged)
         "surface":"asphalt","roundabout":false,"link":false,"motorroad":false,"expressway":false,"dual_carriageway":false,
         "access_restricted":false,"toll":false,"parking":false,"forest":false,"layer":0,
         "built_up":{"value":true,"source":"tagged|landuse|class"}},
"speed_limit": {"limit_kmh":70,"source":"maxspeed|implicit",
                "rule":null,               // implicit: built_up, outside, outside_dual_2x2, residential_zone, expressway, expressway_single, motorway, link (null limit)
                "detail":"tagged|sign|zone" /* maxspeed */ or "built_up:tagged|landuse|class" /* implicit */,
                "confidence":0.9,"variable":false},
"ahead": {"horizon_m":1500.0,"path_confidence":0.62,                                               // along the most likely path
          "features":[{"kind":"traffic_signals","dist_m":230.0,"confidence":0.9}],                 // also crossing, crossing_signals, level_crossing, tram_crossing, stop, give_way, mini_roundabout, speed_bump, toll_booth, motorway_junction, speed_camera (format 2 packs)
          "junctions":[{"dist_m":412.0,"kind":"junction|roundabout|roundabout_exit|dead_end|no_way_on","branches":3,
                        "turn_deg":-2.0,           // turn of the most likely path there, positive = right
                        "minor":false,             // every side branch is a service road
                        "confidence":0.62}],
          "tunnel":{"in_tunnel":false,"entry_m":840.0,"length_m":2310.0,"exit_m":3150.0},          // in a tunnel: exit_m = distance to its end (the path is followed past the horizon for it)
          "built_up_change":{"dist_m":655.0,"to":false,"source":"landuse","confidence":0.4},
          "speed_limit_changes":[{"dist_m":655.0,"limit_kmh":90,"source":"implicit","rule":"outside","detail":"built_up:landuse","confidence":0.4}],
          "road_change":{"dist_m":1210.0,"class":"secondary","name":null,"ref":"414"},
          "shape":{"s":[0,20,..600],"x":[..],"y":[..],"curvature":[..]},   // road ahead beyond camera range; x forward, y right (modeld's frame), origin at the matched position; curvature 1/m, positive = bending right
          "curves":[{"start_m":300.0,"apex_m":380.0,"end_m":480.0,"min_radius_m":150.0,"direction":"left","angle_deg":-88.0}],   // radius below 500 m, at most 4
          "turn_lanes":{...},              // the first stretch with painted turn arrows within 400 m on the path, or null (below)
          "junction_view":{...},           // the first major junction or roundabout within 400 m on the path, or null (below)
          "geometry":{...}},               // the path, its side roads and the first roundabout as geometry (below)
"proc": {"step_ms":0.9,"cpu_pct":0.8,"feeders":1,"tile_loads":14}
```

**Junction view** (`ahead.junction_view`, lane L16; `context.junction_view`): what a consumer needs to draw
the next junction or roundabout as a picture, computed from the pack's graph (no lane-level data, no pack
rebuild). It is made for the first junction on the most likely path within 400 m that has at least one side
branch that is not a service road; junction nodes within 35 m of each other along the path are one junction
(the two carriageways of a crossing dual carriageway).

```
{"kind": "junction",
 "dist_m": 171.0, "confidence": 0.58,            // along the path to the (first) junction node; path confidence past it
 "control": "signals|give_way|stop" | null,      // from the point features on the path 60 m before .. 35 m behind the node
 "approach": {"class":"secondary","name":"..","ref":null,"lanes":2,"lanes_source":"tagged","oneway":true,
              "turn_lanes":[["left"],["through","right"]]},   // of the edge that enters the junction; null = not tagged
 "arms": [ {"angle_deg": -91.0,                  // direction of the arm, measured 25 m out, relative to the direction of travel into the junction: 0 = straight on, positive = right
            "class":"secondary","name":"1 Maja","ref":null,
            "exit": true,                        // can be driven into (false: a one-way street towards the junction)
            "entry": true,                       // traffic comes out of it
            "path": true,                        // the most likely path leaves here
            "minor": false,                      // a service road or restricted access
            "lanes": 1} ],                       // tagged lanes in the leaving direction, null when not tagged
 "ring": null}
{"kind": "roundabout", "dist_m": 52.0, ..., "control": "give_way",
 "arms": [ {"angle_deg": 124.0,                  // where the arm meets the ring, seen from its centre: 180 = where the car enters, positive = to the right
            ..., "approach": false,              // the arm the car comes from
            "exit_no": 1} ],                     // exits in driving order from the entry (service roads and the way back are not counted)
 "ring": {"radius_m": 8.2, "lanes": null, "exits": 3, "path_exit_no": 3,        // the exit the most likely path takes (same road number or name, else the direction of the approach)
          "exit_dist_m": 71.0,
          "on_ring": false, "car_angle_deg": null}}
```
The carriageways of one road that meet the ring within 45 degrees are one arm. While the car is on the ring
the view announced before the entry is repeated unchanged (`dist_m` 0, `on_ring` true, `car_angle_deg` = the
car's place on the ring in the same angles), so a picture does not turn under the driver; a position that is
first found on a ring gets a view with the car at 180 and no path exit. The view is a property of the most
likely path: with `confidence` near 0.5 the path, and therefore the highlighted arm, is a guess. Turn
restrictions are not in the pack. Cost: `look_ahead` with the view 0.53 ms on the laptop (Poland pack,
a town centre, 660 positions); not yet measured on the phone.

**Turn lanes** (`road.turn_lanes`, `ahead.turn_lanes`, lane L22; pack format 2, `docs/MAPS.md`): the painted
arrows of OpenStreetMap's `turn:lanes`, per direction of travel. `road.turn_lanes` is the matched edge's own
(the car is on the stretch). `ahead.turn_lanes` announces the first stretch on the most likely path:

```
{"dist_m": 250.0,            // where the stretch begins (0: the car is on it)
 "junction_m": 400.0,        // where it ends: the junction the arrows belong to
 "lanes": [["left"], ["through"], ["through","right"]], "count": 3,
 "path": [false, true, true],   // the lanes that lead where the most likely path goes (from its turn at that junction); null when the arrows do not say
 "turn_deg": -2.0, "source": "tagged", "confidence": 0.62}
```
Arrow words: `left`, `slight_left`, `sharp_left`, `through`, `right`, `slight_right`, `sharp_right`,
`reverse`, `merge_to_left`, `merge_to_right`. Turn lanes whose number contradicts the tagged lane count of the
edge are withheld (neither can then be trusted lane by lane). The arrows also count lanes: where `lanes` /
`lanes:forward` are not tagged, `road.lanes.forward` is the number of arrows (`source: "tagged"`).

**Road geometry** (`ahead.geometry`, lane L17; `context.geometry`): the road around the most likely path as
polylines, for a consumer that draws it. Frame of `shape`: x forward along the matched bearing, y right, metres,
origin at the matched position. The pack has no lane-level geometry: a road is its centre line and a lane
count. The api turns this into the car's frame and joins it with the camera (`view.road`, below); a consumer
that draws should read that, not this object.

```
{"path": {"s":[0,4,..200,220,..600], "x":[..], "y":[..]},     // the most likely path: every 4 m up to 200 m, every 20 m up to 600 m
 "lanes": [{"dist_m":0.0,"forward":1,"backward":1,"source":"tagged"}],   // lane counts along the path, one entry per change
 "branches": [ {"dist_m": 171.0,            // path distance of the node it leaves from (null: an arm of the ring beyond the path's exit)
                "pts": [[171.0,0.4],[171.3,6.4],..],   // from the node outwards, at 0, 6, 12, 20, 32, 45 m of the side road (or to its end)
                "class":"tertiary","minor":false,      // minor: a service road or restricted access
                "exit":true,"entry":true,"oneway":false,   // exit: can be driven into; entry: traffic comes out of it
                "lanes":2,"lanes_source":"assumed",
                "ring":false,               // an arm of the roundabout: then also
                "ring_m": 203.0,            //   its place along the ring as a path distance (driving order)
                "path": false} ],           //   the path itself uses this road (entry or exit)
 "ring": {"centre":[214.0,0.3],"radius_m":14.1,"lanes":null,"on_ring":false,
          "entry_m":200.0,"exit_m":244.0}}  // where the path enters and leaves it; entry_m null on the ring
```
Side roads are listed for the junction nodes of the path within 260 m (at most 20), the ring is the first
roundabout within that distance. On a ring the path takes the exit that was announced before the entry (the
most likely path of a position first found on a ring has no reason to leave it); the junction view keeps its
picture the same way.

Speed limit: the `maxspeed` tag of the matched direction when present, else the Polish statutory limit
(`mapd/roadsense_mapd/limits/pl.json`: built-up 50, living street 20, outside 90, dual carriageway with two
lanes per direction 100, expressway 120 / single carriageway 100, motorway 140), `source` says which. An
expressway is a way with `motorroad=yes` or a `highway=motorway` whose refs are all S-numbers (`road.expressway`). The
built-up state comes from a tag on the way (`maxspeed`, `zone:traffic`, `source:maxspeed` = `PL:urban` /
`PL:rural`), else from landuse polygons (inferred, confidence 0.5); `built_up.source` says which. The most
likely path follows, at every junction, the continuation with the same ref, name and class and the smallest
turn; `path_confidence` is the product of its choices, and every `ahead` entry carries the confidence of the
path up to it. Turn restrictions are not in the pack.
