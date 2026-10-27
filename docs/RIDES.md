# Ride library (recorded Polish drives for replay)

Openly licensed forward-facing drives from Poland, cut to 3-15 min windows and prepared for
`camerad-replay` (`tools/make_ride.sh`: 1280x720, 20 fps, MJPEG q 4). Built by lane L9
([docs/lanes/L9-ride-library.md](lanes/L9-ride-library.md)); the first ride (`krakow-2024`) came from L5.
**Videos are never committed.** Each ride folder holds `<slug>.mjpeg`, `camera-replay.json` (focal, fps),
`calib.json` (pitch/yaw/roll/height), `SOURCE.txt` (title, author, licence, URL, window, changes) and,
on the laptop only, `download/` (the source cut), `calib-frames/` and 1-2 `overlay-*.jpg` from a laptop
modeld run.

* Laptop: `~/datasets/rides/<slug>/` &nbsp; Phone: `~/rides/<slug>/` (user@192.168.0.86)
* Play on the phone: `sudo sh scripts/use-replay.sh --enable /home/user/rides/<slug>/<slug>.mjpeg`
  (installs that ride's `camera-replay.json` + `calib.json`), back with `--disable`.
* Licence: every ride is **CC BY 3.0** (YouTube "Creative Commons Attribution licence (reuse allowed)",
  checked with yt-dlp's `license` field on 2026-10-27; `krakow-2024` via the licence-reviewed Commons copy).
  Private copying and testing are allowed; **attribution is required wherever frames are shown**
  (the line is in each `SOURCE.txt`): "<title>" by <author>, <URL>, CC BY 3.0, cut/scaled/re-encoded.
* Intrinsics are **estimates** (no ride comes with a calibration). Column "intr.":
  **M** = focal measured (lane dashes tracked at a known speed), **A** = focal from the lane-dash rhythm
  or a sibling ride of the same channel/camera, checked on the overlay, **G** = focal guessed from the
  camera type (HFOV), **R** = rough (no markings or curved road for the vanishing point). Pitch/yaw come
  from the road vanishing point, height from lane width (f-independent). All overlays were judged
  plausible (lane lines on the painted lines, path in lane, leads on the car ahead) unless noted.

## Catalogue

| slug | source (author) | conditions | road types | length | notable content | f px (intr.) | pitch / yaw / h | laptop / phone |
|---|---|---|---|---|---|---|---|---|
| krakow-2024 | Commons: City Driving 4K Kraków 2024 (Relaxing Roads 4K) | overcast, dusk light | city | 10:00 | trams, buses, Vistula bridge, big junctions, roadworks chevrons | 690 (G) | -0.085 / +0.015 / 1.10 | both |
| rural-zloczew | YouTube OGizusc1nJo (Road) | overcast summer day | rural single carriageway (DK/DW), villages, small town | 15:00 | village entries/exits (area signs), double centre lines, town square of Lututów | 690 (G) | -0.072 / -0.094 / 1.5 | both |
| forest-lubelskie | YouTube t2_y5OwUjDI (Road) | sunny spring, hard sun/shade | rural, long forest stretches, villages | 15:00 | forest road, dappled shadows, A-signs, 40 limit | 690 (G) | -0.072 / -0.012 / 1.65 | both |
| forest-vistula-spit | YouTube 0cNjnCKWSLU (Road) | overcast early spring | pine-forest road DW501, villages, resort town | 15:00 | D-42/D-43 area signs, pedestrian crossings, no-stopping, car park at the end | 690 (G, VP on a curve) | -0.051 / -0.120 / 1.65 | both |
| winter-snow-rural | YouTube im6WUD0RHrw (Road) | winter, snow; overcast | unmarked minor road covered in snow/slush (~10 min), village, junction | 15:00 | snow-covered road without markings, priority-road sign, signalised junction | 690 (R) | -0.051 / -0.094 / 1.6 | both |
| winter-fog | YouTube XDz6TElO21s (Road) | **dense daytime fog**, frost | expressway-class 2x2 | 6:26 | ~100-200 m visibility, rumble edge line | 690 (G) | -0.090 / 0.000 / 1.6 | both |
| a1-night | YouTube 4pBfbhg105M (Road) | **night**, dry | A1 motorway, lit junction then unlit | 10:00 | gantries, trucks, headlight glare | 690 (G) | -0.111 / -0.048 / 1.7 | both |
| a1-night-fog | YouTube 4pBfbhg105M (Road) | **night + fog** | A1 motorway, lit and unlit | 12:00 | fog halos around lamps, low contrast markings | 690 (G) | -0.054 / -0.007 / 1.7 | both |
| a1-s8-rain | YouTube J-OURk-kRjA (Unwinding Rides) | **steady rain**, day | A1 / S8 motorway-expressway into Warsaw | 15:00 | wipers across the view, spray, traffic jam, junction signs, lane-merge warning sign | 1100 (A) | -0.075 / +0.022 / 1.22 | both |
| s7-elblag-lowsun | YouTube CNlg1HRyekE (Slow TV Travel Cruise) | winter, **low sun / glare** | S7 expressway, exit, Elbląg town streets | 15:00 | low sun ahead, snow verges, traffic lights + tram tracks at the end; **burnt-in speed/GPS bar and the uploader's graphics** (arrows, road badges, flag) | 900 (**M**) | -0.050 / +0.044 / 1.24 | both |
| s1-roadworks | YouTube zVLoR4-qZkQ (Inżynieria Ruchu PL.) | winter sun, **low sun into the lens** | S1 under construction | 7:37 | **roadworks**: contraflow, yellow temporary lines, U-21 boards, chevrons, temporary 60/70 limits; camera low behind the dashboard | 800 (G, R) | 0.000 / +0.050 / 1.3 | both |
| tunnels-s3 | YouTube Nr4H9yJID_Y (Road) | overcast late winter | S3 expressway | 10:20 | **tunnels** TS-26 (2.3 km) / TS-23 both ways, portal light transitions, lane-control signals | 690 (G) | -0.033 / -0.069 / 1.5 | both |
| tunnel-pow-warsaw | YouTube hfbi83wEPFo (Awizonosz) | overcast December | S2 (POW) urban expressway | 4:55 | **2.3 km Ursynów tunnel**, VMS, junction gantries; channel captions (junction names, "Grudzień 2021") | 900 (G) | -0.094 / +0.168 / 1.16 | both |
| tunnel-gdansk | YouTube DYZJ2Pocpa0 (zabek1) | sunny spring | city expressway (Trasa Słowackiego) | 4:36 | **tunnel under the Martwa Wisła**, 50 limit, exit into bright sun; small dashcam text line | 800 (G) | -0.106 / +0.055 / 1.2 | both |
| tunnel-swinoujscie | YouTube KNgYZVarDn8 (FlyLokietek) | wet windscreen | town tunnel | 3:14 | **Świna tunnel** (curved), lane signals, 50 limit | 950 (G, R) | -0.090 / -0.046 / 1.14 | both |
| gdansk-night-snow | YouTube 4kAnzWjY6II (Road) | **night + falling snow** | snow-covered city streets, ring road | 12:00 | no visible markings in the city part, snowflakes on the glass, headlight glare; model path wanders in the snowy streets (hard case) | 690 (R) | -0.115 / -0.029 / 1.7 | both |
| krakow-night-tunnel | YouTube z9zcuptxpgc (Unwinding Rides) | **night** | unlit roads, roundabout, lit expressway | 15:00 | roundabout (C-12), covered section/tunnel, **VMS speed limits** 100/80, junction signs | 1000 (A) | -0.070 / -0.021 / 1.1 | both |
| katowice-rain | YouTube dCwYKNyQOps (Unwinding Rides) | **light rain**, overcast | city arterials (tram city), DK79 | 15:00 | buses, traffic lights with arrows, no-entry/WJAZD, **DTŚ tunnel** (~12:00), 70 sign in the tunnel; camera low behind the dashboard (bottom ~30 % dashboard) | 760 (G) | -0.026 / -0.066 / 1.05 | both |
| warsaw-storm | YouTube BJEgnjMDqb0 (astrokacyper) | **thunderstorm, heavy rain** | city (tram city) | 15:00 | wipers, streaked glass, flooded road, signalised junctions, red/yellow lights; MiVue text bottom-right | 600 (G) | -0.178 / -0.124 / 1.1 | both |
| warsaw-city | YouTube _AD1zhQSp5E (Unwinding Rides) | overcast late winter | city centre arterials | 15:00 | **trams** + wires, many signals incl. green arrows, pedestrian crossings, signalised roundabout under a flyover, taxis/buses | 1100 (A) | -0.086 / +0.015 / 1.1 | both |
| krakow-lights | YouTube KE4VmNhNkOw (Unwinding Rides) | overcast summer | city, big junctions | 15:00 | **traffic lights with arrow signals** and lane-arrow boards, trams, cyclists, police car, roadworks barriers; camera pitched up (horizon ~165 px below centre) | 1000 (A) | -0.164 / +0.013 / 1.06 | both |

21 rides, 4 h 06 min in total; MJPEG 20.4 GB (19.0 GiB; laptop and phone), plus 9.4 GB of source cuts on the
laptop only. Overlay checks: laptop modeld (ORT CPU, 2 threads) on ~240 frames per check point, two
points per ride.

## Coverage

| wanted | rides |
|---|---|
| motorway | a1-night, a1-night-fog, a1-s8-rain |
| expressway (S) | s7-elblag-lowsun, tunnels-s3, tunnel-pow-warsaw, s1-roadworks, krakow-night-tunnel, winter-fog |
| rural + area signs (D-42/D-43) | rural-zloczew, forest-vistula-spit, forest-lubelskie, winter-snow-rural |
| forest road | forest-lubelskie, forest-vistula-spit |
| small town | rural-zloczew (Lututów), forest-vistula-spit (Krynica Morska), s7-elblag-lowsun (Elbląg) |
| tunnel | tunnels-s3, tunnel-pow-warsaw, tunnel-gdansk, tunnel-swinoujscie, katowice-rain, krakow-night-tunnel |
| night | a1-night, a1-night-fog, gdansk-night-snow, krakow-night-tunnel |
| rain | a1-s8-rain, katowice-rain, warsaw-storm |
| snow / winter | winter-snow-rural, gdansk-night-snow, s7-elblag-lowsun (verges), s1-roadworks |
| fog | winter-fog (day), a1-night-fog (night) |
| low sun / glare | s7-elblag-lowsun, s1-roadworks, tunnel-gdansk (tunnel exit) |
| roadworks | s1-roadworks, krakow-lights (barriers), krakow-2024 (chevrons) |
| roundabouts | krakow-night-tunnel, warsaw-city (signalised), krakow-2024 |
| tram cities | warsaw-city, warsaw-storm, krakow-2024, krakow-lights, katowice-rain |
| traffic lights incl. green arrows | warsaw-city, krakow-lights, katowice-rain, warsaw-storm |
| speed limits (B-33) | s1-roadworks (60/70), krakow-night-tunnel (VMS 100/80), forest-lubelskie (40), tunnel-swinoujscie / tunnel-gdansk (50), katowice-rain (70) |

## Gaps (not found under an open licence)

* **Underground or multi-storey car park driving**: none. The only open candidate (Awizonosz, CC BY,
  Warsaw Plac Powstańców car park) is a handheld walk-through, not a drive.
* **Flashing yellow (S-1 in yellow-flashing mode)**: not identified in any window (not searched frame by
  frame; may appear in the city rides at night).
* **Daytime heavy snowfall at speed**: winter rides have snow on the ground; falling snow only at night
  (gdansk-night-snow). Unwinding Rides' snowfall/fog drives are under the standard YouTube licence.
* **Rain at night**: not covered (candidates were standard-licensed).
* **Level crossings, school zones, residential zones (D-40/B-43 zone signs)**: not deliberately
  covered; may occur in the rural/forest rides.
* **Native 20 fps image sequences from Mapillary**: Mapillary is CC BY-SA but needs an account token and
  its sequences are ~1-2 fps stills (useless for modeld, possible later for signs/envd).
* Sped-up (2x-4x) CC BY sign-chronicle drives exist (DSzumaher "Drive & Road Signs Chronicles") and could
  serve a sign-only test; not added.
