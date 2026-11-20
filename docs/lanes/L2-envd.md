# L2 envd: slow scene / environment analyser (<= 1 fps)

Lane opened 2026-10-26 17:00 CET (Fable). Question: can a zero-shot CLIP-type image encoder, run
at most once per second on the Poco F1 next to modeld + signd, tell the place (highway, city street,
residential, forest road, rural road, tunnel, parking lot, underground parking, ...) and the
conditions (weather: clear / overcast / rain / snow / fog; light: day / dusk-dawn / night) from the
forward camera, stably, with per-class scores and a confidence? If zero-shot is not good enough on
some classes, does a linear probe on frozen embeddings fix them?

Phone state during this lane: the user's thermal stress test (`rs-stress-signd` + `rs-stress-guard`)
is active on top of the stack (die 80-84 C at 17:03 CET), so no phone timing in this lane: laptop
numbers here, phone measurements as an order (`orders/L2-envd-phone.md`). Times CET (Warsaw local; DST ended 2026-10-25).

## 0. Pre-registration (written before any scoring, 17:15 CET)

### Candidate encoders (image side only runs on the phone; text side precomputed on the laptop)
| tag | model (open_clip) | weights licence | image GFLOPs (paper) | IN-1k zero-shot |
|---|---|---|---|---|
| vitb32_openai | ViT-B-32 / openai | MIT | 4.4 | 63.2 |
| vitb32_laion2b | ViT-B-32 / laion2b_s34b_b79k | MIT (card: "deployed use out of scope" is a recommendation, not a licence term) | 4.4 | 66.6 |
| siglip2_b32_256 | ViT-B-32-SigLIP2-256 / webli | Apache-2.0 | ~5.7 | ~74 |
| tinyclip40m | TinyCLIP-ViT-40M-32-Text-19M (LAION-400M) | MIT (Microsoft) | 3.5 (MACs, their table) | 59.8 |
| tinyclip45m | TinyCLIP-auto-ViT-45M-32-Text-18M (LAION-400M) | MIT | 1.9-3.7 (table inconsistent) | 61.4 |
| tinyclip22m | TinyCLIP-auto-ViT-22M-32-Text-10M (LAION-400M) | MIT | 1.9 | 53.7 |

Excluded on licence: MobileCLIP / MobileCLIP2 (Apple ML Research Model License: research purposes
only, "does not include commercial exploitation, product development or use in any commercial
product"); not MIT-compatible, so not evaluated either.

### Datasets (evaluation only; nothing from them is shipped except, if trained, a probe that is NOT committed)
| dataset | what | labels used | licence / access |
|---|---|---|---|
| BDD100K val images (10 000, 1280x720, `<datasets>/dashcam/bdd100k_val_fo`, mirror `dgural/bdd100k` on HF, downloaded 2026-10-19 without an account) | dashcam frames, USA | weather (clear 5346, overcast 1239, partly cloudy 738, rainy 738, snowy 769, foggy 13, undefined 1157), scene (city street 6112, highway 2499, residential 1253, parking lot 49, tunnel 27, gas stations 7, undefined 53), timeofday (daytime 5258, night 3929, dawn/dusk 778, undefined 35) | BDD100K licence: educational / research / not-for-profit use |
| DAWN (`<datasets>/dashcam/dawn`, Fog 302, Rain 202, Snow 206; Sand not downloaded) | traffic scenes in adverse weather | weather | CC BY-NC 3.0 (Mendeley Data 766ygrbt8y) |
| Places365-Standard val_256 (`<datasets>/places365`, downloaded 2026-10-26 from data.csail.mit.edu without a form; the site asks for a form for research use of the legacy set, noted) | scene photos, NOT dashcam views; proxy for the classes BDD lacks | forest_road, field_road (-> rural road), highway, street, residential_neighborhood, parking_garage-indoor (-> underground parking), parking_lot, tunnel-ish classes if present, bridge, gas_station (100 images each) | research use |
| ACDC, Dark Zurich | fog/rain/snow/night dashcam | not used: registration required (never create accounts) | - |

### Classes scored (exactly these; the daemon publishes the same names)
- place: highway, city_street, residential, rural_road, forest_road, tunnel, parking_lot, underground_parking, gas_station, bridge
- weather: clear, overcast, rain, snow, fog
- light: day, dusk_dawn, night

Label mapping (fixed now): BDD scene {highway, city street, residential, parking lot, tunnel, gas
stations} -> {highway, city_street, residential, parking_lot, tunnel, gas_station}; BDD weather
{clear, overcast, rainy, snowy, foggy} -> {clear, overcast, rain, snow, fog}, BDD "partly cloudy" is
reported separately (prediction split) and excluded from the headline, "undefined" excluded;
BDD timeofday {daytime, dawn/dusk, night} -> {day, dusk_dawn, night}; DAWN {Fog, Rain, Snow} ->
{fog, rain, snow}; Places365 {highway, street, residential_neighborhood, forest_road, field_road,
parking_lot, parking_garage-indoor, gas_station, bridge} -> {highway, city_street, residential,
forest_road, rural_road, parking_lot, underground_parking, gas_station, bridge}.

### Metric
Per class: recall = correct / images of that class (within its group: place, weather or light; a
prediction is the arg-max over that group's classes, with the full class list of the group as
candidates, including classes the dataset has no examples of, so e.g. "bridge" or "underground
parking" can steal BDD images and that counts as an error). Headline per group: macro-average of
the per-class recall over the classes the dataset has >= 20 images of (BDD foggy 13, gas stations 7
are reported but not in the macro). Also the overall (micro) accuracy and the confusion rows.
Zero-shot: prompt ensemble per class (fixed in `tools/envd/prompts.py` before scoring; any later
prompt change is a disclosed iteration with the before/after numbers both kept in this report).
Encoder selection = the best macro average over (BDD weather, BDD light, BDD scene, DAWN weather)
with the cost (GFLOPs, laptop ms) as the tie-breaker; the selection is on the full BDD val set, so
the probe numbers below use a held-out half.

### Linear probe (only if zero-shot misses: a class under 70 % recall)
Split BDD val by the sha1 of the file name (even -> A = train, odd -> B = test, ~5000 each; one
image per video in BDD, so no video leaks across the split). Multinomial logistic regression on the
frozen L2-normalised embedding per group, C chosen on A by 5-fold CV. Report zero-shot and probe
on B only, same metric. Probe weights come from BDD (research licence): not committed; the training
script is.

### Preprocessing disclosure
Candidate selection uses each model's own transform (resize short side, centre crop). The daemon
squashes the whole 16:9 frame to the model's square input (no crop; the sides carry the scene),
so the chosen model is re-scored with the squash preprocessing (`--preproc squash`) and those are
the numbers that stand for the shipped path.

### Stability (temporal smoothing) check
No labelled dashcam video is available without registration, so the smoother is checked on
synthetic sequences: per-frame zero-shot scores of BDD images of one class in runs of 30 "seconds"
(one image per second), switching class between runs, plus runs with 20 % of the frames replaced by
images of another class (noise). Reported: switch latency (seconds from the true change to the
reported change) and flicker count (reported changes beyond the true ones) for the chosen window and
hysteresis, versus the raw per-frame arg-max.

## 1. Implementation (envd/, tools/envd/, 17:2x-17:5x CET)

`envd` (C++17, `envd/`): ring reader at `--max-hz` (default 1, newest frame), `prep.cc` box-downscales
the whole NV12 frame to the encoder's square input in one pass (integer block bounds, block mean
converted to RGB afterwards; cheap statistics from the same pass: mean luma, top/bottom third
luma, dark/bright fractions, chroma), `encoder.h` runners (`ort_encoder.cc` ONNX Runtime CPU,
`ncnn_encoder.cc` ncnn/Vulkan behind `-DROADSENSE_NCNN=ON`, same link recipe as modeld),
`pipeline.cc` (cosine against the class text embeddings x logit scale, softmax per group),
`smoother.h` (window mean + hysteresis + minimum hold), `output.cc` (`env` JSON), `main.cc`
(options, `envd.json`, little cores, nice 5, thermal pause above 92 C, greeting with the last
message). `models/envd-classes.json` = class text embeddings + preprocessing constants from
`tools/envd/text_embeddings.py` (prompt ensemble in `tools/envd/prompts.py`), the image tower from
`tools/envd/export_onnx.py` (`scripts/fetch-envd-model.sh` does both). `tools/envd_replay.cc` runs
the same pipeline on JPEGs or a raw NV12 sequence. api: `envd` socket, `env` topic, `/v1/env`,
`/ws` topic, `/v1/health` (`env_ms`, labels), status page block; `tools/fake_daemons.py` emits a
fake `env`; `api/tests` extended (11 tests pass: `PYTHON=.venv-models/bin/python tools/smoke_api.sh`,
17:26 CET). Unit `systemd/roadsense-envd.service`, wanted by `roadsense.target`;
`scripts/install-phone.sh` installs the binary, the models and the unit.

Laptop build (no system cmake / libjpeg-dev; the ORT 1.22 tarball and the unpacked libjpeg-dev of
the L1 scratch were reused): `~/.pico-sdk/venv/bin/cmake -S envd -B build-envd -DONNXRUNTIME_ROOT=
<onnxruntime-linux-x64-1.22.0> -DROADSENSE_NATIVE=OFF "-DJPEG_INCLUDE_DIR=<jpeg>/usr/include;<jpeg>/usr/
include/x86_64-linux-gnu" -DJPEG_LIBRARY=/usr/lib/x86_64-linux-gnu/libjpeg.so.8`.

### Parity of the shipped path with the Python evaluation (17:4x CET)
`tools/envd/embed_dataset.py --preproc squash` is a numpy replica of `envd/prep.cc`. 20 BDD images,
ViT-B-32/openai, `envd_replay --reset-each --emb` vs the replica (cosine of the embeddings):
- raw NV12 input (the ring path; NV12 written by Python from the PIL decode): **min 1.00000, mean 1.00000**
  (`envd_replay --nv12 1280x720 parity20.nv12`): preprocessing + ORT are exact.
- JPEG input decoded by envd_replay's libjpeg path: min 0.977, mean 0.993: the two JPEG decoders
  (PIL vs libjpeg RGB output) differ, not the pipeline. Only affects `envd_replay` on JPEG files.
- For scale: the model's own transform (resize short side + centre crop) vs the squash gives
  cos 0.86-0.95 (mean 0.92) on the same images, so the squash numbers below are the ones that count.

### End-to-end on the laptop (17:5x CET)
`tools/ring_replay.py --nv12 1920x1080 ljkr.nv12 --fps 20 --loop --ring /tmp/roadsense-ring-l2` (a
fake camerad: the 120-frame Ljubljana motorway clip from the phone's `~/rs-replay`, 6 s looped),
`envd --threads 4 --cpus none --max-hz 1 --pause-temp 0 --verbose` and the api on port 18091:
envd takes one frame per second (seq 48, 68, 88, ...: every 20th frame), 109-197 ms per frame
(encode 98-184 ms with the extraction jobs loading the laptop), says highway 56-67 % / clear 96 % /
day 77-92 % with `stable_s` growing; `GET /v1/env` returns the message, `/v1/health` shows `env_ms`
and the three labels. Null found on the way: with the laptop package at 97 C the default
`--pause-temp 92000` silently paused envd (that is the phone's guard); it now logs the pause and
the resume.

## 1b. Cost on the laptop (i7-8850H, ORT 1.22 CPU, `envd_replay --nv12 1280x720 parity20.nv12`, 20 frames, 17:5x CET, with a 4-thread extraction job running alongside)
| image tower | ONNX size | 2 threads: total / encode median ms | 4 threads: total / encode median ms |
|---|---|---|---|
| **ViT-B-32/openai** (shipped) | 352 MB | 93.0 / 88.7 | 46.5 / 43.1 |
| ViT-B-32-SigLIP2-256 | 378 MB | 114.4 / 110.9 | 55.1 / 52.0 |
| TinyCLIP-ViT-40M-32 | 159 MB | 33.2 / 30.1 | 25.9 / 22.5 |

Preprocessing (box squash of 1280x720 to 224x224 + stats) is ~4 ms on the laptop. TinyCLIP-40M is
3x cheaper than ViT-B/32 (its MACs table lists image + text), at -4 points of zero-shot macro; if the
phone's A55s turn out too slow for ViT-B/32 at 1 fps (expected 0.5-1 s with 2 threads, i.e. right
at the period), the order has the big-core and TinyCLIP rows as fallbacks.

## 2. Zero-shot results (model transform = resize short side + centre crop; candidate selection)

Command per cell: `tools/envd/zeroshot_eval.py --emb <cache>/<ds>_<tag>.npz --classes <cache>/classes_<tag>.json
--group G --label-col COL --dataset DS` on the embeddings of `tools/envd/embed_dataset.py` (job
`<datasets>/envd-cache/extract_all.sh`, laptop CPU, 8 threads, ~21 img/s for ViT-B/32). Recall in %,
macro over classes with n >= 20 (BDD foggy n=13 and gas stations n=7 shown, not in the macro).

TinyCLIP "auto" variants (22M, 45M) dropped: their checkpoints are weight-inheritance pruned and
need TinyCLIP's masked modules (`mask_image` config key unknown to upstream open_clip 3.3.0);
only the plain TinyCLIP-ViT-40M-32 loads.

### ViT-B-32 / openai (MIT)
| BDD weather | n | recall | predicted as |
|---|---|---|---|
| clear | 5346 | 91.8 | clear 92, rain 4, fog 2 |
| overcast | 1239 | 31.7 | clear 64, overcast 32 |
| rain | 738 | 69.8 | rain 70, clear 18, overcast 9 |
| snow | 769 | 30.0 | clear 59, snow 30, rain 8 |
| fog | 13 | 15.4 | clear 62, rain 15, fog 15 |
| macro | | **55.8** | micro 74.6; partly cloudy (738): clear 96, overcast 3 |

| BDD light | n | recall | predicted as |
|---|---|---|---|
| day | 5258 | 96.0 | day 96, dusk_dawn 3 |
| dusk_dawn | 778 | 45.8 | day 52, dusk_dawn 46 |
| night | 3929 | 98.0 | night 98 |
| macro | | **79.9** | micro 92.9 |

| BDD place | n | recall | predicted as |
|---|---|---|---|
| highway | 2499 | 46.3 | highway 46, bridge 13, city_street 12, parking_lot 10 |
| city_street | 6112 | 60.4 | city_street 60, parking_lot 19, residential 9, highway 5 |
| residential | 1253 | 51.1 | residential 51, city_street 24, parking_lot 21 |
| tunnel | 27 | 85.2 | tunnel 85, bridge 7 |
| parking_lot | 49 | 87.8 | parking_lot 88, underground_parking 8 |
| gas_station | 7 | 42.9 | parking_lot 57, gas_station 43 |
| macro | | **66.1** | micro 55.8 |

| DAWN weather | n | recall | predicted as |
|---|---|---|---|
| rain | 200 | 66.5 | rain 66, fog 26, overcast 5 |
| snow | 204 | 98.5 | snow 99 |
| fog | 300 | 82.3 | fog 82, rain 14 |
| macro | | **82.5** | |

### ViT-B-32 / laion2b_s34b_b79k (MIT)
BDD weather macro 44.7 (overcast 12.0, rain 50.0, snow 25.7, clear 91.2); BDD light macro 69.5
(dusk_dawn 10.7); BDD place macro 65.1 (highway 51.2, city 63.6, residential 56.4, tunnel 88.9,
parking lot 65.3); DAWN macro 82.3 (rain 66, snow 99, fog 82). Worse than openai on everything
that is hard (overcast, snow, dusk), so dropped.

### ViT-B-32-SigLIP2-256 / webli (Apache-2.0; 256 px input, 768-dim; needs `transformers` for its tokenizer)
BDD weather macro 46.2 (clear 66.7: 29 % called overcast; overcast 41.5, rain 39.3, snow 37.2);
BDD light macro 74.0 (dusk_dawn 24.7); BDD place macro **72.5** (highway 61.1, city 50.9,
residential 79.8, tunnel 88.9, parking lot 81.6, gas station 6/7); DAWN macro 79.5 (rain 56.5,
snow 99.0, fog 83.0). Better than openai on place, clearly worse on weather and light. Anchored
probe (L2 0.1, A -> B): weather 76.5, light 85.3, place 73.7: the same as openai's probe within a
point, at 1.3x the input pixels and 1.5x the embedding size.

### TinyCLIP-ViT-40M-32-Text-19M (MIT; logit scale 50)
BDD weather macro 48.5 (overcast 14.4, rain 53.0, snow 41.2, clear 85.5); BDD light macro 73.7
(dusk_dawn 24.3); BDD place macro 64.6 (highway 57.3, city 52.8, residential 60.1, tunnel 81.5,
parking lot 71.4); DAWN macro 81.3 (rain 56.5, snow 98.5, fog 89.0).

### Places365 val subset (900 scene photos, 100 per class; NOT dashcam views, a proxy for the classes BDD lacks)
| place (ViT-B-32/openai) | recall | predicted as |
|---|---|---|
| highway | 48.0 | rural_road 26, forest_road 8, bridge 7 (Places "highway" is often a side/aerial view) |
| city_street | 82.0 | residential 9 |
| residential | 87.0 | city_street 8 |
| rural_road | 66.0 | forest_road 17, bridge 5 |
| forest_road | 73.0 | rural_road 12, highway 5, bridge 5 |
| parking_lot | 69.0 | residential 13, gas_station 8 |
| underground_parking | 73.0 | gas_station 12, parking_lot 8, **tunnel 3** |
| gas_station | 92.0 | |
| bridge | 93.0 | |
| macro | **75.9** | laion2b 77.3 (underground parking 88, parking lot 78, highway 35); SigLIP2 77.2 (underground parking 78, forest 79, highway 46); TinyCLIP-40M 75.9 (underground parking 85, highway 35) |

Tunnel vs underground parking, the confusion the brief asked about: BDD tunnels (27) go to
underground parking 0 % (openai) / 4 % (laion2b, siglip2); Places underground garages go to
tunnel 3 %. The real confusion of underground parking is with gas station (canopy + pillars + cars,
12 %) and the outdoor parking lot (8 %).

### Selection (pre-registered rule: mean of the four macros, cost as tie-breaker)
| encoder | BDD weather | BDD light | BDD place | DAWN | mean | licence | input / dim |
|---|---|---|---|---|---|---|---|
| **ViT-B-32 / openai** | 55.8 | 79.9 | 66.1 | 82.5 | **71.1** | MIT | 224 / 512 |
| ViT-B-32-SigLIP2-256 | 46.2 | 74.0 | 72.5 | 79.5 | 68.1 | Apache-2.0 | 256 / 768 |
| TinyCLIP-40M/32 | 48.5 | 73.7 | 64.6 | 81.3 | 67.0 | MIT | 224 / 512 |
| ViT-B-32 / laion2b | 44.7 | 69.5 | 65.1 | 82.3 | 65.4 | MIT | 224 / 512 |

Decision: **ViT-B-32 / openai** (best zero-shot mean, best on the three weak groups weather / light,
equal to SigLIP2 after the probe, smallest input of the two, MIT). TinyCLIP-40M would save ~20 %
of the encoder time for 4 points of macro; not worth it at 1 fps. Everything below is on this
encoder with the squash preprocessing of the daemon.

### Reading (first two encoders)
Zero-shot misses the 70 % bar on overcast (called clear), snow (BDD "snowy" = snow on the ground
under a bright sky: called clear), dusk/dawn (called day), highway / city / residential (confused
among themselves, and "parking lot" steals 10-21 % of city/residential frames); it is good on night
(98), day (96), clear (92), tunnel (85-89), parking lot (65-88) and the DAWN adverse weather (fog 82,
snow 99). So the probe is on (pre-registered trigger).

### Probe, first look (ViT-B-32/openai, model transform, split A -> B; `tools/envd/train_probe.py`)
Two probe variants, both without bias (envd scores `logit_scale * emb . w_c`):
plain (W from 0, L2 to 0) and, added after seeing the plain one, `--init-zeroshot` (W starts at the
prompt embeddings and the L2 pulls towards them, so classes the data lacks - forest, rural,
underground parking, bridge, gas station on BDD - keep their zero-shot rows instead of being
driven to "never"). Disclosed: the zero-shot-anchored variant was not in the pre-registration.

| group | zero-shot macro (B) | plain probe, best L2 | anchored probe L2=0.1 | per class (anchored) |
|---|---|---|---|---|
| weather | 56.4 | 75.1 (L2 1e-2) | **77.4** | clear 82, overcast 82, rain 72, snow 74, fog (n=9) 11 |
| light | 81.0 | 84.5 (1e-2) | **86.6** | day 86, dusk_dawn 77, night 98 |
| place | 60.8 | 67.5 (1e-2) | **73.4** | highway 80, city 69, residential 76, tunnel (n=10) 70, parking lot (n=26) 69 |

The probe trades a few points of day/clear for the minority classes (class-balanced loss). The
remaining confusions are city vs residential (BDD's own labels are noisy here) and dusk vs day.
Final probe numbers are re-done on the chosen encoder with the squash preprocessing (section 4).

## 3. Final numbers: ViT-B-32/openai, squash preprocessing (the shipped path), zero-shot class file
`<datasets>/envd-cache/final_eval.sh` (18:1x CET): `embed_dataset.py --preproc squash` for BDD val,
DAWN, Places subset, then `zeroshot_eval.py --md`. The committed `models/envd-classes.json`
(sha256 5001c370...) is this file.

| group / set | class | n | recall | predicted as |
|---|---|---|---|---|
| weather, BDD | clear | 5346 | 81.1 | rain 15, fog 2 |
| | overcast | 1239 | 34.8 | clear 58, rain 6 |
| | rain | 738 | 76.3 | clear 14, overcast 7 |
| | snow | 769 | 30.0 | clear 54, rain 13 |
| | fog | 13 | 7.7 | clear 54, rain 38 |
| | **macro** | | **55.6** | micro 68.6; partly cloudy (738): clear 95 |
| light, BDD | day | 5258 | 97.3 | dusk_dawn 2 |
| | dusk_dawn | 778 | 37.8 | day 59 |
| | night | 3929 | 97.8 | |
| | **macro** | | **77.6** | micro 92.9 |
| place, BDD | highway | 2499 | 34.7 | city_street 19, tunnel 17, bridge 16 |
| | city_street | 6112 | 74.0 | residential 9, parking_lot 7 |
| | residential | 1253 | 55.9 | city_street 34, parking_lot 7 |
| | tunnel | 27 | 85.2 | bridge 7, parking_lot 4 |
| | parking_lot | 49 | 77.6 | underground_parking 12, gas_station 6 |
| | gas_station | 7 | 85.7 | city_street 14 |
| | **macro** | | **65.5** | micro 61.9 |
| weather, DAWN | rain | 200 | 70.5 | fog 22 |
| | snow | 204 | 99.0 | |
| | fog | 300 | 81.7 | rain 13, snow 4 |
| | **macro** | | **83.7** | |
| place, Places365 | highway | 100 | 49.0 | rural_road 24, forest_road 8, bridge 7 |
| | city_street | 100 | 81.0 | residential 8, parking_lot 6 |
| | residential | 100 | 83.0 | city_street 8 |
| | rural_road | 100 | 66.0 | forest_road 18 |
| | forest_road | 100 | 74.0 | rural_road 10, bridge 6, highway 5 |
| | parking_lot | 100 | 68.0 | residential 13, gas_station 8 |
| | underground_parking | 100 | 73.0 | gas_station 11, parking_lot 10, tunnel 3 |
| | gas_station | 100 | 94.0 | |
| | bridge | 100 | 95.0 | |
| | **macro** | | **75.9** | |

Versus the model's own transform (section 2): the squash costs highway on BDD (46 -> 35: the
whole wide frame with sky and verge reads as "tunnel"/"bridge" more often) and gains city street
(60 -> 74) and rain (70 -> 76); light and the other sets move by < 3 points.

## 4. Linear probe on the shipped path (`tools/envd/train_probe.py`, anchored, L2 0.1)
Training = split A of BDD val plus split A of DAWN (weather) / of the Places subset (place), test =
the B halves (hash of the file name; disclosed change from the pre-registration: the extra sets
are split too, after the first attempt - BDD-only training - drove the Places-only classes to 0-12 %
recall on Places: rural road 0, forest road 12, underground parking 1, bridge 11; the anchored L2
alone did not protect them once highway's row had moved). Light has BDD only.
Output: `<datasets>/envd-cache/envd-classes-probe.json` (sha256 8d433489..., NOT committed:
BDD / DAWN / Places are research-licensed; drop-in for `--classes` or `classes` in `envd.json`).

| group | test set | zero-shot macro | probe macro | probe per class |
|---|---|---|---|---|
| weather | BDD B (4105) | 56.9 | **78.9** | clear 82.9, overcast 82.8, rain 72.9, snow 77.0, fog (n=9) 11 |
| weather | DAWN B (376) | 83.1 | 80.6 | rain 56.1 (fog 39), snow 95.2, fog 90.5 |
| light | BDD B (5016) | 78.2 | **86.2** | day 86.7, dusk_dawn 74.5, night 97.4 |
| place | BDD B (5006) | 58.3 | **74.0** | highway 78.4, city_street 69.2, residential 75.5, tunnel (n=10) 70, parking_lot (n=26) 73.1 |
| place | Places B (445) | 75.3 | 73.1 | highway 28.6 (Places' side views -> rural/forest), city 46.3, residential 88.0, rural_road 64.2, forest_road 82.6, parking_lot 65.8, underground_parking 97.9, gas_station 93.5, bridge 90.9 |

Reading: the probe fixes the three failures (overcast 35 -> 83, snow 30 -> 77, dusk/dawn 38 -> 75)
and the highway/residential confusion (35 -> 78, 56 -> 76) at a cost of a few points of the easy
classes (day 97 -> 87, clear 81 -> 83 is a gain here) and 2-3 points on the non-dashcam sets. What
is left: city vs residential (16-18 % each way; BDD's labels are themselves fuzzy there), rain vs
fog on DAWN, dusk vs day. Fog has 13 + 300 examples only in BDD + DAWN; with the probe the DAWN
fog recall is 90 %, BDD's 13 foggy frames stay mostly "clear" (they look like haze).

## 5. Stability (`tools/envd/stability_sim.py`, squash scores, probe class file, 12 runs x 30 s, 1 fps; smoother = window 10 s, hysteresis 0.10, hold 3 s)
| group | noise | raw arg-max accuracy / flips | smoothed accuracy | label changes (11 true) | latency s median (all) | extra flips |
|---|---|---|---|---|---|---|
| light | 0 % | 88.6 / 75 | 83.1 | 11 | 6 (4-7) | 0 |
| light | 20 % | 70.6 / 156 | 81.1 | 13 | 5 (2-10) | 2 |
| weather | 0 % | 78.3 / 137 | 83.6 | 11 | 5 (4-7) | 0 |
| weather | 20 % | 65.6 / 198 | 82.5 | 13 | 5 (3-8) | 2 |
| place | 0 % | 80.6 / 114 | 81.9 | 11 | 6 (3-8) | 0 |
| place | 20 % | 65.3 / 194 | 81.7 | 11 | 5 (3-10) | 0 |
| place, no smoothing (window 1, hysteresis 0, hold 0) | 20 % | - | 70.0 | 132 | 0 | 121 |

With the zero-shot file the smoother behaves the same on place (0-1 extra flips, latency 5-7 s) but
"misses" runs whose per-frame accuracy is below 50 % (overcast, snow, dusk runs never win: 3-8
missed changes), which is the per-frame problem of section 3, not the smoother's.
C++ check: the Python replica fed with envd_replay's per-frame scores of a 360-frame NV12 sequence
reproduces the C++ `label` on 360/360 frames (section 1).
Null: the smoothed accuracy with 0 % noise is a few points below the raw arg-max for light
(83 vs 89) because every true change costs ~6 s of lag out of each 30 s run; at 1 fps that is the
price of the 10 s window, and with 20 % wrong frames the smoother is +10-16 points ahead.

## 6. Answer
Does it work: **yes, with the probe**. Zero-shot ViT-B-32/openai alone gets night / day / clear /
tunnel / parking lot / gas station / bridge / fog (DAWN) / snow (DAWN) at 78-99 % and fails on
overcast (35), snow on the ground (30), dusk/dawn (38) and highway from the squashed wide frame (35);
the linear probe on frozen embeddings (trained on BDD + DAWN + Places A halves, ~10 s on the
laptop) brings every class with >= 20 held-out examples to 65-98 % except Places' "highway" side
views (29) and city street (46-69, fuzzy against residential), with weather macro 79, light 86,
place 74 on the BDD halves and 73-81 on the non-BDD halves. Labels are stable: 0-2 spurious changes
per 6 minutes with 20 % wrong frames, 5-6 s to follow a real change.
Cost: ViT-B/32 fp32, 89 ms per frame on 2 laptop threads (43 ms on 4); on the phone's A55s
expected at the 1 s period (order L2-envd-phone, rows A-C, TinyCLIP-40M fallback F at 1/3 of the
cost and -4 macro points). ncnn/Vulkan: the conversion (`tools/envd/make_ncnn.sh`, 169 layers,
MultiHeadAttention / LayerNorm / GELU all supported) is done but unverified: the x86 ncnn Python
wheel segfaults on this graph in every mode, so the GPU row of the order is conditional on a check
on the phone's ncnn build.

## 7. State at hand-back (18:2x CET)
- roadsense commits: 2c03711 (envd + tools + api + docs), 135d8f9 (thermal-pause log, ring_replay),
  08c4ba7 (class file, config), 645547d (final numbers); all pushed to origin.
- Phone: untouched (the stress test ran the whole lane; die 80-84 C). envd is not installed there;
  `orders/L2-envd-phone.md` covers build, install, cost rows A-F and a 30-min run.
- Laptop files outside the repo: `<datasets>/envd-cache/` (embeddings, class files incl.
  `envd-classes-probe.json`, `onnx/*.onnx`, `ncnn/vitb32_openai.*`, logs, `final_eval.out`),
  `<datasets>/places365/` (val_256.tar + the 900-image `sel/`), `<datasets>/roadsense-replay/ljkr.nv12(.xz)`,
  `models/envd-image.onnx` (ignored by git). `.venv-models` gained open_clip, torchvision, aiohttp,
  transformers, sentencepiece.
- Open: phone cost + 30-min run (order); ncnn verification on the phone; BDD "partly cloudy" is
  reported as clear (95 %), which is what the user's 5-class taxonomy implies; fog has too few
  dashcam examples to certify beyond DAWN; the probe's data licences keep it out of the repo
  (`scripts/fetch-envd-model.sh` reproduces the zero-shot file, `tools/envd/train_probe.py` the probe
  from the cache).
