# Ways of working: roadsense

## Target
A robust in-car perception API on the reference phone (Poco F1): the USB DVR camera mounted
and calibrated, road + sign streams served and recorded, thermals stable with the user's fan.
Kill criteria per lane are stated in its brief.

## Guidelines (owner, 2026-11-18; apply to every lane)
1. Quality first. Better output quality outranks power, heat and cost. The phone may run at its maximum, but quality must not suffer when it throttles: load is shed in a stated order, measurement outputs last, and every shed step is reported.
2. Room for new features second. Where better quality is not feasible, anything that gives the same quality cheaper (GPU time, watts, big-core time) is wanted and adopted, also when no use for the freed budget is planned yet. Report the freed budget as a result of its own.
3. "Same quality" is shown, not assumed: measured after the trackers (accuracy, flicker, far range). Measurement quality is never traded for speed or savings.
4. Measurement topics and the display are separate. Latency and quality count on the measurement topics (future self-driving); they are never smoothed or delayed for a nicer picture. The display channel (`view`, MFD) is informational: smoothness and low cost win over lag there.
5. "Unknown" beats a confident wrong answer; held or coasted values are marked.
6. The API stays car-agnostic and consumer-agnostic; roadsense informs and never controls. Breaking changes are allowed at a major version as long as known consumers (the MFD) are kept connected.

## Record
- Notebook: NOTEBOOK.md (numbered folds; roadmap, facts, lanes). docs/ROADMAP.md stays the public status page.
- Lane reports: docs/lanes/<LANE>-<slug>.md (code lanes may also touch roadsense-signs).
- Orders (long phone runs, thermal soaks): orders/<ID>.md.
- Claims: lightweight; a number that steers a decision (latency, fps, accuracy vs reference,
  temperature plateau) must come with the command that reproduces it.

## Models
- Build / code / analysis lanes: Fable when the user asks for it (2026-10-23: user asked for the
  signs-on-Vulkan lane on Fable); otherwise Opus.
- Execution (long phone runs, soaks, babysitting): Opus.
- Max parallel lanes: 2; the phone is a single surface, only one lane touches it at a time.

## Compute
| Host | Use | Limits | Never touch |
|---|---|---|---|
| laptop | builds, model conversion, replays | heavy jobs in `systemd-run --user -p MemoryMax=...` | other programmes' jobs |
| Poco F1 (`user@phone`) | on-device benchmarks and runs | stop anything that passes 95 C; no reboots unless needed | the flex/USB setup (user's hardware) |

Secrets: root on the phone is `ssh user@phone 'sudo -S -p "" CMD'` with the password fed on stdin from a
secret store, never typed into a command line, a file or a lane report.

Placeholders in commands of lane reports and orders (the repository is public, so hosts and personal paths are
generalised): `user@phone` is the phone's ssh login (postmarketOS user `user`; `phone` is a host alias),
`<phone-home>` that user's home directory on the phone, `<datasets>` the laptop's data folder (rides, drive
recordings, model downloads; never committed), `<scratch>` a large scratch disk, `~` the laptop user's home.

## User rulings (dated)
- 2026-10-22: boot autostart of roadsense.target stays disabled (thermal); start by hand.
- 2026-10-23: lane orchestration adopted for roadsense; first lane L1 = signs on Vulkan (Fable).
- 2026-10-26: L2 = slow environment analyser (weather + place, <= 1 fps), built on Fable (user).
- 2026-10-26 ~18:40: thermal headroom is ample with the fan (CPU+GPU stress plateaus around 80 C per the user);
  optimise for SPEED and responsiveness, not for lower GPU/heat. Lane safety stop stays at 95 C.
- 2026-10-27: L11 "signs in effect" (generalised speed-limit engine, panel) on Fable (user).
- 2026-11-18: 2.0 campaign (refinement, refactoring, optimisation; quality first, breaking changes allowed, MFD stays connected): Fable approved for any lane of this campaign that benefits from it.
- 2026-11-19: direction after the running lanes are done (L24-S, L25, L26, the on-phone governor proof): build around the new openpilot model line (vision + policy; path, ego-motion, lead speed), as the base for the future self-driving track. Lane lines for drawing on the MFD come from a separate cheap analysis and are a nice-to-have, not critical. L24-M's finding stands as the starting point: the new model's own lane and lead outputs are worse than supercombo's after the trackers, so the move needs a lane source and a lead-stability fix of its own.

## Preferences
- Times in CEST. Commits: add only own paths; co-author trailer per session reminder.
