# Ways of working: roadsense

## Target
A robust in-car perception API on the reference phone (Poco F1): the USB DVR camera mounted
and calibrated, road + sign streams served and recorded, thermals stable with the user's fan.
Kill criteria per lane are stated in its brief.

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
| P52 (laptop) | builds, model conversion, replays | heavy jobs in `systemd-run --user -p MemoryMax=...` | sd_runs / other programmes' jobs |
| Poco F1 192.168.0.86 | on-device benchmarks and runs | stop anything that passes 95 C; no reboots unless needed | the flex/USB setup (user's hardware) |

Secrets: vault key `poco_user_password` via `dotinfra vault exec poco_user_password -- ssh ... sudo -S -p ""`.

## User rulings (dated)
- 2026-10-22: boot autostart of roadsense.target stays disabled (thermal); start by hand.
- 2026-10-23: lane orchestration adopted for roadsense; first lane L1 = signs on Vulkan (Fable).
- 2026-10-26: L2 = slow environment analyser (weather + place, <= 1 fps), built on Fable (user).
- 2026-10-26 ~18:40: thermal headroom is ample with the fan (CPU+GPU stress plateaus around 80 C per the user);
  optimise for SPEED and responsiveness, not for lower GPU/heat. Lane safety stop stays at 95 C.

## Preferences
- Times in CEST. Commits: add only own paths; co-author trailer per session reminder.
