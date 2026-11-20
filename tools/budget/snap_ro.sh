#!/bin/sh
# snap_ro.sh [SECONDS=40] : read-only thread budget of a running stack, no root, nothing written on the host.
# Two snapshots of /proc/<pid>/task/*/{stat,status} SECONDS apart for every process of the stack; the text
# goes to stdout and is turned into a table by snap_ro.py on the dev box:
#     ssh HOST sh -s 40 < tools/budget/snap_ro.sh > snap.txt && python3 tools/budget/snap_ro.py snap.txt
# (lane L48; tools/budget/measure.py is the root tool with GPU time, rail power and temperatures.)
S=${1:-40}
PAT=${SNAP_PATTERN:-"signd|modeld|envd|laned|camerad|chris_integrator|roadsense_api|cyberpunk_computer|hu-gnss|prius-bled"}
snap() {
  for p in $(pgrep -f "$PAT"); do
    for t in /proc/$p/task/*; do
      [ -r $t/stat ] || continue
      s=$(cat $t/stat 2>/dev/null); c=$(cat $t/comm 2>/dev/null | tr ' ' '_')
      set -- ${s##*) }
      echo "$p ${t##*/} $c $((${12}+${13})) ${37} $(awk '/ctxt_switches/{printf "%s ",$2}' $t/status 2>/dev/null)"
    done
  done
}
ta=$(date +%s.%N); A=$(snap); T1=$(cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | sort -n | tail -1)
sleep $S
tb=$(date +%s.%N); B=$(snap)
for p in $(echo "$B" | awk '{print $1}' | sort -u); do echo "PID $p $(tr '\0' ' ' < /proc/$p/cmdline | cut -c1-70)"; done
echo "T $ta $tb $(getconf CLK_TCK) $T1 $(cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | sort -n | tail -1)"
echo A; echo "$A"; echo B; echo "$B"
