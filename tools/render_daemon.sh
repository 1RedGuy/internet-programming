#!/usr/bin/env bash
# Render daemon: renders scenes listed in out/final_queue.txt one at a time
# (via tools/render_finals.py), marking each "done" in out/final_done.txt.
# Append a scene id to the queue file to schedule it; add a line "STOP" to exit
# when the queue is empty.  Resumable: just start it again.
#   setsid nohup tools/render_daemon.sh > out/render_daemon.log 2>&1 < /dev/null &
cd "$(dirname "$0")/.."
Q=out/final_queue.txt; D=out/final_done.txt
touch "$Q" "$D"
SHARDS=${SHARDS:-2}; THREADS=${THREADS:-2}; NICE=${NICE:-10}
while true; do
  next=""
  while read -r s; do
    [ -z "$s" ] && continue
    [ "$s" = "STOP" ] && continue
    grep -qx "$s" "$D" || { next="$s"; break; }
  done < "$Q"
  if [ -z "$next" ]; then
    grep -qx STOP "$Q" && { echo "$(date '+%F %T') queue empty, STOP"; exit 0; }
    sleep 60; continue
  fi
  echo "$(date '+%F %T') starting $next"
  python3 tools/render_finals.py "$next" --shards "$SHARDS" --threads "$THREADS" --nice "$NICE"
  # done only if the scene video exists
  [ -s "video/${next}_final.mp4" ] && echo "$next" >> "$D" || { echo "$(date '+%F %T') $next incomplete, retry in 5 min"; sleep 300; }
done
