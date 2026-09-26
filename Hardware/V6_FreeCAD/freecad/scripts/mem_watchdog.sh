#!/bin/zsh
# Kill the largest headless FreeCAD job if system free memory falls below the threshold, then exit
# (exiting notifies the orchestrator, which re-arms the watchdog).
THRESH=${1:-12}
while true; do
  free=$(memory_pressure 2>/dev/null | awk '/free percentage/ {gsub("%","",$NF); print $NF}')
  if [ -n "$free" ] && [ "$free" -lt "$THRESH" ]; then
    line=$(ps -Ao pid,rss,command | grep "[f]reecadcmd" | sort -k2 -nr | head -1)
    pid=$(echo "$line" | awk '{print $1}'); rss=$(echo "$line" | awk '{print int($2/1024)}')
    if [ -n "$pid" ]; then
      kill -9 "$pid"
      echo "$(date +%H:%M:%S) watchdog: free ${free}% < ${THRESH}% -> killed freecadcmd pid $pid (${rss} MB): $(echo "$line" | cut -c1-160)"
      exit 0
    fi
  fi
  sleep 3
done
