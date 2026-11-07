#!/bin/bash
# Launch vo_flight_capture_always.py in the background, same pattern as
# run_vo_flight_capture.sh -- survives the SSH session / terminal closing.
# Unlike the original, vo_flight_capture_always.py does NOT wait for any
# takeoff altitude: it starts capturing as soon as the startup checks pass
# (Pixhawk connected, GPS confirmed, camera self-check PASS/FAIL shown).
#
# Usage:
#   ./run_vo_flight_capture_always.sh                       # defaults: /dev/ttyTHS1 @57600
#   ./run_vo_flight_capture_always.sh --interval 1            # any extra args pass straight through
#
#   ./run_vo_flight_capture_always.sh status                 # is it running? PID + log path
#   ./run_vo_flight_capture_always.sh stop                    # stop it early
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PID_FILE="$SCRIPT_DIR/vo_flight_capture_always.pid"
LOG_DIR="$SCRIPT_DIR/logs"

status() {
    if [ -f "$PID_FILE" ] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
        echo "Running (PID $(cat "$PID_FILE"))."
        echo "Log: $(cat "$PID_FILE.log")"
    else
        echo "Not running."
    fi
}

stop() {
    if [ -f "$PID_FILE" ] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
        PID="$(cat "$PID_FILE")"
        echo "Stopping PID $PID ..."
        # SIGINT, not SIGTERM: vo_flight_capture_always.py only cleans up the
        # camera (cap.release()) on KeyboardInterrupt. A plain SIGTERM kills
        # it before that runs and can leave the CSI camera session wedged.
        kill -INT "$PID"
        # SIGINT doesn't guarantee the process actually exits -- if it's
        # stuck inside a blocking native call (e.g. a wedged camera read
        # that never returns control to Python), the signal just sits
        # pending and the process keeps running despite us "stopping" it.
        # Poll for real exit instead of trusting the signal blindly.
        for i in $(seq 1 10); do
            if ! kill -0 "$PID" 2>/dev/null; then
                echo "Stopped."
                rm -f "$PID_FILE" "$PID_FILE.log"
                return 0
            fi
            sleep 1
        done
        echo "PID $PID did NOT exit within 10s of SIGINT -- it's likely stuck in a blocking"
        echo "native call (e.g. a wedged camera read) that isn't responding to signals."
        echo "Escalating to kill -9 (camera resources will NOT be cleanly released this way --"
        echo "a wedged capture session underneath it may still need 'sudo systemctl restart"
        echo "nvargus-daemon' or a reboot before the camera works again)."
        kill -9 "$PID" 2>/dev/null
        sleep 1
        if kill -0 "$PID" 2>/dev/null; then
            echo "ERROR: PID $PID survived kill -9 -- it's likely in an uninterruptible kernel"
            echo "wait (D state). Nothing more a script can do; check 'ps -o stat -p $PID'."
            return 1
        fi
        echo "Force-killed."
        rm -f "$PID_FILE" "$PID_FILE.log"
        return 0
    else
        echo "Not running."
    fi
}

case "${1:-}" in
    status) status; exit 0 ;;
    stop) stop; exit 0 ;;
esac

if [ -f "$PID_FILE" ] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
    echo "Already running (PID $(cat "$PID_FILE")). Use '$0 stop' first, or '$0 status'."
    exit 1
fi

mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/vo_flight_capture_always_$(date +%Y%m%d_%H%M%S).log"

# Output goes through `tee` to BOTH the log file and this terminal, live --
# needed so you actually see the GPS-confirmation and camera self-check
# (rotate-the-camera countdown, PASS/FAIL) prompts in time to react, not
# just by opening the log file afterward.
# `> >(tee ...)` (not `| tee`) is deliberate: it keeps $! as THIS script's
# own python3 PID, not tee's, so stop/status still target the right process.
nohup python3 -u vo_flight_capture_always.py --port /dev/ttyTHS1 --baud 57600 "$@" \
    > >(tee -a "$LOG_FILE") 2>&1 < /dev/null &
PID=$!
disown

echo "$PID" > "$PID_FILE"
echo "$LOG_FILE" > "$PID_FILE.log"

echo "Started vo_flight_capture_always.py in the background (PID $PID)."
echo "Log:            $LOG_FILE"
echo "Follow output:  tail -f $LOG_FILE"
echo "Check status:   $0 status"
echo "Stop it early:  $0 stop"
