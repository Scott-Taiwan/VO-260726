#!/bin/bash
# Launch vo_flight_capture.py in the background for a real test flight, so it
# keeps running even if the SSH session / terminal that started it closes.
#
# Usage:
#   ./run_vo_flight_capture.sh                        # defaults: /dev/ttyTHS1 @57600
#   ./run_vo_flight_capture.sh --takeoff-altitude-m 20 # any extra args pass straight through
#
#   ./run_vo_flight_capture.sh status                 # is it running? PID + log path
#   ./run_vo_flight_capture.sh stop                    # stop it early
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PID_FILE="$SCRIPT_DIR/vo_flight_capture.pid"
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
        # SIGINT, not SIGTERM: vo_flight_capture.py only cleans up the camera
        # (cap.release()) on KeyboardInterrupt. A plain SIGTERM kills it before
        # that runs and can leave the CSI camera session wedged (as happened
        # during testing when `timeout` SIGTERM'd it mid-capture).
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
LOG_FILE="$LOG_DIR/vo_flight_capture_$(date +%Y%m%d_%H%M%S).log"

# Output goes through `tee` to BOTH the log file and this terminal, live --
# not just the file. Needed because vo_flight_capture.py now does an
# interactive camera self-check at startup (prints a countdown and asks you
# to rotate the camera); a plain `> file` redirect means you'd only see that
# prompt by opening the log file after the fact, too late to react to it.
# `> >(tee ...)` (not `| tee`) is deliberate: it keeps $! as THIS script's
# own python3 PID, not tee's, so stop/status still target the right process.
nohup python3 -u vo_flight_capture.py --port /dev/ttyTHS1 --baud 57600 "$@" \
    > >(tee -a "$LOG_FILE") 2>&1 < /dev/null &
PID=$!
disown

echo "$PID" > "$PID_FILE"
echo "$LOG_FILE" > "$PID_FILE.log"

echo "Started vo_flight_capture.py in the background (PID $PID)."
echo "Log:            $LOG_FILE"
echo "Follow output:  tail -f $LOG_FILE"
echo "Check status:   $0 status"
echo "Stop it early:  $0 stop"
