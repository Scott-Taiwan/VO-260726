#!/bin/bash
# Launch vo_capture_nopx.py in the background: camera only, NO Pixhawk wait.
# Takes one photo every 2 s, 10 shots, then exits (about 20 s total).
# Keeps running even if the SSH session / terminal that started it closes.
# Output is shown live in this terminal AND saved under logs/.
#
# Usage:
#   ./run_vo_capture_nopx.sh                          # 10 shots, every 2 s
#   ./run_vo_capture_nopx.sh --shots 20 --interval 1  # any extra args override the defaults
#   ./run_vo_capture_nopx.sh --cam 0                  # use the camera on CAM0 (default CAM1)
#   ./run_vo_capture_nopx.sh --exposure-ms 1 3        # shutter range 1-3 ms (default 1-5)
#   ./run_vo_capture_nopx.sh --exposure-ms 2 2        # fixed 2 ms shutter (1/500 s)
#   ./run_vo_capture_nopx.sh --self-check             # rotate-the-camera check first
#   ./run_vo_capture_nopx.sh help                     # list every parameter
#
#   ./run_vo_capture_nopx.sh status                   # is it running? PID + log path
#   ./run_vo_capture_nopx.sh stop                     # stop it early
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PROGRAM="vo_capture_nopx.py"
DEFAULT_ARGS=(--interval 2 --shots 10)
PID_FILE="$SCRIPT_DIR/vo_capture_nopx.pid"
LOG_DIR="$SCRIPT_DIR/logs"

status() {
    if [ -f "$PID_FILE" ] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
        echo "Running (PID $(cat "$PID_FILE"))."
        echo "Log: $(cat "$PID_FILE.log")"
    else
        echo "Not running."
        [ -f "$PID_FILE.log" ] && echo "Last log: $(cat "$PID_FILE.log")"
        rm -f "$PID_FILE"
    fi
}

stop() {
    if [ -f "$PID_FILE" ] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
        PID="$(cat "$PID_FILE")"
        echo "Stopping PID $PID ..."
        # SIGINT, not SIGTERM: vo_capture_nopx.py only releases the camera
        # (cap.release()) on KeyboardInterrupt. A plain SIGTERM kills it
        # before that runs and can leave the CSI camera session wedged.
        kill -INT "$PID"
        # Poll for real exit -- a process stuck in a blocking native camera
        # read may not act on SIGINT.
        for i in $(seq 1 10); do
            if ! kill -0 "$PID" 2>/dev/null; then
                echo "Stopped."
                rm -f "$PID_FILE" "$PID_FILE.log"
                return 0
            fi
            sleep 1
        done
        echo "PID $PID did NOT exit within 10s of SIGINT -- likely stuck in a blocking"
        echo "native call (e.g. a wedged camera read). Escalating to kill -9 (camera"
        echo "will NOT be cleanly released; you may need 'sudo systemctl restart"
        echo "nvargus-daemon' or a reboot before the camera works again)."
        kill -9 "$PID" 2>/dev/null
        sleep 1
        if kill -0 "$PID" 2>/dev/null; then
            echo "ERROR: PID $PID survived kill -9 -- likely in an uninterruptible kernel"
            echo "wait (D state). Check 'ps -o stat -p $PID'."
            return 1
        fi
        echo "Force-killed."
        rm -f "$PID_FILE" "$PID_FILE.log"
        return 0
    else
        echo "Not running."
        rm -f "$PID_FILE"
    fi
}

case "${1:-}" in
    help)   python3 "$PROGRAM" --help; exit 0 ;;
    status) status; exit 0 ;;
    stop)   stop; exit 0 ;;
esac

if [ -f "$PID_FILE" ] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
    echo "Already running (PID $(cat "$PID_FILE")). Use '$0 stop' first, or '$0 status'."
    exit 1
fi

# Only one program can own the CSI camera at a time.
for pf in vo_flight_capture.pid vo_flight_capture_1s.pid vo_flight_capture_always.pid \
          vo_flight_capture_fix2.pid photo_capture.pid photo_capture_nopx.pid photo_capture_dualcam.pid; do
    if [ -f "$pf" ] && kill -0 "$(cat "$pf")" 2>/dev/null; then
        echo "ERROR: another capture program is running (PID $(cat "$pf"), $pf)."
        echo "Stop it first (e.g. ./run_vo_flight_capture.sh stop), then retry."
        exit 1
    fi
done

mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/vo_capture_nopx_$(date +%Y%m%d_%H%M%S).log"

# `> >(tee ...)` (not `| tee`) keeps $! as python3's own PID, not tee's, so
# stop/status target the right process. tee also mirrors output to this
# terminal while it's open; once the terminal closes, the log file still
# gets everything: tee ignores SIGHUP, and --output-error=warn makes it keep
# writing the log after the terminal write starts failing (instead of dying
# and taking python3 down with a broken pipe).
# set -m (job control): without it, bash starts `&` jobs in a script with
# SIGINT *ignored*, and Python inherits that -- so `stop`'s kill -INT would
# be silently dropped and never reach the KeyboardInterrupt/cap.release()
# cleanup. Job control also puts python3 in its own process group.
set -m
nohup python3 -u "$PROGRAM" "${DEFAULT_ARGS[@]}" "$@" \
    > >(trap '' HUP; tee -a --output-error=warn "$LOG_FILE") 2>&1 < /dev/null &
PID=$!
disown

echo "$PID" > "$PID_FILE"
echo "$LOG_FILE" > "$PID_FILE.log"

echo "Started $PROGRAM in the background (PID $PID)."
echo "Log:            $LOG_FILE"
echo "Follow output:  tail -f $LOG_FILE"
echo "Check status:   $0 status"
echo "Stop it early:  $0 stop"
