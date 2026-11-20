#!/bin/bash
# Launch vo_flight_capture_fix2.py in the background for a real test flight, so
# it keeps running even if the SSH session / terminal that started it closes.
# (run_vo_flight_capture.sh is unchanged and still launches the older script.)
#
# Built-in defaults (the recommended settings for the next flight):
#   --port /dev/ttyTHS1 --baud 57600 --interval 1 --burst 3 --pick sharpness --save-burst
#   (TNR is already OFF by default inside fix2: --tnr-mode 0)
#
# Usage:
#   ./run_vo_flight_capture_fix2.sh                       # just the defaults above
#   ./run_vo_flight_capture_fix2.sh --interval 2          # any extra arg overrides its default
#   ./run_vo_flight_capture_fix2.sh --pick gyro --burst 5
#   ./run_vo_flight_capture_fix2.sh --cam 0               # use CAM0 (default is CAM1)
#   ./run_vo_flight_capture_fix2.sh --no-save-burst       # don't keep the non-chosen burst frames
#
#   ./run_vo_flight_capture_fix2.sh status                # is it running? PID + log path
#   ./run_vo_flight_capture_fix2.sh stop                  # stop it early (clean camera release)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PROGRAM="vo_flight_capture_fix2.py"
PID_FILE="$SCRIPT_DIR/vo_flight_capture_fix2.pid"
OLD_PID_FILE="$SCRIPT_DIR/vo_flight_capture.pid"   # used by run_vo_flight_capture.sh
LOG_DIR="$SCRIPT_DIR/logs"

DEFAULT_ARGS=(--port /dev/ttyTHS1 --baud 57600 --interval 1 --burst 3 --pick sharpness)
SAVE_BURST=1

is_running() {  # $1 = pid file
    [ -f "$1" ] && kill -0 "$(cat "$1")" 2>/dev/null
}

status() {
    if is_running "$PID_FILE"; then
        echo "Running (PID $(cat "$PID_FILE"))."
        echo "Log: $(cat "$PID_FILE.log")"
    else
        echo "Not running."
    fi
}

stop() {
    if is_running "$PID_FILE"; then
        PID="$(cat "$PID_FILE")"
        echo "Stopping PID $PID ..."
        # SIGINT, not SIGTERM: the program only releases the camera
        # (cap.release()) on KeyboardInterrupt; a SIGTERM can leave the CSI
        # camera session wedged.
        kill -INT "$PID"
        for i in $(seq 1 10); do
            if ! kill -0 "$PID" 2>/dev/null; then
                echo "Stopped."
                rm -f "$PID_FILE" "$PID_FILE.log"
                return 0
            fi
            sleep 1
        done
        echo "PID $PID did NOT exit within 10s of SIGINT (probably stuck in a blocking camera read)."
        echo "Escalating to kill -9 -- the camera may then need 'sudo systemctl restart nvargus-daemon'"
        echo "or a reboot before it works again."
        kill -9 "$PID" 2>/dev/null
        sleep 1
        if kill -0 "$PID" 2>/dev/null; then
            echo "ERROR: PID $PID survived kill -9 (uninterruptible D state?). Check 'ps -o stat -p $PID'."
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

if is_running "$PID_FILE"; then
    echo "Already running (PID $(cat "$PID_FILE")). Use '$0 stop' first, or '$0 status'."
    exit 1
fi
# Only one program can own the CSI camera and the Pixhawk serial port.
if is_running "$OLD_PID_FILE"; then
    echo "The OLD capture program is running (PID $(cat "$OLD_PID_FILE"), started by"
    echo "run_vo_flight_capture.sh) and owns the camera/serial port."
    echo "Stop it first:  ./run_vo_flight_capture.sh stop"
    exit 1
fi

# Pass-through args; --no-save-burst is handled here (fix2 itself only has --save-burst).
USER_ARGS=()
for a in "$@"; do
    if [ "$a" = "--no-save-burst" ]; then
        SAVE_BURST=0
    else
        USER_ARGS+=("$a")
    fi
done
ARGS=("${DEFAULT_ARGS[@]}")
if [ "$SAVE_BURST" = 1 ]; then
    ARGS+=(--save-burst)
fi
# User args go LAST: argparse keeps the last value given, so they override the defaults.
ARGS+=("${USER_ARGS[@]+"${USER_ARGS[@]}"}")

mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/vo_flight_capture_fix2_$(date +%Y%m%d_%H%M%S).log"
echo "command: python3 -u $PROGRAM ${ARGS[*]}" > "$LOG_FILE"

# Output goes through `tee` to BOTH the log file and this terminal (the camera
# self-check at startup asks you to move the camera). `> >(tee ...)` keeps $!
# as python3's PID, so stop/status target the right process.
# set -m (job control): without it, bash starts `&` jobs in a script with
# SIGINT *ignored*, and Python inherits that -- so `stop`'s kill -INT would
# be silently dropped and never reach the KeyboardInterrupt/cap.release()
# cleanup (stop would then always fall through to kill -9).
set -m
nohup python3 -u "$PROGRAM" "${ARGS[@]}" \
    > >(tee -a "$LOG_FILE") 2>&1 < /dev/null &
PID=$!
disown

echo "$PID" > "$PID_FILE"
echo "$LOG_FILE" > "$PID_FILE.log"

echo "Started $PROGRAM in the background (PID $PID)."
echo "Command:        python3 -u $PROGRAM ${ARGS[*]}"
echo "Log:            $LOG_FILE"
echo "Follow output:  tail -f $LOG_FILE"
echo "Check status:   $0 status"
echo "Stop it early:  $0 stop"
