#!/bin/bash
# Launch photo_capture.py (plain photo + GPS logger) in the background, so it
# keeps running even if the SSH session / terminal that started it closes.
#
# Built-in defaults:  --port /dev/ttyTHS1 --baud 57600 --interval 0.5 --tnr-mode 0
#
# Usage:
#   ./run_photo_capture.sh                                  # start with the defaults
#   ./run_photo_capture.sh --exposure-ms 1 3                # shutter range 1-3 ms
#   ./run_photo_capture.sh --exposure-ms 2 2 --tnr-mode 1   # fixed 2 ms shutter, TNR on
#   ./run_photo_capture.sh --interval 1 --format jpg        # any extra arg overrides its default
#   ./run_photo_capture.sh help                             # list every parameter
#
#   ./run_photo_capture.sh status                           # running? PID + log path
#   ./run_photo_capture.sh stop                             # stop (clean camera release)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PROGRAM="photo_capture.py"
PID_FILE="$SCRIPT_DIR/photo_capture.pid"
LOG_DIR="$SCRIPT_DIR/logs"
DEFAULT_ARGS=(--port /dev/ttyTHS1 --baud 57600 --interval 0.5 --tnr-mode 0)
# Other launchers' PID files -- only one program can own the camera / serial port.
OTHER_PID_FILES=(vo_flight_capture.pid vo_flight_capture_1s.pid vo_flight_capture_always.pid
                 vo_flight_capture_fix2.pid)

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
        kill -INT "$PID"   # SIGINT -> the program releases the camera cleanly
        for i in $(seq 1 10); do
            if ! kill -0 "$PID" 2>/dev/null; then
                echo "Stopped."
                rm -f "$PID_FILE" "$PID_FILE.log"
                return 0
            fi
            sleep 1
        done
        echo "PID $PID did NOT exit within 10s (probably stuck in a camera read) -- kill -9."
        echo "The camera may then need 'sudo systemctl restart nvargus-daemon' or a reboot."
        kill -9 "$PID" 2>/dev/null || true
        sleep 1
        if kill -0 "$PID" 2>/dev/null; then
            echo "ERROR: PID $PID survived kill -9. Check 'ps -o stat -p $PID'."
            return 1
        fi
        rm -f "$PID_FILE" "$PID_FILE.log"
        echo "Force-killed."
    else
        echo "Not running."
    fi
}

case "${1:-}" in
    status) status; exit 0 ;;
    stop) stop; exit 0 ;;
    help|-h|--help) python3 "$PROGRAM" --help; exit 0 ;;
esac

if is_running "$PID_FILE"; then
    echo "Already running (PID $(cat "$PID_FILE")). Use '$0 stop' first, or '$0 status'."
    exit 1
fi
for f in "${OTHER_PID_FILES[@]}"; do
    if is_running "$SCRIPT_DIR/$f"; then
        echo "Another capture program is running (PID $(cat "$SCRIPT_DIR/$f"), pid file $f)"
        echo "and owns the camera / serial port. Stop it first with its own run_*.sh stop."
        exit 1
    fi
done

ARGS=("${DEFAULT_ARGS[@]}" "$@")   # user args last: argparse keeps the last value given

mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/photo_capture_$(date +%Y%m%d_%H%M%S).log"
echo "command: python3 -u $PROGRAM ${ARGS[*]}" > "$LOG_FILE"

# Output goes to BOTH the log file and this terminal. `> >(tee ...)` keeps $!
# as python3's PID so stop/status target the right process.
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
echo "Stop:           $0 stop"
