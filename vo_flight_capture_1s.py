#!/usr/bin/env python3
"""
vo_flight_capture_1s.py — identical to vo_flight_capture.py, except the
default --interval is 1 second instead of 2. Everything else (startup
camera self-check, takeoff/landing altitude gating, VO math, filename
format, photo_obtained/<run_timestamp>/ folder, stuck-camera detection,
save-failure warnings) is unchanged -- this just runs
vo_flight_capture.main() with a different default capture cadence,
reused directly rather than duplicated.

Usage:
    python3 vo_flight_capture_1s.py --port /dev/ttyTHS1
    python3 vo_flight_capture_1s.py --interval 0.5   # still overridable
"""
import sys

import vo_flight_capture as vfc

if __name__ == '__main__':
    if not any(arg == '--interval' or arg.startswith('--interval=') for arg in sys.argv[1:]):
        sys.argv[1:1] = ['--interval', '1']
    vfc.main()
