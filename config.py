"""Camera and algorithm parameters for GPS-less visual odometry (VO)."""

# ── Camera: Jetson IMX219 CSI, captured at 1280x720 (same rig as gpsless_mapping) ──
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720

# Empirical ground footprint at a known reference altitude, from
# gpsless_mapping/readme.md: "at 60 m altitude the IMX219 camera captures a
# ground footprint of roughly 70 m x 55 m". No checkerboard calibration has
# been done for this rig — these are the best numbers on hand. If you ever
# calibrate the lens properly (focal length / distortion coeffs), replace
# FX_PIXELS/FY_PIXELS below with the calibrated values instead of deriving
# them from the footprint estimate.
REFERENCE_ALTITUDE_M = 60.0
FOOTPRINT_WIDTH_M = 70.0
FOOTPRINT_HEIGHT_M = 55.0

# Focal length in pixels, derived from the footprint estimate:
#   fx = (image_width / 2) * (altitude / (footprint_width / 2))
FX_PIXELS = (IMAGE_WIDTH / 2) * (REFERENCE_ALTITUDE_M / (FOOTPRINT_WIDTH_M / 2))
FY_PIXELS = (IMAGE_HEIGHT / 2) * (REFERENCE_ALTITUDE_M / (FOOTPRINT_HEIGHT_M / 2))

# The images show visible barrel (fisheye-style) distortion near the edges
# (curved running track / court lines close to the border). No distortion
# coefficients are known, so instead of correcting distortion, feature
# matching is restricted to a centered crop where the lens is closest to
# rectilinear. 1.0 = use the full frame, 0.8 = use the central 80%.
CENTRAL_CROP_FRACTION = 0.8

# ── Camera exposure/shutter override (nvarguscamerasrc) ─────────────────────
# Default auto-exposure picks exposure times close to the full 1/framerate
# interval (~33ms at 30fps) in daylight, long enough to visibly motion-blur
# the ground texture at normal flight speed/altitude-change rate (confirmed
# 2026-11-08 on the three test flights in flight_analysis_20261108/ -- grass
# texture smeared into directional streaks, degrading SIFT match/inlier
# counts). Capping the max exposure time forces a faster shutter; gain is
# raised to compensate for the resulting loss of brightness.
#
# Measured on this rig's IMX219 2026-11-08 via:
#   gst-launch-1.0 nvarguscamerasrc num-buffers=3 ! fakesink   (GST_DEBUG=3)
# which dumps the sensor's actual supported ranges (same across all sensor
# modes, 1280x720 included): exposure 13,000-683,709,000 ns,
# analog gain 1.0-10.625x. Stay within these hardware limits.
EXPOSURE_TIME_RANGE_NS = (1_000_000, 5_000_000)   # 1-5ms, vs ~33ms default -- re-tune in daylight vs overcast
GAIN_RANGE = (1.0, 10.625)                         # full analog gain headroom, to offset the shorter exposure
ISP_DIGITAL_GAIN_RANGE = (1.0, 4.0)                # additional ISP-stage digital gain

# ── Feature matching ─────────────────────────────────────────────────────────
DETECTOR = 'sift'          # 'sift' or 'orb'
SIFT_N_FEATURES = 4000
ORB_N_FEATURES = 4000
MATCH_RATIO = 0.75         # Lowe's ratio test threshold
RANSAC_REPROJ_THRESHOLD_PX = 4.0
MIN_INLIERS = 12           # below this, the pair's displacement is unreliable
# Repetitive ground texture (grass rows, painted track lane markings, etc.)
# lets RANSAC lock onto a self-consistent but WRONG transform when only a
# small fraction of matches actually agree. Same guard used in the sibling
# gpsless_superpoint project for the same reason.
MIN_INLIER_RATIO = 0.30

# altitude_change_m is derived from the transform's scale as
# altitude_m * (1/scale - 1) -- the 1/scale term blows up to a huge value
# (or NaN, if scale is exactly 0) for a near-degenerate RANSAC fit, which
# would otherwise permanently poison PositionTracker.alt_m (it accumulates
# with +=, so one NaN/huge leg corrupts every altitude reading for the rest
# of the flight -- confirmed happening on the 2026-11-08 flights, see
# flight_analysis_20261108/). A real one-leg altitude change (a couple of
# seconds apart) should keep scale close to 1.0; outside this range the fit
# is treated as not carrying usable altitude information, and
# altitude_change_m falls back to 0.0 (no altitude update that leg) instead
# of a NaN/wild value.
SCALE_SANE_RANGE = (0.3, 3.0)

# ── Dead reckoning / MAVLink GPS_INPUT bridge ────────────────────────────────
# Compass heading (deg, clockwise from North) that the CAMERA's "image up"
# direction points toward when the airframe heading is 0. Leave at 0 if the
# camera is mounted with image-up = nose-forward; adjust if the CSI camera
# is physically rotated relative to the airframe (common on compact mounts).
CAMERA_MOUNT_YAW_OFFSET_DEG = 0.0

ANCHOR_HACC_M = 0.5   # accuracy assumed for a fresh absolute fix (tile-match / GPS)
HACC_MAX_M = 50.0     # cap — beyond this, just admit the estimate is lost

# Heuristic dead-reckoning error growth — NOT a calibrated error model, just
# a starting point. Tune against real flight logs once you have some: fly a
# route with real GPS logging alongside GPS1_TYPE left at 1 (auto), compare
# the injected GPS2 track to truth, and adjust these until hacc growth
# roughly tracks actual drift.
HACC_GROWTH_FRACTION_CONFIDENT = 0.15   # fraction of leg magnitude added (in quadrature) per confident leg
HACC_GROWTH_FRACTION_LOW_CONF = 0.6     # same, but for low-confidence legs
HACC_PENALTY_LOW_CONFIDENCE_M = 2.0     # flat penalty on top, for a low-confidence leg
HACC_PENALTY_NO_MATCH_M = 5.0           # flat penalty when a pair fails outright (zero displacement info)
