"""Canonical topic manifest for synchronized RGB-D flight recordings."""

RGB_RAW_TOPIC = '/camera_448/color/image_raw'
RGB_COMPRESSED_TOPIC = '/camera_448/color/image_raw/compressed'
DEPTH_RAW_TOPIC = '/camera_448/aligned_depth_to_color/image_raw'
DEPTH_COMPRESSED_TOPIC = (
    '/camera_448/aligned_depth_to_color/image_raw/compressedDepth')

CAMERA_INFO_TOPICS = (
    '/camera_448/color/camera_info',
    '/camera_448/aligned_depth_to_color/camera_info',
)

ALL_CAMERA_IMAGE_TOPICS = (
    RGB_RAW_TOPIC,
    RGB_COMPRESSED_TOPIC,
    DEPTH_RAW_TOPIC,
    DEPTH_COMPRESSED_TOPIC,
)

# Raw Vicon pose plus PX4's local estimate in NED. Keeping both lets later
# visualizations compare mocap ground truth with the state used by control.
POSITION_TOPICS = (
    '/vicon/px4_201/px4_201',
    '/px4_201/fmu/in/vehicle_visual_odometry',
    '/px4_201/fmu/out/vehicle_local_position',
    '/px4_201/fmu/out/vehicle_odometry',
)

# The input setpoint is the commanded path. The viz topic is retained when a
# trajectory publisher provides its geometry-friendly mirror.
TRAJECTORY_TOPICS = (
    '/px4_201/fmu/in/trajectory_setpoint',
    '/px4_201/viz/trajectory_setpoint',
    '/px4_201/fmu/in/offboard_control_mode',
)

TF_TOPICS = (
    '/tf',
    '/tf_static',
)

# These topics are small and preserve the evidence needed to diagnose mode or
# failsafe transitions alongside the visual recording.
STATUS_TOPICS = (
    '/px4_201/fmu/out/vehicle_status',
    '/px4_201/fmu/out/failsafe_flags',
    '/px4_201/fmu/out/battery_status',
    '/px4_201/fmu/out/vehicle_land_detected',
    '/px4_201/fmu/out/commander_status',
    '/rosout',
)

BASE_RECORD_TOPICS = (
    CAMERA_INFO_TOPICS + POSITION_TOPICS + TRAJECTORY_TOPICS + TF_TOPICS +
    STATUS_TOPICS
)

ALL_RECORD_TOPICS = ALL_CAMERA_IMAGE_TOPICS + BASE_RECORD_TOPICS
