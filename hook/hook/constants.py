# Tekeoff
TAKEOFF_ALTITUDE = 5.0

# Line Detection Parameters
LINE_DETECTION_BLUE_COLOR_NAME = "blue"
LINE_DETECTION_RED_COLOR_NAME = "red"
LINE_DETECTION_BLUE_SPACE = "hsv"
LINE_DETECTION_RED_SPACE = "hsv"
LINE_DETECTION_IMAGE_SOURCE = "webcam"
LINE_DETECTION_SHOW_VISUALIZATION = "false"
LINE_DETECT_NODE_NAME = "line_detector_process"
RED_DETECT_NODE_NAME = "red_detector_process"
LINE_DETECTION_BLUE_SEARCH_TITLE = "Blue Line Search"
LINE_DETECTION_LINE_FOLLOWING_TITLE = "Line Following"
LINE_DETECTION_RED_CENTERING_TITLE = "Red Centering"
LINE_DETECTION_DESCENT_TITLE = "Descent Tracking"
LINE_DETECTION_METHOD = "RotatedRect"

FORWARD_SPEED_SEARCH_BLUE_LINE = 0.18

MIN_BLUE_LINE_DETECTIONS = 20  # Consecutive detections to confirm line

MIN_DESCEND_ALTITUDE = 2.6  # Failsafe altitude during descent

RETURN_ALTITUDE = 2  # Altitude for RTL

IMAGE_CENTER_X = 320.0
IMAGE_CENTER_Y = 240.0
ANGLE_SETPOINT = 0.0

FORWARD_SPEED_FOLLOW_BLUE_LINE = 0.22

# PID Controller Process Names
CENTER_PID_PROCESS = "center_pid_process"
ANGLE_PID_PROCESS = "angle_pid_process"
CENTERING_PID_PROCESS = "centering_pid_process"

# PID Controller default parameters
CENTER_P = 0.0017
CENTER_I = 0.0
CENTER_D = 0.0001
CENTER_OUTPUT_MIN = -0.7
CENTER_OUTPUT_MAX = 0.7

ANGLE_P = 0.013
ANGLE_I = 0.0
ANGLE_D = 0.001
ANGLE_OUTPUT_MIN = -0.65
ANGLE_OUTPUT_MAX = 0.65

CENTERING_P = 0.0015
CENTERING_I = 0.0
CENTERING_D = 0.0001
CENTERING_OUTPUT_MIN = -0.5
CENTERING_OUTPUT_MAX = 0.5

MIN_RED_COUNT_CONFIRMATIONS = 8  # Consecutive frames above threshold
CENTERING_CONFIRMATIONS = 28  # Consecutive low effort confirmations

DESCEND_SPEED = -0.2  # m/s (negative for downward)
DESCEND_TIMEOUT = 30  # Timeout for descent in seconds

# Distance estimation calibration
DISTANCE_CALIBRATION_CONST = 1476.0  # cm*px (24px at 61.5cm)
TARGET_DISTANCE_CM = 20.0  # Target distance from hose in cm
DISTANCE_TOLERANCE_CM = 2.0  # Tolerance for distance estimation

# Descend PID Parameters
DESCEND_KP_Z = 0.01  # Proportional gain for descent speed
DESCEND_KP_Y = -0.003  # Proportional gain for lateral speed
DESCEND_KP_X = -0.003  # Proportional gain for forward speed
DESCEND_MAX_SPEED_Z = 0.25  # m/s
DESCEND_MAX_SPEED_XY = 0.20  # m/s

HOCK_SERVO_CHANNEL = 3
HOCK_RELEASE_PWM = 2000.0
HOCK_HOLD_PWM = 1000.0
