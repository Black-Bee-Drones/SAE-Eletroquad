# Mission Parameters
TAKEOFF_ALTITUDE = 3.0

FORWARD_SPEED = 0.0  # m/s
MIN_BLUE_LINE_DETECTIONS = 20  # Consecutive detections to confirm line

MAX_DESCEND_AREA_FACTOR = 0.95  # Descend until area is 95% of max seen
MIN_DESCEND_ALTITUDE = 2.15  # Failsafe altitude during descent

RETURN_ALTITUDE = 2  # Altitude for RTL
LINE_DETECT_NODE_NAME = "line_detector_process"
RED_DETECT_NODE_NAME = "red_detector_process"  # If we make a separate node
IMAGE_CENTER_X = 320.0
IMAGE_CENTER_Y = 240.0
ANGLE_SETPOINT = 0.0

# Line Detection Parameters
LINE_DETECTION_BLUE_COLOR_NAME = "blue"
LINE_DETECTION_RED_COLOR_NAME = "red"
LINE_DETECTION_BLUE_SPACE = "lab"
LINE_DETECTION_RED_SPACE = "lab"
LINE_DETECTION_IMAGE_SOURCE = "webcam"
LINE_DETECTION_SHOW_VISUALIZATION = "true"
LINE_DETECTION_BLUE_SEARCH_TITLE = "Blue Line Search"
LINE_DETECTION_LINE_FOLLOWING_TITLE = "Line Following"
LINE_DETECTION_RED_CENTERING_TITLE = "Red Centering"
LINE_DETECTION_DESCENT_TITLE = "Descent Tracking"

MIN_RED_COUNT_CONFIRMATIONS = 20  # Consecutive frames above threshold
FORWARD_SPEED = 0.0  # m/s

CENTERING_CONFIRMATIONS = 5  # Consecutive low effort confirmations

# PID Controller Process Names
CENTER_PID_PROCESS = "center_pid_process"
ANGLE_PID_PROCESS = "angle_pid_process"
CENTERING_PID_PROCESS = "centering_pid_process"

# PID Controller default parameters
CENTER_P = 0.001
CENTER_I = 0.0
CENTER_D = 0.0001
CENTER_OUTPUT_MIN = -0.3
CENTER_OUTPUT_MAX = 0.3

ANGLE_P = 0.01
ANGLE_I = 0.0
ANGLE_D = 0.001
ANGLE_OUTPUT_MIN = -0.5
ANGLE_OUTPUT_MAX = 0.5

CENTERING_P = 0.002
CENTERING_I = 0.0
CENTERING_D = 0.0001
CENTERING_OUTPUT_MIN = -0.2
CENTERING_OUTPUT_MAX = 0.2


DESCEND_SPEED = -0.2  # m/s (negative for downward)
DESCEND_TIMEOUT = 30  # Timeout for descent in seconds

HOCK_SERVO_CHANNEL = 9  # Example channel
HOCK_RELEASE_PWM = 1900  # Example PWM
HOCK_HOLD_PWM = 1100  # Example PWM
