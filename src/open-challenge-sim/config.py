"""
WRO 2026 Future Engineers - Simulation Configuration
All measurements in mm, angles in degrees, time in seconds.
Based on official WRO-2026-Future-Engineers-Self-Driving-Cars-General-Rules.pdf
"""

import math

# =============================================================================
# TRACK DIMENSIONS (from rules section 13)
# =============================================================================
MAT_SIZE = 3200            # Full mat size (mm)
TRACK_INNER_SIZE = 3000    # Internal track size (mm)
WALL_HEIGHT = 100          # Wall height (mm) - for reference
WALL_OFFSET = (MAT_SIZE - TRACK_INNER_SIZE) / 2  # 100mm border

# Track widths
TRACK_WIDTH_OPEN_NARROW = 600    # Open challenge narrow
TRACK_WIDTH_OPEN_WIDE = 1000     # Open challenge wide
TRACK_WIDTH_OBSTACLE = 1000      # Obstacle challenge always 1000mm

# Lines
LINE_THICKNESS = 20        # Orange/blue line thickness (mm)

# Sections: the track has 8 sections (4 straight + 4 corners)
NUM_SECTIONS = 8
NUM_STRAIGHT_SECTIONS = 4
NUM_CORNER_SECTIONS = 4

# =============================================================================
# TRAFFIC SIGNS / PILLARS (from rules sections 8, 9, 13)
# =============================================================================
PILLAR_DIAMETER = 50       # Pillar base ~50mm (fits in seat)
PILLAR_HEIGHT = 100        # Approximate pillar height
SIGN_SEAT_SIZE = 50        # Traffic sign seat 50x50mm
EVALUATION_CIRCLE_DIAMETER = 85  # Circle around seat for movement check

# Zones per straight section
ZONES_PER_SECTION = 6      # 6 zones per straightforward section
SEATS_PER_SECTION = 6      # 4 T-intersections + 2 X-intersections

# =============================================================================
# PARKING LOT (from rules and appendix A.6)
# =============================================================================
PARKING_WIDTH = 200         # Always 20cm
PARKING_LENGTH_FACTOR = 1.5 # 1.5 * robot length = 210mm with new dimensions
PARKING_MARKER_SIZE = (200, 20, 100)  # Magenta markers: 20cm x 2cm x 10cm
PARKING_PARALLEL_TOLERANCE = 20  # 2cm = 20mm wheel distance difference

# =============================================================================
# ROBOT SPECIFICATIONS (from CAD model measurements, 2026-05-14)
# =============================================================================
ROBOT_LENGTH = 140         # mm (CAD: 14cm)
ROBOT_WIDTH = 88           # mm (CAD: 8.8cm, includes tires)
ROBOT_HEIGHT = 56          # mm (CAD: 5.626cm)
ROBOT_WEIGHT = 219         # grams (CAD estimate: 219.42g)
ROBOT_MAX_LENGTH = 300     # Max allowed by rules
ROBOT_MAX_WIDTH = 200      # Max allowed by rules
ROBOT_MAX_HEIGHT = 300     # Max allowed by rules
ROBOT_MAX_WEIGHT = 1500    # 1.5kg in grams

# Drive system (from CAD model)
WHEELBASE = 76             # Distance between front and rear axles (mm, CAD: 7.6cm)
TRACK_WHEEL = 88           # Distance between left and right wheels (mm, same as width)
WHEEL_DIAMETER = 30.4      # mm (LEGO 55981C05)
MAX_STEERING_ANGLE = 77.48  # degrees (measured from CAD linkage)

# Motor: N20 12V 136RPM
MOTOR_RPM = 136
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
MAX_SPEED = (MOTOR_RPM * WHEEL_CIRCUMFERENCE) / 60  # mm/s ≈ 216.5 mm/s
ACCELERATION = 500         # mm/s² (approximate)
DECELERATION = 800         # mm/s² (approximate, braking)

# Speed profiles (as fraction of max speed)
# NOTE: cruise at 0.80 — balance between speed and stability.
# Corner speed 0.55 keeps turns controlled through the bicycle model turn.
SPEED_CRUISE = 0.80
SPEED_CORNER = 0.55
SPEED_PILLAR = 0.65
SPEED_PARKING = 0.35
SPEED_THREE_POINT = 0.30

# Open challenge speeds (no pillars to dodge — push harder)
SPEED_OPEN_CRUISE = 0.95
SPEED_OPEN_CORNER = 0.70

# Wall-following PD steering limit (separate from robot's physical max)
WALL_FOLLOW_MAX_STEER = 30  # degrees — prevents PD oscillation with high max steering

# Corner turn geometry (bicycle model: R = wheelbase / tan(δ))
# R = track_width/2 = 500mm keeps the robot centered through the corner arc
CORNER_TURN_RADIUS = 500   # mm — turning radius matched to track geometry
CORNER_STEERING_ANGLE = math.degrees(math.atan(WHEELBASE / CORNER_TURN_RADIUS))  # ~8.6°
CORNER_ARC_LENGTH = CORNER_TURN_RADIUS * math.pi / 2  # ~785mm for a 90° arc
CORNER_MIN_EXIT_ANGLE = 87   # degrees — minimum heading change to exit corner
CORNER_MAX_EXIT_ANGLE = 100  # degrees — safety cap, force exit above this

# =============================================================================
# SENSOR SIMULATION
# =============================================================================
# ToF sensors (VL53L1X × 4, all mounted at 90° perpendicular to their face)
TOF_MAX_RANGE = 4000       # 4m max range
TOF_MIN_RANGE = 4          # 4mm min range
TOF_ACCURACY = 1           # ±1mm
TOF_BEAM_ANGLE = 4         # ~4 degree cone (approximate)
TOF_COUNT = 4              # front, left, right, rear
TOF_ANGLES = {             # pointing angle relative to robot heading (degrees)
    'front': 0,
    'left': -90,
    'right': 90,
    'rear': 180,
}

# IMU (MPU6050 / 10-DoF)
IMU_GYRO_DRIFT = 1.0       # degrees per minute drift
IMU_NOISE = 0.1            # degrees noise per reading

# Color sensor (TCS34725, mounted on underside)
COLOR_SENSOR_RANGE = 30    # mm from ground (effective detection range)

# Camera (OV5647 160° wide angle, mounted front-facing at 90° — no tilt)
CAMERA_FOV = 160           # degrees
CAMERA_DETECTION_RANGE = 1500  # mm max pillar detection distance
CAMERA_MIN_DETECTION = 100 # mm min detection distance

# =============================================================================
# PD CONTROL PARAMETERS (from team design doc)
# =============================================================================
KP_WALL = 0.05
KD_WALL = 0.02
KP_PILLAR = 0.08
KD_PILLAR = 0.03
KP_TURN = 0.06
KD_TURN = 0.02

# =============================================================================
# SCORING (from rules section 10)
# =============================================================================
POINTS_PER_SECTION = 1       # 1.1: per section in correct direction
MAX_SECTION_POINTS = 24      # 24 sections total (3 laps × 8)
POINTS_PER_LAP = 1           # 1.2: per completed lap
MAX_LAP_POINTS = 3           # 3 laps
POINTS_FINISH_SECTION = 3    # 1.3: stopped in finish section
POINTS_OBSTACLE_MOVED = 2    # 1.4: signs moved, < 3 laps (need 1 lap)
POINTS_OBSTACLE_NOT_MOVED = 4  # 1.5: signs not moved, < 3 laps
POINTS_3LAPS_MOVED = 8       # 1.6: 3 laps, signs moved
POINTS_3LAPS_NOT_MOVED = 10  # 1.7: 3 laps, signs not moved
POINTS_START_IN_PARKING = 7  # 1.8.1: started in parking lot + 1 lap
POINTS_PARKED_FULLY = 15     # 1.8.2: fully parked and parallel
POINTS_PARKED_PARTLY = 7     # 1.8.3: partly parked or not parallel

# =============================================================================
# SIMULATION PARAMETERS
# =============================================================================
SIM_FPS = 60               # Simulation frame rate
CONTROL_HZ = 30            # Robot control loop rate (matches real robot)
ROUND_TIME = 180           # 3 minutes in seconds
PIXELS_PER_MM = 0.25       # Scale: 1mm = 0.25 pixels (800px for 3200mm)
WINDOW_WIDTH = int(MAT_SIZE * PIXELS_PER_MM) + 200  # Extra for UI panel
WINDOW_HEIGHT = int(MAT_SIZE * PIXELS_PER_MM)

# Colors (RGB)
COLOR_WHITE = (255, 255, 255)
COLOR_BLACK = (0, 0, 0)
COLOR_RED = (220, 40, 40)
COLOR_GREEN = (40, 180, 40)
COLOR_ORANGE = (255, 140, 0)       # CMYK (0,60,100,0) ≈ RGB
COLOR_BLUE = (0, 70, 180)          # CMYK (100,80,0,0) ≈ RGB
COLOR_MAGENTA = (200, 0, 200)
COLOR_GRAY = (180, 180, 180)
COLOR_DARK_GRAY = (100, 100, 100)
COLOR_LIGHT_GRAY = (220, 220, 220)
COLOR_YELLOW = (255, 255, 0)
COLOR_ROBOT = (50, 120, 220)
COLOR_ROBOT_FRONT = (255, 200, 0)
COLOR_SENSOR_RAY = (0, 255, 255, 80)
