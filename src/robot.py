"""
WRO 2026 Future Engineers - Robot Model
Simulates the physical robot with Ackermann steering, 4 ToF sensors, IMU, and color sensor.
"""

import math
import random
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
from config import *


@dataclass
class SensorReading:
    """Container for all sensor readings at a given timestamp."""
    tof_front: float = TOF_MAX_RANGE
    tof_rear: float = TOF_MAX_RANGE
    tof_left: float = TOF_MAX_RANGE
    tof_right: float = TOF_MAX_RANGE
    imu_heading: float = 0.0
    color_detected: Optional[str] = None  # 'orange', 'blue', or None
    pillars_visible: List[dict] = field(default_factory=list)  # [{color, angle, distance, x, y}]


class Robot:
    """
    Simulated robot with Ackermann steering geometry.
    
    Coordinate system:
    - x increases to the right
    - y increases downward
    - angle 0 = facing right, 90 = facing down (standard math, but y-flipped)
    """

    def __init__(self, x: float, y: float, angle: float):
        # Position (center of robot)
        self.x = x
        self.y = y
        self.angle = angle  # degrees, 0 = right, increases CW
        
        # Velocity
        self.speed = 0.0        # mm/s, positive = forward
        self.target_speed = 0.0
        
        # Steering
        self.steering_angle = 0.0  # degrees, positive = right turn
        self.target_steering = 0.0
        
        # IMU simulation
        self.imu_heading = angle
        self.imu_drift_rate = IMU_GYRO_DRIFT / 60.0  # degrees per second
        
        # State tracking
        self.distance_traveled = 0.0
        self.is_reversed = False  # True if driving in reverse
        
        # Dimensions
        self.length = ROBOT_LENGTH
        self.width = ROBOT_WIDTH
        self.wheelbase = WHEELBASE
        self.max_steering = MAX_STEERING_ANGLE

    def update(self, dt: float):
        """Update robot position based on current speed and steering."""
        # Smooth speed changes (acceleration/deceleration)
        speed_diff = self.target_speed - self.speed
        if speed_diff > 0:
            self.speed += min(speed_diff, ACCELERATION * dt)
        elif speed_diff < 0:
            self.speed += max(speed_diff, -DECELERATION * dt)

        # Smooth steering (servo response ~100-200ms)
        servo_speed = 300  # degrees per second
        steer_diff = self.target_steering - self.steering_angle
        if abs(steer_diff) > 0.1:
            self.steering_angle += max(-servo_speed * dt, 
                                       min(servo_speed * dt, steer_diff))
        else:
            self.steering_angle = self.target_steering

        # Clamp steering
        self.steering_angle = max(-self.max_steering, 
                                  min(self.max_steering, self.steering_angle))

        if abs(self.speed) < 0.1:
            self.imu_heading += self.imu_drift_rate * dt
            self.imu_heading += random.gauss(0, IMU_NOISE * dt)
            self.imu_heading = self.imu_heading % 360
            return

        # Ackermann steering kinematics
        if abs(self.steering_angle) > 0.5:
            # Turning radius from steering angle
            steer_rad = math.radians(self.steering_angle)
            turning_radius = self.wheelbase / math.tan(steer_rad)
            
            # Angular velocity
            angular_vel = self.speed / turning_radius  # rad/s
            
            # Update angle
            angle_change = math.degrees(angular_vel * dt)
            self.angle += angle_change
        else:
            angle_change = 0

        # Update position
        angle_rad = math.radians(self.angle)
        dx = self.speed * math.cos(angle_rad) * dt
        dy = self.speed * math.sin(angle_rad) * dt
        
        self.x += dx
        self.y += dy
        self.distance_traveled += abs(self.speed) * dt

        # Normalize angle
        self.angle = self.angle % 360

        # Update IMU (with drift and noise), normalize to [0, 360)
        self.imu_heading += angle_change
        self.imu_heading += self.imu_drift_rate * dt
        self.imu_heading += random.gauss(0, IMU_NOISE * dt)
        self.imu_heading = self.imu_heading % 360

    def set_speed(self, speed_fraction: float):
        """Set target speed as fraction of max speed. Negative = reverse."""
        self.target_speed = speed_fraction * MAX_SPEED
        self.is_reversed = speed_fraction < 0

    def set_steering(self, angle: float):
        """Set target steering angle in degrees. Positive = right."""
        self.target_steering = max(-self.max_steering, 
                                   min(self.max_steering, angle))

    def stop(self):
        """Emergency stop."""
        self.target_speed = 0
        self.speed = 0

    def get_sensors(self, track) -> SensorReading:
        """Read all sensors based on current position and track state."""
        reading = SensorReading()

        # ToF sensors - ray cast in 4 directions
        reading.tof_front = self._raycast_tof(track, 0)     # Forward
        reading.tof_rear = self._raycast_tof(track, 180)    # Backward
        reading.tof_left = self._raycast_tof(track, -90)    # Left
        reading.tof_right = self._raycast_tof(track, 90)    # Right

        # IMU heading
        reading.imu_heading = self.imu_heading

        # Color sensor (checks ground directly below robot)
        reading.color_detected = track.get_line_at_position(self.x, self.y)

        # Camera: detect pillars in FOV
        reading.pillars_visible = self._detect_pillars(track)

        return reading

    def _raycast_tof(self, track, relative_angle: float) -> float:
        """
        Cast a ray from the robot in a relative direction and find distance to nearest wall.
        relative_angle: 0 = forward, 90 = right, -90 = left, 180 = rear
        """
        ray_angle = math.radians(self.angle + relative_angle)
        ray_dx = math.cos(ray_angle)
        ray_dy = math.sin(ray_angle)

        # Offset start position based on sensor placement on robot
        if relative_angle == 0:  # Front sensor
            start_x = self.x + (self.length / 2) * math.cos(math.radians(self.angle))
            start_y = self.y + (self.length / 2) * math.sin(math.radians(self.angle))
        elif relative_angle == 180:  # Rear sensor
            start_x = self.x - (self.length / 2) * math.cos(math.radians(self.angle))
            start_y = self.y - (self.length / 2) * math.sin(math.radians(self.angle))
        elif relative_angle == -90:  # Left sensor
            perp = math.radians(self.angle - 90)
            start_x = self.x + (self.width / 2) * math.cos(perp)
            start_y = self.y + (self.width / 2) * math.sin(perp)
        else:  # Right sensor (90)
            perp = math.radians(self.angle + 90)
            start_x = self.x + (self.width / 2) * math.cos(perp)
            start_y = self.y + (self.width / 2) * math.sin(perp)

        min_dist = TOF_MAX_RANGE

        # Check against all wall segments
        for wall in track.get_wall_segments():
            dist = self._ray_segment_intersection(
                start_x, start_y, ray_dx, ray_dy,
                wall[0], wall[1], wall[2], wall[3]
            )
            if dist is not None and TOF_MIN_RANGE <= dist < min_dist:
                min_dist = dist

        # Check against pillars (treated as circles)
        pillar_r = PILLAR_DIAMETER / 2
        for pillar in track.pillars:
            dist = self._ray_circle_intersection(
                start_x, start_y, ray_dx, ray_dy,
                pillar.x, pillar.y, pillar_r
            )
            if dist is not None and TOF_MIN_RANGE <= dist < min_dist:
                min_dist = dist

        # Add noise
        min_dist += random.gauss(0, TOF_ACCURACY)
        return max(TOF_MIN_RANGE, min(TOF_MAX_RANGE, min_dist))

    def _ray_segment_intersection(self, ox, oy, dx, dy, x1, y1, x2, y2) -> Optional[float]:
        """
        Find intersection distance between ray (ox,oy,dx,dy) and line segment (x1,y1)-(x2,y2).
        Returns distance or None.
        """
        sx = x2 - x1
        sy = y2 - y1

        denom = dx * sy - dy * sx
        if abs(denom) < 1e-10:
            return None

        t = ((x1 - ox) * sy - (y1 - oy) * sx) / denom
        s = ((x1 - ox) * dy - (y1 - oy) * dx) / denom

        if t > 0 and 0 <= s <= 1:
            return t
        return None

    def _ray_circle_intersection(self, ox, oy, dx, dy, cx, cy, r) -> Optional[float]:
        """Find nearest intersection distance between ray and circle."""
        fx = ox - cx
        fy = oy - cy
        b = 2 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - r * r
        discriminant = b * b - 4 * c
        if discriminant < 0:
            return None
        sqrt_disc = math.sqrt(discriminant)
        t1 = (-b - sqrt_disc) / 2
        t2 = (-b + sqrt_disc) / 2
        if t1 > 0:
            return t1
        if t2 > 0:
            return t2
        return None

    def _detect_pillars(self, track) -> List[dict]:
        """
        Simulate camera detection of pillars.
        Returns list of visible pillars with their properties.
        """
        visible = []
        half_fov = CAMERA_FOV / 2

        for pillar in track.pillars:
            # Vector from robot to pillar
            dx = pillar.x - self.x
            dy = pillar.y - self.y
            distance = math.sqrt(dx * dx + dy * dy)

            if distance > CAMERA_DETECTION_RANGE or distance < CAMERA_MIN_DETECTION:
                continue

            # Angle to pillar relative to robot heading
            angle_to_pillar = math.degrees(math.atan2(dy, dx))
            relative_angle = (angle_to_pillar - self.angle + 180) % 360 - 180

            if abs(relative_angle) <= half_fov:
                # Check wall occlusion: cast ray toward pillar, see if a wall is closer
                ray_dx_n = dx / distance
                ray_dy_n = dy / distance
                occluded = False
                for wall in track.get_wall_segments():
                    wall_dist = self._ray_segment_intersection(
                        self.x, self.y, ray_dx_n, ray_dy_n,
                        wall[0], wall[1], wall[2], wall[3]
                    )
                    if wall_dist is not None and wall_dist < distance:
                        occluded = True
                        break

                if not occluded:
                    visible.append({
                        'color': pillar.color,
                        'angle': relative_angle,
                        'distance': distance,
                        'x': pillar.x,
                        'y': pillar.y,
                        'pillar_ref': pillar,
                    })

        # Sort by distance (closest first)
        visible.sort(key=lambda p: p['distance'])
        return visible

    def get_corners(self) -> List[Tuple[float, float]]:
        """Get the 4 corners of the robot for collision detection."""
        rad = math.radians(self.angle)
        cos_a = math.cos(rad)
        sin_a = math.sin(rad)
        hl = self.length / 2
        hw = self.width / 2

        return [
            (self.x + hl * cos_a - hw * sin_a, self.y + hl * sin_a + hw * cos_a),  # Front-left
            (self.x + hl * cos_a + hw * sin_a, self.y + hl * sin_a - hw * cos_a),  # Front-right
            (self.x - hl * cos_a + hw * sin_a, self.y - hl * sin_a - hw * cos_a),  # Rear-right
            (self.x - hl * cos_a - hw * sin_a, self.y - hl * sin_a + hw * cos_a),  # Rear-left
        ]

    def check_wall_collision(self, track) -> bool:
        """Check if robot is colliding with any wall."""
        corners = self.get_corners()
        for cx, cy in corners:
            if not track.is_on_track(cx, cy):
                return True
        return False

    def check_pillar_collision(self, track) -> Optional[int]:
        """Check if robot is touching a pillar. Returns pillar index or None."""
        corners = self.get_corners()
        robot_radius = max(self.length, self.width) / 2

        for i, pillar in enumerate(track.pillars):
            # Simple distance check first
            dist = math.sqrt((self.x - pillar.x)**2 + (self.y - pillar.y)**2)
            if dist > robot_radius + PILLAR_DIAMETER:
                continue
            
            # More precise: check if any corner is within pillar radius
            pillar_r = PILLAR_DIAMETER / 2
            for cx, cy in corners:
                if math.sqrt((cx - pillar.x)**2 + (cy - pillar.y)**2) < pillar_r + 10:
                    return i
        return None
