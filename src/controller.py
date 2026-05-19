"""
WRO 2026 Future Engineers - Autonomous Controller
Implements the state machine, PD wall following, pillar avoidance,
three-point turn, and parking maneuver.

This is the code that most closely mirrors what runs on the real robot.
"""

import math
from enum import Enum, auto
from typing import Optional
from config import *


def _angle_diff(a, b):
    """Shortest signed angle from b to a, in range [-180, 180]."""
    d = (a - b) % 360
    if d > 180:
        d -= 360
    return d


class State(Enum):
    """Robot state machine states."""
    STARTING = auto()
    WALL_FOLLOWING = auto()
    PILLAR_AVOIDANCE = auto()
    CORNER_TURN = auto()
    THREE_POINT_TURN = auto()
    PARKING_APPROACH = auto()
    PARKING_EXECUTE = auto()
    STOP_SECTION = auto()
    FINISHED = auto()
    STOPPED = auto()


class ThreePointPhase(Enum):
    """Phases of the three-point turn maneuver."""
    TURN_RIGHT_FORWARD = auto()
    TURN_LEFT_REVERSE = auto()
    TURN_RIGHT_FORWARD_2 = auto()
    COMPLETE = auto()


class ParkingPhase(Enum):
    """Phases of the parallel parking maneuver."""
    APPROACH = auto()
    ALIGN = auto()
    STEER_IN = auto()
    STRAIGHTEN = auto()
    FINAL_ADJUST = auto()
    COMPLETE = auto()


class Controller:
    """
    Autonomous driving controller for WRO Future Engineers.
    """

    def __init__(self, driving_direction: int = 1, start_in_parking: bool = True,
                 challenge_type: str = 'obstacle'):
        self.state = State.STARTING
        self.driving_direction = driving_direction  # 1=CW, -1=CCW
        self.start_in_parking = start_in_parking
        self._is_open = (challenge_type == 'open')

        # Lap counting
        self.sections_passed = 0
        self.current_section = -1
        self.laps_completed = 0
        self.sections_in_current_lap = 0

        # Line detection
        self.last_line_color = None
        self.line_cooldown = 0  # Frames to ignore after detection
        self._last_line_pos = None  # (x, y) of last line detection

        # PD control state
        self.prev_wall_error = 0
        self.prev_pillar_error = 0

        # Pillar avoidance
        self.avoiding_pillar = None
        self.avoidance_side = 0  # -1 = go left, +1 = go right
        self.pillar_passed = False
        self.avoidance_timer = 0

        # Corner turn
        self._corner_entry_heading = None
        self._corner_cooldown = 0
        self._corner_line_detected = False
        self._corner_entry_distance = 0
        self._corner_radius = CORNER_TURN_RADIUS  # Dynamic per-turn radius
        self._corner_approach_ticks = 0  # Pre-corner centering countdown

        # Three-point turn
        self.three_point_phase = ThreePointPhase.TURN_RIGHT_FORWARD
        self.three_point_start_heading = 0
        self.three_point_target_heading = 0
        self.needs_direction_change = False
        self.direction_changed = False

        # Parking
        self.parking_phase = ParkingPhase.APPROACH
        self.parking_start_heading = 0
        self.parking_timer = 0

        # Stop section verification (open challenge)
        self.start_tof_front = None
        self.start_tof_rear = None
        self.stop_section_tolerance = 200  # mm

        # Timing
        self.elapsed_time = 0
        self.control_timer = 0

        # Track info (set during first update)
        self.track_width = TRACK_WIDTH_OBSTACLE

    def update(self, sensors, robot, track, dt: float):
        """
        Main control loop. Called at CONTROL_HZ rate.
        
        Args:
            sensors: SensorReading from robot
            robot: Robot instance
            track: Track instance
            dt: Time step in seconds
        """
        self._corner_line_detected = False
        self.elapsed_time += dt
        self.control_timer += dt
        self.track_width = track.get_local_track_width(robot.x, robot.y)

        # Check time limit
        if self.elapsed_time >= ROUND_TIME:
            robot.stop()
            self.state = State.STOPPED
            return

        # Detect section changes via color lines
        self._detect_lines(sensors, robot)

        # State machine
        if self.state == State.STARTING:
            self._handle_starting(sensors, robot, track)
        elif self.state == State.WALL_FOLLOWING:
            self._handle_wall_following(sensors, robot, track)
        elif self.state == State.PILLAR_AVOIDANCE:
            self._handle_pillar_avoidance(sensors, robot, track)
        elif self.state == State.CORNER_TURN:
            self._handle_corner_turn(sensors, robot, track)
        elif self.state == State.THREE_POINT_TURN:
            self._handle_three_point_turn(sensors, robot, track)
        elif self.state == State.PARKING_APPROACH:
            self._handle_parking_approach(sensors, robot, track)
        elif self.state == State.PARKING_EXECUTE:
            self._handle_parking_execute(sensors, robot, track)
        elif self.state == State.STOP_SECTION:
            self._handle_stop_section(sensors, robot, track)
        elif self.state == State.FINISHED:
            robot.stop()
        elif self.state == State.STOPPED:
            robot.stop()

    def _detect_lines(self, sensors, robot):
        """Detect orange/blue lines for section counting.

        Uses both a frame cooldown and a minimum distance requirement
        to prevent double-counting when the robot oscillates near a line.
        """
        if self.line_cooldown > 0:
            self.line_cooldown -= 1
            return

        if sensors.color_detected in ('orange', 'blue'):
            if self._last_line_pos is not None:
                dx = robot.x - self._last_line_pos[0]
                dy = robot.y - self._last_line_pos[1]
                if math.sqrt(dx * dx + dy * dy) < 200:
                    return

            self._last_line_pos = (robot.x, robot.y)
            self.last_line_color = sensors.color_detected
            self.sections_passed += 1
            self.sections_in_current_lap += 1
            self.line_cooldown = 15

            # Corner entry detection: only trigger on the "entry" color
            # CW: orange = entering corner, CCW: blue = entering corner
            corner_entry_color = 'orange' if self.driving_direction == 1 else 'blue'
            if sensors.color_detected == corner_entry_color:
                self._corner_line_detected = True

            if self.sections_in_current_lap >= 8:
                self.laps_completed += 1
                self.sections_in_current_lap = 0

                if self.laps_completed >= 3:
                    if self._is_open:
                        pass
                    elif self.state != State.THREE_POINT_TURN:
                        self.state = State.PARKING_APPROACH

    def _handle_starting(self, sensors, robot, track):
        """Initial start - begin driving."""
        self.start_tof_front = sensors.tof_front
        self.start_tof_rear = sensors.tof_rear
        robot.set_speed(SPEED_CRUISE)
        robot.set_steering(0)
        self.state = State.WALL_FOLLOWING

    def _handle_wall_following(self, sensors, robot, track):
        """PD wall following using left and right ToF sensors."""
        # Corner cooldown: soft PD centering until clear of corner area.
        # Uses reduced-gain wall centering + heading correction to keep
        # the robot on track without the oscillation of full PD in corners.
        if self._corner_cooldown > 0:
            self._corner_line_detected = False
            cruise = SPEED_OPEN_CRUISE if self._is_open else SPEED_CRUISE
            robot.set_speed(cruise)

            # PD centering — gain scales with track narrowness.
            # Narrow tracks need stronger correction to avoid wall collision.
            narrow = self.track_width <= 700
            wall_gain = 0.6 if narrow else 0.3
            heading_gain = 1.5 if narrow else 0.3

            error = sensors.tof_right - sensors.tof_left
            correction = error * KP_WALL * wall_gain

            # Heading correction toward nearest cardinal (0/90/180/270)
            heading = sensors.imu_heading
            cardinal = (round(heading / 90) * 90) % 360
            heading_error = _angle_diff(cardinal, heading)
            correction += heading_error * heading_gain

            correction = max(-WALL_FOLLOW_MAX_STEER, min(WALL_FOLLOW_MAX_STEER, correction))
            robot.set_steering(correction)

            # Exit cooldown early if robot is in a straight section
            section_idx = track.get_section_at_position(robot.x, robot.y)
            in_straight = (section_idx is not None and
                           track.sections[section_idx].section_type == 'straight')
            if in_straight or sensors.tof_front > 1000:
                self._corner_cooldown = 0  # Resume full PD
            else:
                self._corner_cooldown -= 1
            return  # Skip full PD and pillar logic while in cooldown

        # Pre-corner centering: aggressive PD to center + heading correction.
        # This compensates for pillar avoidance shifting the robot off-center
        # and ensures cardinal heading before the turn begins.
        if self._corner_approach_ticks > 0:
            self._corner_approach_ticks -= 1

            tof_l = sensors.tof_left
            tof_r = sensors.tof_right
            if tof_l < self.track_width and tof_r < self.track_width:
                error = tof_r - tof_l
                steering = error * KP_WALL * 2.0
            else:
                steering = 0.0

            heading = sensors.imu_heading
            cardinal = (round(heading / 90) * 90) % 360
            heading_error = _angle_diff(cardinal, heading)
            # Stronger heading correction for wide tracks before corner entry
            heading_gain = 2.0 if self.track_width > 900 else 1.5
            steering += heading_error * heading_gain

            steering = max(-WALL_FOLLOW_MAX_STEER, min(WALL_FOLLOW_MAX_STEER, steering))
            corner_speed = SPEED_CORNER if self.track_width <= 700 else (
                SPEED_OPEN_CORNER if self._is_open else SPEED_CORNER)
            robot.set_speed(corner_speed)
            robot.set_steering(steering)
            if self._corner_approach_ticks == 0:
                self._corner_entry_heading = None
                self.state = State.CORNER_TURN
            return

        # Corner detection via color sensor (line at section boundary)
        if self._corner_line_detected:
            self._corner_line_detected = False
            self._corner_entry_heading = sensors.imu_heading
            self._corner_entry_distance = robot.distance_traveled
            # More time for wide tracks to correct heading before turn
            self._corner_approach_ticks = 20 if self.track_width > 900 else 15
            return

        # Check for pillars (only in front hemisphere, within avoidance range)
        if sensors.pillars_visible and track.challenge_type == 'obstacle':
            closest = sensors.pillars_visible[0]
            if closest['distance'] < 600 and abs(closest['angle']) < 60:
                self.avoiding_pillar = closest
                # Red pillar: pass on RIGHT → steer so pillar is on LEFT
                # Green pillar: pass on LEFT → steer so pillar is on RIGHT
                if closest['color'] == 'red':
                    self.avoidance_side = -1  # Steer left of pillar (pass on right)
                else:
                    self.avoidance_side = 1   # Steer right of pillar (pass on left)
                self.pillar_passed = False
                self.avoidance_timer = 0
                self.state = State.PILLAR_AVOIDANCE
                return

        # PD wall following - keep centered between walls.
        # If either sensor reads beyond track width, it's looking through a
        # corner opening — the reading is meaningless for centering, so fall
        # back to heading-only control until both readings are valid.
        tof_l = sensors.tof_left
        tof_r = sensors.tof_right
        sensors_valid = tof_l < self.track_width and tof_r < self.track_width

        if sensors_valid:
            error = tof_r - tof_l
            derivative = error - self.prev_wall_error
            self.prev_wall_error = error
            steering = error * KP_WALL + derivative * KD_WALL
        else:
            self.prev_wall_error = 0
            steering = 0.0

        # Heading correction for ALL track widths — prevents angular oscillation.
        # Dominant term when sensors are invalid (near corners).
        heading = sensors.imu_heading
        cardinal = (round(heading / 90) * 90) % 360
        heading_error = _angle_diff(cardinal, heading)
        kp_heading = 0.5
        steering += heading_error * kp_heading

        steering = max(-WALL_FOLLOW_MAX_STEER, min(WALL_FOLLOW_MAX_STEER, steering))

        cruise = SPEED_OPEN_CRUISE if self._is_open else SPEED_CRUISE
        robot.set_speed(cruise)
        robot.set_steering(steering)

    def _handle_pillar_avoidance(self, sensors, robot, track):
        """Avoid a pillar by steering to the correct side with a simple offset approach."""
        self.avoidance_timer += 1

        # Timeout: if we've been avoiding for too long, go back to wall following
        if self.avoidance_timer > 90:  # ~3 seconds at 30Hz
            self.state = State.WALL_FOLLOWING
            self.avoiding_pillar = None
            return

        # If a corner entry line was detected during avoidance, hand off
        if self._corner_line_detected:
            self._corner_line_detected = False
            self._corner_entry_heading = sensors.imu_heading
            self._corner_entry_distance = robot.distance_traveled
            self.state = State.CORNER_TURN
            self.avoiding_pillar = None
            return

        # Simple approach: steer a fixed offset in the avoidance direction
        # avoidance_side: -1 = steer left (red pillar, pass right)
        #                 +1 = steer right (green pillar, pass left)
        steer_amount = self.avoidance_side * 14  # Moderate steering angle

        # Also do basic wall following to avoid hitting walls
        wall_correction = 0
        if sensors.tof_left < 120:
            wall_correction = 8   # Push right
        elif sensors.tof_right < 120:
            wall_correction = -8  # Push left

        steering = steer_amount + wall_correction
        steering = max(-WALL_FOLLOW_MAX_STEER, min(WALL_FOLLOW_MAX_STEER, steering))

        robot.set_speed(SPEED_PILLAR)
        robot.set_steering(steering)

        # Check if the pillar we were avoiding is now behind us
        pillar_behind = False
        if self.avoiding_pillar and 'pillar_ref' in self.avoiding_pillar:
            ref = self.avoiding_pillar['pillar_ref']
            dx = ref.x - robot.x
            dy = ref.y - robot.y
            angle_to = math.degrees(math.atan2(dy, dx))
            relative = (angle_to - robot.angle + 180) % 360 - 180
            if abs(relative) > 100:  # Pillar is behind us
                pillar_behind = True

        if pillar_behind or self.avoidance_timer > 45:
            # Transition back but keep steering briefly to clear
            if self.avoidance_timer > 50 or pillar_behind:
                self.state = State.WALL_FOLLOWING
                self.avoiding_pillar = None

    def _handle_corner_turn(self, sensors, robot, track):
        """Navigate a 90° corner using bicycle model with dynamic radius.

        At corner entry, the turn radius is set based on the distance to the
        inner wall (measured by the ToF sensor on the turn side).  This keeps
        the exit position away from the inner wall even when pillar avoidance
        shifted the robot off-center before the corner.

        Steering angle: δ = atan(wheelbase / R)
        IMU tracks heading change; exit when turned ≥ CORNER_MIN_EXIT_ANGLE.
        """
        # Slow down more for narrow corners to reduce angular overshoot
        if self._is_open:
            corner_speed = SPEED_CORNER if self.track_width <= 700 else SPEED_OPEN_CORNER
        else:
            corner_speed = SPEED_CORNER
        robot.set_speed(corner_speed)
        turn_sign = 1 if self.driving_direction == 1 else -1

        if self._corner_entry_heading is None:
            self._corner_entry_heading = sensors.imu_heading
            self._corner_entry_distance = robot.distance_traveled

            # Dynamic radius: distance from robot center to inner wall
            # CW right turns → inner wall is to the right (tof_right)
            # CCW left turns → inner wall is to the left (tof_left)
            if turn_sign == 1:
                inner_dist = sensors.tof_right + ROBOT_WIDTH / 2
            else:
                inner_dist = sensors.tof_left + ROBOT_WIDTH / 2

            # Radius = inner wall distance minus safety margin.
            # After a 90° turn, the robot front sticks out ROBOT_LENGTH/2
            # past the turn exit point, so margin must account for that
            # plus extra for servo lag and IMU drift.
            margin = ROBOT_LENGTH + 30  # 140mm body + 30mm dynamics buffer
            max_r = track.track_width / 2 - ROBOT_LENGTH / 2 - 30
            self._corner_radius = max(200, min(inner_dist - margin, max_r))

        heading_turned = abs(_angle_diff(sensors.imu_heading, self._corner_entry_heading))
        arc_traveled = robot.distance_traveled - self._corner_entry_distance
        arc_limit = self._corner_radius * math.pi  # Full semicircle as safety

        # Narrow tracks need earlier exit to account for angular overshoot
        exit_angle = CORNER_MIN_EXIT_ANGLE
        if self.track_width <= 700:
            exit_angle = 78  # ~12° overshoot → actual ~90°

        if heading_turned >= exit_angle:
            # Normal exit — counter-steer briefly to kill angular momentum
            cruise = SPEED_OPEN_CRUISE if self._is_open else SPEED_CRUISE
            if self._is_open and self.laps_completed >= 3:
                self.state = State.STOP_SECTION
            else:
                self.state = State.WALL_FOLLOWING
            robot.set_speed(cruise)
            robot.set_steering(-turn_sign * 5)  # Counter-steer
            self._corner_entry_heading = None
            self._corner_cooldown = 30
        elif arc_traveled > arc_limit:
            # Safety: driven too far without reaching exit angle
            cruise = SPEED_OPEN_CRUISE if self._is_open else SPEED_CRUISE
            if self._is_open and self.laps_completed >= 3:
                self.state = State.STOP_SECTION
            else:
                self.state = State.WALL_FOLLOWING
            robot.set_speed(cruise)
            robot.set_steering(0)
            self._corner_entry_heading = None
            self._corner_cooldown = 30
        else:
            steer_angle = math.degrees(math.atan(WHEELBASE / self._corner_radius))
            robot.set_steering(turn_sign * steer_angle)

    def _handle_three_point_turn(self, sensors, robot, track):
        """Execute a three-point turn to reverse direction.

        Phase 1: Forward turn in current corner direction (~80°)
        Phase 2: Reverse turn in opposite direction (~80°)
        Phase 3: Forward turn to complete ~180° total
        """
        heading = sensors.imu_heading
        turn_sign = 1 if self.driving_direction == 1 else -1

        if self.three_point_phase == ThreePointPhase.TURN_RIGHT_FORWARD:
            robot.set_speed(SPEED_THREE_POINT)
            robot.set_steering(turn_sign * MAX_STEERING_ANGLE)

            heading_diff = abs(_angle_diff(heading, self.three_point_start_heading))
            if heading_diff > 80:
                self.three_point_phase = ThreePointPhase.TURN_LEFT_REVERSE
                self.three_point_target_heading = heading

        elif self.three_point_phase == ThreePointPhase.TURN_LEFT_REVERSE:
            robot.set_speed(-SPEED_THREE_POINT)
            robot.set_steering(-turn_sign * MAX_STEERING_ANGLE)

            heading_diff = abs(_angle_diff(heading, self.three_point_target_heading))
            if heading_diff > 80 or sensors.tof_rear < 150:
                self.three_point_phase = ThreePointPhase.TURN_RIGHT_FORWARD_2
                self.three_point_target_heading = heading

        elif self.three_point_phase == ThreePointPhase.TURN_RIGHT_FORWARD_2:
            robot.set_speed(SPEED_THREE_POINT)
            robot.set_steering(turn_sign * MAX_STEERING_ANGLE * 0.5)

            total_turn = abs(_angle_diff(heading, self.three_point_start_heading))
            if total_turn > 160:
                self.three_point_phase = ThreePointPhase.COMPLETE
                self.driving_direction *= -1
                self.needs_direction_change = False
                self.direction_changed = True
                self.state = State.WALL_FOLLOWING
                robot.set_steering(0)

    def _handle_parking_approach(self, sensors, robot, track):
        """Approach the parking lot after completing 3 laps."""
        if track.parking_lot is None:
            self.state = State.FINISHED
            return

        # Slow down and look for parking lot
        robot.set_speed(SPEED_PARKING)

        error = sensors.tof_right - sensors.tof_left
        derivative = error - self.prev_wall_error
        self.prev_wall_error = error

        steering = error * KP_WALL * 0.5 + derivative * KD_WALL * 0.5
        robot.set_steering(steering)

        # Check if we're in the parking section
        # Use ToF sensors to detect the gap created by parking markers
        pl = track.parking_lot

        dist_to_parking = math.sqrt((robot.x - (pl.x + pl.width/2))**2 + 
                                     (robot.y - (pl.y + pl.length/2))**2)

        if dist_to_parking < 400:
            self.parking_start_heading = sensors.imu_heading
            self.state = State.PARKING_EXECUTE
            self.parking_phase = ParkingPhase.STEER_IN

    def _handle_parking_execute(self, sensors, robot, track):
        """Execute parallel parking maneuver."""
        if self.parking_phase == ParkingPhase.STEER_IN:
            # Steer toward the parking spot (toward outer wall)
            robot.set_speed(SPEED_PARKING * 0.5)

            # Determine which direction to steer based on parking position
            pl = track.parking_lot
            if pl.section_idx == 0:  # Top - park against top wall
                robot.set_steering(-15 * self.driving_direction)
            elif pl.section_idx == 1:  # Right - park against right wall
                robot.set_steering(-15 * self.driving_direction)
            elif pl.section_idx == 2:  # Bottom
                robot.set_steering(15 * self.driving_direction)
            else:  # Left
                robot.set_steering(15 * self.driving_direction)

            # Check if we're close to the outer wall
            if sensors.tof_front < 200 or sensors.tof_right < 100 or sensors.tof_left < 100:
                self.parking_phase = ParkingPhase.STRAIGHTEN

        elif self.parking_phase == ParkingPhase.STRAIGHTEN:
            heading_diff = _angle_diff(sensors.imu_heading, self.parking_start_heading)
            robot.set_speed(SPEED_PARKING * 0.3)

            if abs(heading_diff) > 5:
                robot.set_steering(-heading_diff * 0.3)
            else:
                robot.set_steering(0)
                self.parking_phase = ParkingPhase.FINAL_ADJUST

        elif self.parking_phase == ParkingPhase.FINAL_ADJUST:
            # Small adjustments
            self.parking_timer += 1
            if self.parking_timer > 30:  # ~1 second
                robot.stop()
                self.parking_phase = ParkingPhase.COMPLETE
                self.state = State.FINISHED

    def _handle_stop_section(self, sensors, robot, track):
        """Wall follow at reduced speed, verify ToF matches starting section, then stop."""
        if self._corner_cooldown > 0:
            self._corner_cooldown -= 1
            cruise = SPEED_PARKING
            robot.set_speed(cruise)
            error = sensors.tof_right - sensors.tof_left
            correction = error * KP_WALL * 0.3
            heading = sensors.imu_heading
            cardinal = (round(heading / 90) * 90) % 360
            heading_error = _angle_diff(cardinal, heading)
            correction += heading_error * 0.5
            correction = max(-WALL_FOLLOW_MAX_STEER, min(WALL_FOLLOW_MAX_STEER, correction))
            robot.set_steering(correction)
            return

        if self._corner_line_detected:
            self._corner_line_detected = False
            self._corner_entry_heading = sensors.imu_heading
            self._corner_entry_distance = robot.distance_traveled
            self._corner_approach_ticks = 15
            self.state = State.CORNER_TURN
            return

        tof_l = sensors.tof_left
        tof_r = sensors.tof_right
        sensors_valid = tof_l < self.track_width and tof_r < self.track_width

        if sensors_valid:
            error = tof_r - tof_l
            derivative = error - self.prev_wall_error
            self.prev_wall_error = error
            steering = error * KP_WALL + derivative * KD_WALL
        else:
            self.prev_wall_error = 0
            steering = 0.0

        heading = sensors.imu_heading
        cardinal = (round(heading / 90) * 90) % 360
        heading_error = _angle_diff(cardinal, heading)
        steering += heading_error * 0.5

        steering = max(-WALL_FOLLOW_MAX_STEER, min(WALL_FOLLOW_MAX_STEER, steering))
        robot.set_speed(SPEED_PARKING)
        robot.set_steering(steering)

        if self.start_tof_front is not None and self.start_tof_rear is not None:
            front_match = abs(sensors.tof_front - self.start_tof_front) < self.stop_section_tolerance
            rear_match = abs(sensors.tof_rear - self.start_tof_rear) < self.stop_section_tolerance
            if front_match and rear_match:
                robot.stop()
                self.state = State.FINISHED

    def get_state_name(self) -> str:
        """Get human-readable state name."""
        return self.state.name

    def get_score(self, track, robot_x=None, robot_y=None, robot_angle=None) -> dict:
        """Calculate current score based on rules."""
        score = {
            'sections': min(self.sections_passed, MAX_SECTION_POINTS),
            'laps': min(self.laps_completed, MAX_LAP_POINTS),
            'finish_section': 0,
            'obstacle_bonus': 0,
            'parking_start': 0,
            'parking_result': 0,
            'total': 0,
        }

        if self.laps_completed >= 3 and self.state in (State.FINISHED, State.STOPPED):
            score['finish_section'] = POINTS_FINISH_SECTION

        if track.challenge_type == 'obstacle':
            signs_moved = any(p.is_outside_circle() for p in track.pillars)

            if self.laps_completed >= 3:
                if signs_moved:
                    score['obstacle_bonus'] = POINTS_3LAPS_MOVED
                else:
                    score['obstacle_bonus'] = POINTS_3LAPS_NOT_MOVED
            elif self.laps_completed >= 1:
                if signs_moved:
                    score['obstacle_bonus'] = POINTS_OBSTACLE_MOVED
                else:
                    score['obstacle_bonus'] = POINTS_OBSTACLE_NOT_MOVED

            if self.start_in_parking and self.laps_completed >= 1:
                score['parking_start'] = POINTS_START_IN_PARKING

            if self.state == State.FINISHED and track.parking_lot and robot_x is not None:
                parking_status = track.check_parking(
                    robot_x, robot_y, robot_angle, ROBOT_LENGTH, ROBOT_WIDTH
                )
                if parking_status == 'fully_parked':
                    score['parking_result'] = POINTS_PARKED_FULLY
                elif parking_status == 'partly_parked':
                    score['parking_result'] = POINTS_PARKED_PARTLY

        score['total'] = (score['sections'] + score['laps'] +
                         score['finish_section'] + score['obstacle_bonus'] +
                         score['parking_start'] + score['parking_result'])

        return score
