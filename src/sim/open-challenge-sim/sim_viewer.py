"""
WRO 2026 Future Engineers — Real-time 2D Simulation Viewer
A polished Pygame-based visualizer for the autonomous driving simulation.

Controls:
  SPACE       Pause / Resume
  R           Restart current config
  +/=         Speed up (2x, 4x, 8x, 16x)
  -           Slow down
  RIGHT       Next configuration
  LEFT        Previous configuration
  A           Auto-play (cycle all configs)
  T           Toggle sensor ray display
  P           Toggle path trail
  G           Toggle grid overlay
  S           Screenshot (saved to screenshots/)
  Q / ESC     Quit
"""

import sys
import math
import os
import time
import pygame
from config import *
from track import Track
from robot import Robot
from controller import Controller, State

# ─── Display constants ───────────────────────────────────────────────────────
TRACK_AREA_PX = 800
PANEL_WIDTH = 280
WINDOW_W = TRACK_AREA_PX + PANEL_WIDTH
WINDOW_H = TRACK_AREA_PX
SCALE = TRACK_AREA_PX / MAT_SIZE  # px per mm

FPS = 60

# ─── Colors (dark theme) ─────────────────────────────────────────────────────
BG_DARK = (18, 18, 24)
BG_PANEL = (28, 28, 38)
PANEL_BORDER = (50, 50, 65)
TEXT_PRIMARY = (230, 230, 240)
TEXT_SECONDARY = (150, 155, 170)
TEXT_ACCENT = (100, 200, 255)
TEXT_SUCCESS = (80, 220, 120)
TEXT_WARN = (255, 180, 50)
TEXT_FAIL = (255, 80, 80)
TRACK_BG = (35, 38, 48)
WALL_COLOR = (200, 205, 215)
WALL_INNER = (160, 165, 175)
LINE_ORANGE = (255, 150, 30)
LINE_BLUE = (40, 120, 255)
ROBOT_BODY = (60, 140, 255)
ROBOT_FRONT = (255, 210, 50)
SENSOR_RAY = (0, 255, 200, 60)
TRAIL_COLOR = (100, 160, 255, 100)
GRID_COLOR = (45, 48, 58)
CORNER_FILL = (42, 45, 55)
SECTION_HIGHLIGHT = (50, 55, 70)


def mm_to_px(x_mm, y_mm):
    """Convert mm coordinates to pixel coordinates."""
    return int(x_mm * SCALE), int(y_mm * SCALE)


def mm_to_px_f(x_mm, y_mm):
    """Convert mm to px, float version."""
    return x_mm * SCALE, y_mm * SCALE


class SimConfig:
    """A simulation configuration (width combo + direction + start + offset + angle)."""
    def __init__(self, name, widths, direction, start, offset=0, angle_offset=0):
        self.name = name
        self.widths = widths
        self.direction = direction
        self.start = start
        self.offset = offset
        self.angle_offset = angle_offset

    def label(self):
        d = "CW" if self.direction == 1 else "CCW"
        lbl = f"{self.name} | {d} | S{self.start}"
        if self.offset != 0:
            lbl += f" | {self.offset:+d}mm"
        if self.angle_offset != 0:
            lbl += f" | {self.angle_offset:+d}°"
        return lbl


def build_configs():
    """Build all test configurations: 48 centered + 24 offset cases."""
    width_configs = [
        ('All Wide', [1000, 1000, 1000, 1000]),
        ('All Narrow', [600, 600, 600, 600]),
        ('Alt W/N', [1000, 600, 1000, 600]),
        ('Alt N/W', [600, 1000, 600, 1000]),
        ('Top+Right Narrow', [600, 600, 1000, 1000]),
        ('One Narrow (top)', [600, 1000, 1000, 1000]),
    ]
    configs = []
    for name, widths in width_configs:
        for direction in [1, -1]:
            for start in range(4):
                configs.append(SimConfig(name, widths, direction, start))

    # Offset start cases — realistic placement tolerance
    offset_configs = [
        ('All Wide', [1000, 1000, 1000, 1000], 75),
        ('All Wide', [1000, 1000, 1000, 1000], -75),
        ('All Wide', [1000, 1000, 1000, 1000], 100),
        ('All Wide', [1000, 1000, 1000, 1000], -100),
        ('All Narrow', [600, 600, 600, 600], 50),
        ('All Narrow', [600, 600, 600, 600], -50),
        ('All Narrow', [600, 600, 600, 600], 75),
        ('All Narrow', [600, 600, 600, 600], -75),
        ('Alt W/N', [1000, 600, 1000, 600], 75),
        ('Alt W/N', [1000, 600, 1000, 600], -75),
        ('Alt N/W', [600, 1000, 600, 1000], 50),
        ('Alt N/W', [600, 1000, 600, 1000], -50),
    ]
    for name, widths, offset in offset_configs:
        for direction in [1, -1]:
            configs.append(SimConfig(name, widths, direction, 0, offset))

    # Angle offset start cases — realistic heading misalignment
    angle_configs = [
        ('All Wide', [1000, 1000, 1000, 1000]),
        ('All Narrow', [600, 600, 600, 600]),
        ('Alt W/N', [1000, 600, 1000, 600]),
        ('Alt N/W', [600, 1000, 600, 1000]),
    ]
    for name, widths in angle_configs:
        for direction in [1, -1]:
            for angle_off in [5, -5]:
                configs.append(SimConfig(name, widths, direction, 0, 0, angle_off))

    # Combined: lateral offset + angle offset (worst case)
    combined_configs = [
        ('All Wide', [1000, 1000, 1000, 1000], 75, 5),
        ('All Wide', [1000, 1000, 1000, 1000], -75, -5),
        ('All Narrow', [600, 600, 600, 600], 50, 5),
        ('All Narrow', [600, 600, 600, 600], -50, -5),
    ]
    for name, widths, offset, angle_off in combined_configs:
        for direction in [1, -1]:
            configs.append(SimConfig(name, widths, direction, 0, offset, angle_off))

    # Extreme worst-case: 12° angle + offset (real-world stress test)
    extreme_configs = [
        ('All Wide', [1000, 1000, 1000, 1000], 100, 12),
        ('All Wide', [1000, 1000, 1000, 1000], -100, -12),
        ('All Narrow', [600, 600, 600, 600], 75, 12),
        ('All Narrow', [600, 600, 600, 600], -75, -12),
        ('Alt W/N', [1000, 600, 1000, 600], 75, 12),
        ('Alt N/W', [600, 1000, 600, 1000], 75, 12),
    ]
    for name, widths, offset, angle_off in extreme_configs:
        for direction in [1, -1]:
            configs.append(SimConfig(name, widths, direction, 0, offset, angle_off))

    return configs


class Simulation:
    """Encapsulates one simulation run."""

    def __init__(self, config: SimConfig):
        self.config = config
        self.track = Track(
            challenge_type='open',
            section_widths=config.widths,
            driving_direction=config.direction,
            starting_section_idx=config.start,
        )

        sec = self.track.straight_sections[config.start]
        cx = (sec.x1 + sec.x2) / 2
        cy = (sec.y1 + sec.y2) / 2

        angles = {0: 0, 1: 90, 2: 180, 3: 270}
        if config.direction == -1:
            angles = {0: 180, 1: 270, 2: 0, 3: 90}
        angle = angles[config.start]

        # Apply lateral offset perpendicular to travel direction
        if config.offset != 0:
            offset_angle = math.radians(angle + 90)
            cx += config.offset * math.cos(offset_angle)
            cy += config.offset * math.sin(offset_angle)

        start_angle = angle + config.angle_offset
        self.robot = Robot(cx, cy, start_angle)
        self.controller = Controller(
            driving_direction=config.direction,
            challenge_type='open',
        )
        self.elapsed = 0.0
        self.trail = []
        self.finished = False
        self.result = None
        self.control_accum = 0.0

    def step(self, dt):
        """Advance simulation by dt seconds."""
        if self.finished:
            return

        self.control_accum += dt
        control_dt = 1.0 / CONTROL_HZ
        while self.control_accum >= control_dt:
            self.control_accum -= control_dt
            sensors = self.robot.get_sensors(self.track)
            self.controller.update(sensors, self.robot, self.track, control_dt)

        self.robot.update(dt)

        # Wall collision handling
        if self.robot.check_wall_collision(self.track):
            irx1, iry1, irx2, iry2 = self.track.inner_rect
            in_inner = irx1 <= self.robot.x <= irx2 and iry1 <= self.robot.y <= iry2
            if in_inner:
                icx = (irx1 + irx2) / 2
                icy = (iry1 + iry2) / 2
                dx = self.robot.x - icx
                dy = self.robot.y - icy
            else:
                ocx = (self.track.outer_rect[0] + self.track.outer_rect[2]) / 2
                ocy = (self.track.outer_rect[1] + self.track.outer_rect[3]) / 2
                dx = ocx - self.robot.x
                dy = ocy - self.robot.y
            dist = max(1, math.sqrt(dx * dx + dy * dy))
            self.robot.x += (dx / dist) * 20
            self.robot.y += (dy / dist) * 20
            self.robot.speed = max(abs(self.robot.speed) * 0.8, MAX_SPEED * 0.30)
            if self.robot.target_speed < 0:
                self.robot.speed = -self.robot.speed

        self.elapsed += dt
        self.trail.append((self.robot.x, self.robot.y))
        if len(self.trail) > 3000:
            self.trail = self.trail[-2000:]

        if self.controller.state in (State.FINISHED, State.STOPPED):
            self.finished = True
            self.result = 'PASS' if self.controller.laps_completed >= 3 else 'TIMEOUT'

        if self.elapsed >= ROUND_TIME:
            self.finished = True
            self.result = 'PASS' if self.controller.laps_completed >= 3 else 'TIMEOUT'


class Viewer:
    """Pygame-based 2D simulation viewer."""

    def __init__(self):
        pygame.init()
        pygame.display.set_caption("Artemis WRO 2026 — Open Challenge Sim")
        self.screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
        self.clock = pygame.time.Clock()

        # Fonts
        self.font_lg = pygame.font.SysFont("JetBrains Mono, Menlo, monospace", 18, bold=True)
        self.font_md = pygame.font.SysFont("JetBrains Mono, Menlo, monospace", 14)
        self.font_sm = pygame.font.SysFont("JetBrains Mono, Menlo, monospace", 11)

        # State
        self.configs = build_configs()
        self.config_idx = 0
        self.sim = None
        self.paused = False
        self.speed_mult = 1
        self.show_sensors = True
        self.show_trail = True
        self.show_grid = False
        self.show_list = False
        self.list_scroll = 0
        self.running = True
        self.auto_play = False
        self.batch_results = {}  # config_idx -> 'PASS'/'FAIL'

        # Surfaces
        self.track_surface = None

        self._start_config(0)

    def _start_config(self, idx):
        """Start a new simulation with the given config index."""
        self.config_idx = idx % len(self.configs)
        self.sim = Simulation(self.configs[self.config_idx])
        self.track_surface = self._render_track()
        self.paused = False

    def _render_track(self):
        """Pre-render the static track onto a surface."""
        surf = pygame.Surface((TRACK_AREA_PX, TRACK_AREA_PX))
        surf.fill(BG_DARK)

        track = self.sim.track

        # Grid
        if self.show_grid:
            for i in range(0, MAT_SIZE + 1, 500):
                px = int(i * SCALE)
                pygame.draw.line(surf, GRID_COLOR, (px, 0), (px, TRACK_AREA_PX), 1)
                pygame.draw.line(surf, GRID_COLOR, (0, px), (TRACK_AREA_PX, px), 1)

        # Track surface (between walls)
        orx1, ory1, orx2, ory2 = track.outer_rect
        p1 = mm_to_px(orx1, ory1)
        p2 = mm_to_px(orx2 - orx1, ory2 - ory1)
        pygame.draw.rect(surf, TRACK_BG, (p1[0], p1[1], p2[0], p2[1]))

        # Inner fill (non-track area)
        irx1, iry1, irx2, iry2 = track.inner_rect
        p1 = mm_to_px(irx1, iry1)
        p2 = mm_to_px(irx2 - irx1, iry2 - iry1)
        pygame.draw.rect(surf, BG_DARK, (p1[0], p1[1], p2[0], p2[1]))

        # Corner section fills
        for section in track.corner_sections:
            p1 = mm_to_px(section.x1, section.y1)
            w = int((section.x2 - section.x1) * SCALE)
            h = int((section.y2 - section.y1) * SCALE)
            pygame.draw.rect(surf, CORNER_FILL, (p1[0], p1[1], w, h))

        # Outer walls
        for x1, y1, x2, y2 in track.walls_outer:
            pygame.draw.line(surf, WALL_COLOR, mm_to_px(x1, y1), mm_to_px(x2, y2), 3)

        # Inner walls
        for x1, y1, x2, y2 in track.walls_inner:
            pygame.draw.line(surf, WALL_INNER, mm_to_px(x1, y1), mm_to_px(x2, y2), 2)

        # Orange/blue lines
        for line in track.orange_blue_lines:
            lx1, ly1, lx2, ly2 = line['pos']
            color = LINE_ORANGE if line['color'] == 'orange' else LINE_BLUE
            pygame.draw.line(surf, color, mm_to_px(lx1, ly1), mm_to_px(lx2, ly2), 3)

        # Section width labels
        labels = ['T', 'R', 'B', 'L']
        positions = [
            (MAT_SIZE / 2, WALL_OFFSET / 2),
            (MAT_SIZE - WALL_OFFSET / 2, MAT_SIZE / 2),
            (MAT_SIZE / 2, MAT_SIZE - WALL_OFFSET / 2),
            (WALL_OFFSET / 2, MAT_SIZE / 2),
        ]
        for i, (lbl, (mx, my)) in enumerate(zip(labels, positions)):
            w = track.section_widths[i]
            text = self.font_sm.render(f"{lbl}:{w}", True, TEXT_SECONDARY)
            px, py = mm_to_px(mx, my)
            surf.blit(text, (px - text.get_width() // 2, py - text.get_height() // 2))

        return surf

    def _draw_robot(self, surf):
        """Draw the robot with heading indicator."""
        robot = self.sim.robot
        cx, cy = mm_to_px_f(robot.x, robot.y)
        angle_rad = math.radians(robot.angle)

        # Robot body corners
        hl = robot.length / 2 * SCALE
        hw = robot.width / 2 * SCALE
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)

        corners = [
            (cx + hl * cos_a - hw * sin_a, cy + hl * sin_a + hw * cos_a),
            (cx + hl * cos_a + hw * sin_a, cy + hl * sin_a - hw * cos_a),
            (cx - hl * cos_a + hw * sin_a, cy - hl * sin_a - hw * cos_a),
            (cx - hl * cos_a - hw * sin_a, cy - hl * sin_a + hw * cos_a),
        ]

        # Body
        pygame.draw.polygon(surf, ROBOT_BODY, corners)
        pygame.draw.polygon(surf, (120, 190, 255), corners, 2)

        # Front edge highlight
        pygame.draw.line(surf, ROBOT_FRONT,
                         (int(corners[0][0]), int(corners[0][1])),
                         (int(corners[1][0]), int(corners[1][1])), 3)

        # Heading arrow
        head_len = 30
        hx = cx + head_len * cos_a
        hy = cy + head_len * sin_a
        pygame.draw.line(surf, ROBOT_FRONT, (int(cx), int(cy)), (int(hx), int(hy)), 2)
        # Arrowhead
        arr_size = 6
        arr_angle = math.pi / 6
        for sign in [1, -1]:
            ax = hx - arr_size * math.cos(angle_rad + sign * arr_angle)
            ay = hy - arr_size * math.sin(angle_rad + sign * arr_angle)
            pygame.draw.line(surf, ROBOT_FRONT, (int(hx), int(hy)), (int(ax), int(ay)), 2)

    def _draw_sensors(self, surf):
        """Draw ToF sensor rays."""
        if not self.show_sensors:
            return

        robot = self.sim.robot
        sensors = robot.get_sensors(self.sim.track)
        cx, cy = mm_to_px_f(robot.x, robot.y)

        ray_info = [
            (0, sensors.tof_front, (0, 255, 180)),
            (180, sensors.tof_rear, (255, 100, 100)),
            (-90, sensors.tof_left, (180, 100, 255)),
            (90, sensors.tof_right, (255, 200, 50)),
        ]

        for rel_angle, dist, color in ray_info:
            angle_rad = math.radians(robot.angle + rel_angle)
            # Start from robot edge
            if rel_angle == 0:
                sx = cx + (robot.length / 2 * SCALE) * math.cos(angle_rad)
                sy = cy + (robot.length / 2 * SCALE) * math.sin(angle_rad)
            elif rel_angle == 180:
                sx = cx - (robot.length / 2 * SCALE) * math.cos(math.radians(robot.angle))
                sy = cy - (robot.length / 2 * SCALE) * math.sin(math.radians(robot.angle))
            elif rel_angle == -90:
                perp = math.radians(robot.angle - 90)
                sx = cx + (robot.width / 2 * SCALE) * math.cos(perp)
                sy = cy + (robot.width / 2 * SCALE) * math.sin(perp)
            else:
                perp = math.radians(robot.angle + 90)
                sx = cx + (robot.width / 2 * SCALE) * math.cos(perp)
                sy = cy + (robot.width / 2 * SCALE) * math.sin(perp)

            # End point
            d_px = min(dist, 1500) * SCALE  # Cap visual length
            ex = sx + d_px * math.cos(angle_rad)
            ey = sy + d_px * math.sin(angle_rad)

            # Draw ray with glow
            ray_surf = pygame.Surface((TRACK_AREA_PX, TRACK_AREA_PX), pygame.SRCALPHA)
            pygame.draw.line(ray_surf, (*color, 40), (int(sx), int(sy)), (int(ex), int(ey)), 3)
            pygame.draw.line(ray_surf, (*color, 100), (int(sx), int(sy)), (int(ex), int(ey)), 1)
            surf.blit(ray_surf, (0, 0))

            # Endpoint dot
            pygame.draw.circle(surf, color, (int(ex), int(ey)), 3)
            pygame.draw.circle(surf, (*color, 60), (int(ex), int(ey)), 6, 1)

    def _draw_trail(self, surf):
        """Draw robot path trail."""
        if not self.show_trail or len(self.sim.trail) < 2:
            return

        trail_surf = pygame.Surface((TRACK_AREA_PX, TRACK_AREA_PX), pygame.SRCALPHA)
        points = [(int(x * SCALE), int(y * SCALE)) for x, y in self.sim.trail]

        # Draw as fading line segments
        n = len(points)
        step = max(1, n // 500)  # Limit drawn points for performance
        for i in range(0, n - step, step):
            alpha = int(40 + 80 * (i / n))
            color = (100, 160, 255, alpha)
            pygame.draw.line(trail_surf, color, points[i], points[min(i + step, n - 1)], 1)

        surf.blit(trail_surf, (0, 0))

    def _draw_panel(self):
        """Draw the telemetry/info panel on the right side."""
        panel_x = TRACK_AREA_PX
        pygame.draw.rect(self.screen, BG_PANEL, (panel_x, 0, PANEL_WIDTH, WINDOW_H))
        pygame.draw.line(self.screen, PANEL_BORDER, (panel_x, 0), (panel_x, WINDOW_H), 1)

        x = panel_x + 15
        y = 15

        # Title
        title = self.font_lg.render("ARTEMIS SIM", True, TEXT_ACCENT)
        self.screen.blit(title, (x, y))
        y += 30

        # Config info
        cfg = self.sim.config
        self._panel_label(x, y, "CONFIG", f"{self.config_idx + 1}/{len(self.configs)}")
        y += 20
        self._panel_text(x, y, cfg.label(), TEXT_PRIMARY)
        y += 20
        widths_str = f"W: [{cfg.widths[0]},{cfg.widths[1]},{cfg.widths[2]},{cfg.widths[3]}]"
        self._panel_text(x, y, widths_str, TEXT_SECONDARY)
        y += 30

        # Separator
        pygame.draw.line(self.screen, PANEL_BORDER, (x, y), (x + PANEL_WIDTH - 30, y), 1)
        y += 15

        # Time
        elapsed = self.sim.elapsed
        remaining = max(0, ROUND_TIME - elapsed)
        time_color = TEXT_WARN if remaining < 30 else TEXT_PRIMARY
        self._panel_label(x, y, "TIME", f"{elapsed:.1f}s / {ROUND_TIME}s")
        y += 20

        # Progress bar
        progress = min(1.0, elapsed / ROUND_TIME)
        bar_w = PANEL_WIDTH - 30
        pygame.draw.rect(self.screen, (40, 42, 55), (x, y, bar_w, 6), border_radius=3)
        fill_w = int(progress * bar_w)
        bar_color = TEXT_WARN if remaining < 30 else TEXT_ACCENT
        if fill_w > 0:
            pygame.draw.rect(self.screen, bar_color, (x, y, fill_w, 6), border_radius=3)
        y += 20

        # State
        state_name = self.sim.controller.get_state_name()
        if state_name == 'FINISHED':
            state_color = TEXT_SUCCESS
        elif state_name == 'STOP_SECTION':
            state_color = TEXT_WARN
        else:
            state_color = TEXT_PRIMARY
        self._panel_label(x, y, "STATE", state_name, value_color=state_color)
        y += 22

        # Laps
        laps = self.sim.controller.laps_completed
        lap_color = TEXT_SUCCESS if laps >= 3 else TEXT_PRIMARY
        self._panel_label(x, y, "LAPS", f"{laps} / 3", value_color=lap_color)
        y += 22

        # Sections
        sections = self.sim.controller.sections_passed
        self._panel_label(x, y, "SECTIONS", str(sections))
        y += 22

        # Speed
        speed = abs(self.sim.robot.speed)
        self._panel_label(x, y, "SPEED", f"{speed:.0f} mm/s")
        y += 22

        # Heading
        heading = self.sim.robot.imu_heading
        self._panel_label(x, y, "HEADING", f"{heading:.1f}°")
        y += 22

        # Position
        self._panel_label(x, y, "POS",
                          f"({self.sim.robot.x:.0f}, {self.sim.robot.y:.0f})")
        y += 30

        # Separator
        pygame.draw.line(self.screen, PANEL_BORDER, (x, y), (x + PANEL_WIDTH - 30, y), 1)
        y += 15

        # Sensor readings
        sensors = self.sim.robot.get_sensors(self.sim.track)
        self._panel_label(x, y, "SENSORS", "")
        y += 18
        self._panel_label(x, y, "  ToF L", f"{sensors.tof_left:.0f}mm")
        y += 16
        self._panel_label(x, y, "  ToF R", f"{sensors.tof_right:.0f}mm")
        y += 16
        self._panel_label(x, y, "  ToF F", f"{sensors.tof_front:.0f}mm")
        y += 16
        self._panel_label(x, y, "  ToF Rear", f"{sensors.tof_rear:.0f}mm")
        y += 16

        color_display = sensors.color_detected if sensors.color_detected else "none"
        color_color = LINE_ORANGE if sensors.color_detected == 'orange' else (LINE_BLUE if sensors.color_detected == 'blue' else TEXT_SECONDARY)
        self._panel_label(x, y, "  Color", color_display, value_color=color_color)
        y += 18

        # Start ToF reference (shown after capture)
        controller = self.sim.controller
        if controller.start_tof_front is not None:
            self._panel_label(x, y, "  Start F", f"{controller.start_tof_front:.0f}mm",
                              value_color=TEXT_SECONDARY)
            y += 16
            self._panel_label(x, y, "  Start R", f"{controller.start_tof_rear:.0f}mm",
                              value_color=TEXT_SECONDARY)
            y += 16

        # Decision info
        self._panel_label(x, y, "DECISION", "")
        y += 18

        state_name = controller.get_state_name()

        # Add decision based on state
        decision = ""
        if state_name == "WALL_FOLLOWING":
            wall_error = sensors.tof_right - sensors.tof_left
            decision = f"Wall error: {wall_error:+.0f}mm"
        elif state_name == "CORNER_TURN":
            if controller._corner_approach_ticks > 0:
                decision = f"Approach ({controller._corner_approach_ticks})"
            else:
                decision = "Turning..."
        elif state_name == "PILLAR_AVOIDANCE":
            decision = f"Avoid ({controller.avoidance_side:+d})"
        elif state_name == "THREE_POINT_TURN":
            decision = f"3PT: {controller.three_point_phase.name}"
        elif state_name == "PARKING_APPROACH":
            decision = "Park approach"
        elif state_name == "PARKING_EXECUTE":
            decision = f"Park: {controller.parking_phase.name}"
        elif state_name == "STOP_SECTION":
            if controller.start_tof_front is not None:
                f_ok = abs(sensors.tof_front - controller.start_tof_front) < controller.stop_section_tolerance
                r_ok = abs(sensors.tof_rear - controller.start_tof_rear) < controller.stop_section_tolerance
                f_sym = "Y" if f_ok else "N"
                r_sym = "Y" if r_ok else "N"
                decision = f"Verify F:{f_sym} R:{r_sym}"
            else:
                decision = "Stopping..."

        if decision:
            self._panel_text(x, y, decision, TEXT_SECONDARY)
            y += 16

        # Separator
        pygame.draw.line(self.screen, PANEL_BORDER, (x, y), (x + PANEL_WIDTH - 30, y), 1)
        y += 15

        # Sim speed
        speed_str = f"{self.speed_mult}x" if not self.paused else "PAUSED"
        speed_color = TEXT_WARN if self.paused else TEXT_ACCENT
        self._panel_label(x, y, "SIM SPEED", speed_str, value_color=speed_color)
        y += 22

        # Result
        if self.sim.finished:
            result_color = TEXT_SUCCESS if self.sim.result == 'PASS' else TEXT_FAIL
            self._panel_label(x, y, "RESULT", self.sim.result, value_color=result_color)
            y += 22

        # Batch results (if auto-play has run)
        if self.batch_results:
            y += 5
            pygame.draw.line(self.screen, PANEL_BORDER, (x, y), (x + PANEL_WIDTH - 30, y), 1)
            y += 10
            passed = sum(1 for v in self.batch_results.values() if v == 'PASS')
            total = len(self.batch_results)
            batch_color = TEXT_SUCCESS if passed == total else TEXT_WARN
            self._panel_label(x, y, "BATCH", f"{passed}/{total} passed", value_color=batch_color)
            y += 20

            # Mini grid: 48 cells showing pass/fail
            cell_size = 8
            cols = 12
            for i in range(len(self.configs)):
                col = i % cols
                row = i // cols
                cx_cell = x + col * (cell_size + 2)
                cy_cell = y + row * (cell_size + 2)
                if i in self.batch_results:
                    c = TEXT_SUCCESS if self.batch_results[i] == 'PASS' else TEXT_FAIL
                elif i == self.config_idx:
                    c = TEXT_ACCENT
                else:
                    c = (45, 48, 58)
                pygame.draw.rect(self.screen, c, (cx_cell, cy_cell, cell_size, cell_size))
            y += (len(self.configs) // cols + 1) * (cell_size + 2) + 10

        # Auto-play indicator
        if self.auto_play:
            y += 5
            auto_text = self.font_md.render("AUTO-PLAY ACTIVE", True, TEXT_ACCENT)
            self.screen.blit(auto_text, (x, y))
            y += 25

        # Controls help at bottom
        y = WINDOW_H - 195
        pygame.draw.line(self.screen, PANEL_BORDER, (x, y), (x + PANEL_WIDTH - 30, y), 1)
        y += 10
        controls = [
            "SPACE  Pause/Resume",
            "R      Restart",
            "+/-    Speed",
            "←/→    Prev/Next config",
            "A      Auto-play all",
            "T      Toggle sensors",
            "P      Toggle trail",
            "G      Toggle grid",
            "Q      Quit",
        ]
        for line in controls:
            text = self.font_sm.render(line, True, TEXT_SECONDARY)
            self.screen.blit(text, (x, y))
            y += 16

    def _panel_label(self, x, y, label, value, value_color=None):
        """Draw a label: value pair on the panel."""
        lbl = self.font_sm.render(label, True, TEXT_SECONDARY)
        self.screen.blit(lbl, (x, y))
        val = self.font_md.render(str(value), True, value_color or TEXT_PRIMARY)
        self.screen.blit(val, (x + 90, y - 1))

    def _panel_text(self, x, y, text, color):
        """Draw text on the panel."""
        surf = self.font_md.render(text, True, color)
        self.screen.blit(surf, (x, y))

    def _draw_config_list(self):
        """Draw scrollable config list overlay."""
        margin = 40
        lw = WINDOW_W - margin * 2
        lh = WINDOW_H - margin * 2
        row_h = 22
        visible_rows = lh // row_h - 2

        overlay = pygame.Surface((lw, lh), pygame.SRCALPHA)
        overlay.fill((20, 20, 30, 240))
        pygame.draw.rect(overlay, PANEL_BORDER, (0, 0, lw, lh), 1)

        title = self.font_lg.render(
            f"Config List ({len(self.configs)} cases)  —  Click to jump, L to close",
            True, TEXT_SECONDARY)
        overlay.blit(title, (15, 8))

        # Section headers
        y = 36
        n_offset = 24
        n_angle = 16
        n_combined = 8
        n_extreme = len(self.configs) - 48 - n_offset - n_angle - n_combined
        sections = [
            (0, 48, "CENTERED (48 cases)"),
            (48, 48 + n_offset, "OFF-CENTER POSITION (24 cases)"),
            (48 + n_offset, 48 + n_offset + n_angle, "ANGLED START (16 cases)"),
            (48 + n_offset + n_angle, 48 + n_offset + n_angle + n_combined, "COMBINED OFFSET+ANGLE (8 cases)"),
            (48 + n_offset + n_angle + n_combined, len(self.configs), f"EXTREME 12° + OFFSET ({n_extreme} cases)"),
        ]

        row_idx = 0
        for sec_start, sec_end, sec_title in sections:
            if row_idx >= self.list_scroll and row_idx < self.list_scroll + visible_rows:
                hdr = self.font_md.render(sec_title, True, (100, 200, 255))
                overlay.blit(hdr, (15, y))
                y += row_h
            row_idx += 1

            for i in range(sec_start, sec_end):
                if row_idx < self.list_scroll:
                    row_idx += 1
                    continue
                if row_idx >= self.list_scroll + visible_rows:
                    row_idx += 1
                    continue

                cfg = self.configs[i]
                is_current = (i == self.config_idx)
                result = self.batch_results.get(i)

                if is_current:
                    pygame.draw.rect(overlay, (60, 60, 80), (10, y, lw - 20, row_h - 2))

                idx_text = self.font_sm.render(f"{i+1:3d}.", True, TEXT_SECONDARY)
                overlay.blit(idx_text, (15, y + 3))

                label_color = (255, 255, 255) if is_current else (180, 180, 180)
                label = self.font_sm.render(cfg.label(), True, label_color)
                overlay.blit(label, (50, y + 3))

                if result:
                    rc = (100, 255, 100) if result == 'PASS' else (255, 80, 80)
                    rt = self.font_sm.render(result, True, rc)
                    overlay.blit(rt, (lw - 60, y + 3))

                y += row_h
                row_idx += 1

        # Scrollbar
        total_rows = len(self.configs) + len(sections)
        if total_rows > visible_rows:
            sb_h = max(20, int(lh * visible_rows / total_rows))
            sb_y = int((lh - sb_h) * self.list_scroll / max(1, total_rows - visible_rows))
            pygame.draw.rect(overlay, (80, 80, 100), (lw - 8, sb_y, 6, sb_h), border_radius=3)

        self.screen.blit(overlay, (margin, margin))
        self._list_rect = (margin, margin, lw, lh)
        self._list_row_h = row_h
        self._list_visible_rows = visible_rows

    def _handle_list_click(self, pos):
        """Handle click on config list — jump to that config."""
        mx, my = pos
        margin = 40
        row_h = 22
        header_h = 36

        rel_y = my - margin - header_h
        if rel_y < 0:
            return

        clicked_row = rel_y // row_h + self.list_scroll
        n_offset = 24
        n_angle = 16
        n_combined = 8
        sections = [
            (0, 48),
            (48, 48 + n_offset),
            (48 + n_offset, 48 + n_offset + n_angle),
            (48 + n_offset + n_angle, 48 + n_offset + n_angle + n_combined),
            (48 + n_offset + n_angle + n_combined, len(self.configs)),
        ]
        actual_row = 0
        for sec_start, sec_end in sections:
            if actual_row == clicked_row:
                return
            actual_row += 1
            for i in range(sec_start, sec_end):
                if actual_row == clicked_row:
                    self._start_config(i)
                    self.show_list = False
                    return
                actual_row += 1

    def handle_events(self):
        """Process input events."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.MOUSEBUTTONDOWN and self.show_list:
                if event.button == 1:
                    self._handle_list_click(event.pos)
                elif event.button == 4:
                    self.list_scroll = max(0, self.list_scroll - 3)
                elif event.button == 5:
                    self.list_scroll = min(len(self.configs) - 1, self.list_scroll + 3)
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    self.running = False
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key == pygame.K_r:
                    self._start_config(self.config_idx)
                elif event.key in (pygame.K_PLUS, pygame.K_EQUALS):
                    self.speed_mult = min(16, self.speed_mult * 2)
                elif event.key == pygame.K_MINUS:
                    self.speed_mult = max(1, self.speed_mult // 2)
                elif event.key == pygame.K_RIGHT and not self.show_list:
                    self._start_config(self.config_idx + 1)
                elif event.key == pygame.K_LEFT and not self.show_list:
                    self._start_config(self.config_idx - 1)
                elif event.key == pygame.K_t:
                    self.show_sensors = not self.show_sensors
                elif event.key == pygame.K_p:
                    self.show_trail = not self.show_trail
                elif event.key == pygame.K_g:
                    self.show_grid = not self.show_grid
                    self.track_surface = self._render_track()
                elif event.key == pygame.K_a:
                    self.auto_play = not self.auto_play
                    if self.auto_play:
                        self.batch_results = {}
                        self.speed_mult = 8
                        self._start_config(0)
                elif event.key == pygame.K_s:
                    self._screenshot()
                elif event.key == pygame.K_l:
                    self.show_list = not self.show_list
                    self.list_scroll = max(0, self.config_idx - 10)
                elif event.key == pygame.K_UP and self.show_list:
                    self.list_scroll = max(0, self.list_scroll - 5)
                elif event.key == pygame.K_DOWN and self.show_list:
                    self.list_scroll = min(len(self.configs) - 1, self.list_scroll + 5)
                elif event.key == pygame.K_RETURN and self.show_list:
                    self.show_list = False

    def _screenshot(self):
        """Save a screenshot."""
        os.makedirs("screenshots", exist_ok=True)
        filename = f"screenshots/sim_{int(time.time())}.png"
        pygame.image.save(self.screen, filename)
        print(f"Screenshot saved: {filename}")

    def run(self):
        """Main loop."""
        while self.running:
            self.handle_events()

            # Advance simulation
            if not self.paused and not self.sim.finished:
                dt = 1.0 / FPS
                for _ in range(self.speed_mult):
                    self.sim.step(dt)

            # Auto-play: advance to next config when finished
            if self.auto_play and self.sim.finished:
                self.batch_results[self.config_idx] = self.sim.result
                if self.config_idx + 1 < len(self.configs):
                    self._start_config(self.config_idx + 1)
                else:
                    self.auto_play = False

            # Render
            self.screen.fill(BG_DARK)

            # Track (pre-rendered static layer)
            self.screen.blit(self.track_surface, (0, 0))

            # Dynamic elements on track area
            track_surf = pygame.Surface((TRACK_AREA_PX, TRACK_AREA_PX), pygame.SRCALPHA)
            self._draw_trail(track_surf)
            self._draw_sensors(track_surf)
            self._draw_robot(track_surf)
            self.screen.blit(track_surf, (0, 0))

            # Panel
            self._draw_panel()

            # Config list overlay
            if self.show_list:
                self._draw_config_list()

            pygame.display.flip()
            self.clock.tick(FPS)

        pygame.quit()


if __name__ == '__main__':
    viewer = Viewer()
    viewer.run()
