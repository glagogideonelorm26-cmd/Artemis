"""
WRO 2026 Future Engineers - Track Generation
Handles track geometry, pillar randomization, parking lot placement.
"""

import random
import math
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
from config import *


@dataclass
class Pillar:
    """A traffic sign (colored pillar) on the track."""
    x: float           # Center position in mm
    y: float
    color: str         # 'red' or 'green'
    section_idx: int   # Which straight section (0-3)
    seat_idx: int      # Which seat in the section
    moved: bool = False
    knocked: bool = False
    original_x: float = 0
    original_y: float = 0

    def __post_init__(self):
        self.original_x = self.x
        self.original_y = self.y

    def distance_from_seat(self) -> float:
        return math.sqrt((self.x - self.original_x)**2 + (self.y - self.original_y)**2)

    def is_outside_circle(self) -> bool:
        return self.distance_from_seat() > EVALUATION_CIRCLE_DIAMETER / 2


@dataclass
class ParkingLot:
    """The parking area in the obstacle challenge."""
    x: float           # Top-left corner in mm
    y: float
    width: float       # Always 200mm
    length: float      # 1.5 * robot length
    section_idx: int   # Which straight section
    marker_left: Tuple[float, float, float, float] = (0, 0, 0, 0)  # x, y, w, h
    marker_right: Tuple[float, float, float, float] = (0, 0, 0, 0)


@dataclass
class Section:
    """A track section (straight or corner)."""
    index: int
    section_type: str   # 'straight' or 'corner'
    # Bounding box in mm
    x1: float
    y1: float
    x2: float
    y2: float
    # Orange/blue line at entry/exit
    line_color: Optional[str] = None  # 'orange' or 'blue'
    line_pos: Optional[Tuple[float, float, float, float]] = None  # x1,y1,x2,y2


class Track:
    """
    Full WRO 2026 game field.
    
    Track layout (top-down view):
    
        Corner0   Straight0 (top)    Corner1
        
        Straight3                    Straight1
        (left)                       (right)
        
        Corner3   Straight2 (bottom) Corner2
    
    The track goes clockwise: S0 -> C1 -> S1 -> C2 -> S2 -> C3 -> S3 -> C0 -> S0
    """

    def __init__(self, challenge_type: str = 'obstacle', track_width: int = None,
                 section_widths: list = None, driving_direction: int = None,
                 starting_section_idx: int = None):
        self.challenge_type = challenge_type

        # Per-section track widths: [top, right, bottom, left]
        # WRO 2026: open challenge has independent widths per section (coin toss).
        # Obstacle challenge always uses 1000mm for all sections.
        if section_widths is not None:
            self.section_widths = list(section_widths)
        elif track_width is not None:
            self.section_widths = [track_width] * 4
        elif challenge_type == 'obstacle':
            self.section_widths = [TRACK_WIDTH_OBSTACLE] * 4
        else:
            self.section_widths = [
                random.choice([TRACK_WIDTH_OPEN_NARROW, TRACK_WIDTH_OPEN_WIDE])
                for _ in range(4)
            ]

        # Backward-compat: single track_width = max of all sections
        self.track_width = max(self.section_widths)

        self.pillars: List[Pillar] = []
        self.parking_lot: Optional[ParkingLot] = None
        self.sections: List[Section] = []
        self.walls_outer: List[Tuple[float, float, float, float]] = []
        self.walls_inner: List[Tuple[float, float, float, float]] = []
        self.orange_blue_lines: List[dict] = []

        # Driving direction: 1 = clockwise, -1 = counter-clockwise
        self.driving_direction = (driving_direction if driving_direction is not None
                                  else random.choice([1, -1]))

        # Starting section (0-3, index into straight sections)
        self.starting_section_idx = (starting_section_idx if starting_section_idx is not None
                                     else random.randint(0, 3))

        # Build geometry
        self._build_walls()
        self._build_sections()
        self._build_lines()

        if challenge_type == 'obstacle':
            self._randomize_pillars()
            self._place_parking_lot()

    def _build_walls(self):
        """Build outer and inner wall segments.

        The inner box is an asymmetric rectangle when section widths differ.
        Each inner wall is offset from the corresponding outer wall by that
        section's width: top by w0, right by w1, bottom by w2, left by w3.
        """
        ox = WALL_OFFSET  # 100mm
        oy = WALL_OFFSET
        ts = TRACK_INNER_SIZE  # 3000mm

        # Outer walls (fixed square)
        self.walls_outer = [
            (ox, oy, ox + ts, oy),                # Top
            (ox + ts, oy, ox + ts, oy + ts),      # Right
            (ox, oy + ts, ox + ts, oy + ts),      # Bottom
            (ox, oy, ox, oy + ts),                # Left
        ]
        self.outer_rect = (ox, oy, ox + ts, oy + ts)

        # Inner walls: each side inset by corresponding section width
        w0, w1, w2, w3 = self.section_widths  # top, right, bottom, left
        ix_left = ox + w3
        ix_right = ox + ts - w1
        iy_top = oy + w0
        iy_bottom = oy + ts - w2

        self.walls_inner = [
            (ix_left, iy_top, ix_right, iy_top),       # Top inner wall
            (ix_right, iy_top, ix_right, iy_bottom),   # Right inner wall
            (ix_left, iy_bottom, ix_right, iy_bottom), # Bottom inner wall
            (ix_left, iy_top, ix_left, iy_bottom),     # Left inner wall
        ]
        self.inner_rect = (ix_left, iy_top, ix_right, iy_bottom)

    def _build_sections(self):
        """Define the 8 sections (4 straights + 4 corners).

        With per-section widths, corners become rectangles (not squares) and
        straight sections have variable width/height.
        """
        ox, oy = WALL_OFFSET, WALL_OFFSET
        ts = TRACK_INNER_SIZE
        w0, w1, w2, w3 = self.section_widths

        # Inner rectangle corners (same as in _build_walls)
        ix_left = ox + w3
        ix_right = ox + ts - w1
        iy_top = oy + w0
        iy_bottom = oy + ts - w2

        # Corners: rectangular regions at each track corner
        corners = [
            Section(0, 'corner', ox, oy, ix_left, iy_top),              # Top-left (C0)
            Section(2, 'corner', ix_right, oy, ox + ts, iy_top),        # Top-right (C1)
            Section(4, 'corner', ix_right, iy_bottom, ox + ts, oy + ts),# Bottom-right (C2)
            Section(6, 'corner', ox, iy_bottom, ix_left, oy + ts),      # Bottom-left (C3)
        ]

        # Straights: strips between corners
        straights = [
            Section(1, 'straight', ix_left, oy, ix_right, iy_top),      # Top (S0)
            Section(3, 'straight', ix_right, iy_top, ox + ts, iy_bottom),# Right (S1)
            Section(5, 'straight', ix_left, iy_bottom, ix_right, oy + ts),# Bottom (S2)
            Section(7, 'straight', ox, iy_top, ix_left, iy_bottom),     # Left (S3)
        ]

        # Interleave: corner0, straight0, corner1, straight1, ...
        self.sections = []
        for i in range(4):
            self.sections.append(corners[i])
            self.sections.append(straights[i])

        self.straight_sections = straights
        self.corner_sections = corners

    def _build_lines(self):
        """Build the 8 colored lines per WRO 2026 mat (Figure 11).

        There is an invisible 1000×1000 mm square centered on the mat.
        Its vertices sit at (1100,1100), (2100,1100), (2100,2100), (1100,2100).

        From each vertex, two lines radiate outward to the outer walls.
        At each vertex the two sides of the square meet at 90°.  Each line
        is 30° from the nearest side, and 30° apart from each other:
            30° (side-to-line1) + 30° (line1-to-line2) + 30° (line2-to-side) = 90°

        Each line extends from the vertex until it hits an outer wall
        (at x=100, x=3100, y=100 or y=3100).

        These are FIXED PAINT on the mat — they never move regardless of
        where the inner walls are placed.

        Color assignment (physical paint, CW traversal order):
          Going CW around the track, at each corner you cross the orange
          line first (entering the corner) and the blue line second
          (exiting the corner).
        """
        # Invisible center square vertices
        # (same as inner wall corners on a standard 1000mm-wide track)
        v_tl = (1100, 1100)
        v_tr = (2100, 1100)
        v_br = (2100, 2100)
        v_bl = (1100, 2100)

        tan30 = math.tan(math.radians(30))

        def ray_to_wall(vx, vy, angle_deg):
            """Trace ray from vertex at angle until it hits an outer wall."""
            dx = math.cos(math.radians(angle_deg))
            dy = math.sin(math.radians(angle_deg))
            candidates = []
            for wall_val, axis, comp in [
                (100, 'x', dx), (3100, 'x', dx),
                (100, 'y', dy), (3100, 'y', dy),
            ]:
                if abs(comp) < 1e-9:
                    continue
                if axis == 'x':
                    t = (wall_val - vx) / dx
                    if t > 0:
                        y = vy + t * dy
                        if 100 <= y <= 3100:
                            candidates.append((t, wall_val, y))
                else:
                    t = (wall_val - vy) / dy
                    if t > 0:
                        x = vx + t * dx
                        if 100 <= x <= 3100:
                            candidates.append((t, x, wall_val))
            candidates.sort()
            return (candidates[0][1], candidates[0][2])

        # For each vertex, compute two outward ray angles.
        # The exterior angle at each corner spans 90° between the outward
        # normals of the two meeting sides.  Lines are at 30° from each side.
        #
        # Screen coords: x-right, y-down.
        # Outward normal of a side = direction perpendicular to the side,
        # pointing away from the square center.

        # TR vertex (2100,1100): top side normal = 270° (up), right side normal = 0°
        # Exterior spans 270° → 360°.  Line1 = 270°+30° = 300°, Line2 = 360°-30° = 330°
        tr_l1 = ray_to_wall(*v_tr, 300)
        tr_l2 = ray_to_wall(*v_tr, 330)

        # TL vertex (1100,1100): left side normal = 180°, top side normal = 270°
        # Exterior spans 180° → 270°.  Line1 = 180°+30° = 210°, Line2 = 270°-30° = 240°
        tl_l1 = ray_to_wall(*v_tl, 210)
        tl_l2 = ray_to_wall(*v_tl, 240)

        # BL vertex (1100,2100): bottom side normal = 90°, left side normal = 180°
        # Exterior spans 90° → 180°.  Line1 = 90°+30° = 120°, Line2 = 180°-30° = 150°
        bl_l1 = ray_to_wall(*v_bl, 120)
        bl_l2 = ray_to_wall(*v_bl, 150)

        # BR vertex (2100,2100): right side normal = 0°, bottom side normal = 90°
        # Exterior spans 0° → 90°.  Line1 = 0°+30° = 30°, Line2 = 90°-30° = 60°
        br_l1 = ray_to_wall(*v_br, 30)
        br_l2 = ray_to_wall(*v_br, 60)

        # Assign colors based on CW traversal order.
        # Going CW: S0(top,right) → C1(TR) → S1(right,down) → C2(BR) → S2(bottom,left) → C3(BL) → S3(left,up) → C0(TL) → S0
        # At each corner, the first line crossed (orange) is the one closer
        # to the straight you're coming FROM; the second (blue) is closer
        # to the straight you're going TO.
        #
        # C0 = TL corner: CW comes from S3 (left side, going up), exits to S0 (top, going right)
        #   tl_l1 hits left wall (coming from S3) → orange (entry)
        #   tl_l2 hits top wall (going to S0) → blue (exit)
        # C1 = TR corner: CW comes from S0 (top, going right), exits to S1 (right, going down)
        #   tr_l1 hits top wall (coming from S0) → orange (entry)
        #   tr_l2 hits right wall (going to S1) → blue (exit)
        # C2 = BR corner: CW comes from S1 (right, going down), exits to S2 (bottom, going left)
        #   br_l1 hits right wall (coming from S1) → orange (entry)
        #   br_l2 hits bottom wall (going to S2) → blue (exit)
        # C3 = BL corner: CW comes from S2 (bottom, going left), exits to S3 (left, going up)
        #   bl_l2 hits bottom wall (coming from S2) → orange (entry)
        #   bl_l1 hits left wall (going to S3) → blue (exit)

        line_positions = [
            # C0 (TL): orange→left wall (tl_l1), blue→top wall (tl_l2)
            {'pos': (*v_tl, *tl_l1), 'corner': 0, 'role': 'entry', 'color': 'orange'},
            {'pos': (*v_tl, *tl_l2), 'corner': 0, 'role': 'exit',  'color': 'blue'},
            # C1 (TR): orange→top wall (tr_l1), blue→right wall (tr_l2)
            {'pos': (*v_tr, *tr_l1), 'corner': 1, 'role': 'entry', 'color': 'orange'},
            {'pos': (*v_tr, *tr_l2), 'corner': 1, 'role': 'exit',  'color': 'blue'},
            # C2 (BR): orange→right wall (br_l1), blue→bottom wall (br_l2)
            {'pos': (*v_br, *br_l1), 'corner': 2, 'role': 'entry', 'color': 'orange'},
            {'pos': (*v_br, *br_l2), 'corner': 2, 'role': 'exit',  'color': 'blue'},
            # C3 (BL): orange→bottom wall (bl_l1), blue→left wall (bl_l2)
            {'pos': (*v_bl, *bl_l1), 'corner': 3, 'role': 'entry', 'color': 'orange'},
            {'pos': (*v_bl, *bl_l2), 'corner': 3, 'role': 'exit',  'color': 'blue'},
        ]

        self.orange_blue_lines = line_positions

    def _get_seat_positions(self, section_idx: int) -> List[Tuple[float, float, str]]:
        """
        Get traffic sign seat positions for a straight section.
        Returns list of (x, y, side) where side is 'inner' or 'outer'.

        Each straight section has 6 seats:
        - 2 X-intersections (can be on either side)
        - 4 T-intersections (2 on inner side, 2 on outer side)

        Seats are placed within the section's bounding box, spaced evenly
        along the section length, alternating between inner and outer sides.
        Uses per-section width (not the global max) for correct placement
        in asymmetric track configurations.
        """
        section = self.straight_sections[section_idx]
        seats = []

        tw = self.section_widths[section_idx]

        # Use the section bounding box directly — it already accounts
        # for asymmetric inner wall positions from _build_sections.
        if section_idx == 0:  # Top straight (horizontal)
            section_length = section.x2 - section.x1
            start_x = section.x1
            for i in range(6):
                sx = start_x + (i + 0.5) * section_length / 6
                if i % 2 == 0:  # Outer side (top wall)
                    sy = section.y1 + tw * 0.25
                    seats.append((sx, sy, 'outer'))
                else:  # Inner side (inner wall)
                    sy = section.y2 - tw * 0.25
                    seats.append((sx, sy, 'inner'))

        elif section_idx == 1:  # Right straight (vertical)
            section_length = section.y2 - section.y1
            start_y = section.y1
            for i in range(6):
                sy = start_y + (i + 0.5) * section_length / 6
                if i % 2 == 0:  # Outer side (right wall)
                    sx = section.x2 - tw * 0.25
                    seats.append((sx, sy, 'outer'))
                else:  # Inner side (inner wall)
                    sx = section.x1 + tw * 0.25
                    seats.append((sx, sy, 'inner'))

        elif section_idx == 2:  # Bottom straight (horizontal)
            section_length = section.x2 - section.x1
            start_x = section.x1
            for i in range(6):
                sx = start_x + (i + 0.5) * section_length / 6
                if i % 2 == 0:  # Outer side (bottom wall)
                    sy = section.y2 - tw * 0.25
                    seats.append((sx, sy, 'outer'))
                else:  # Inner side (inner wall)
                    sy = section.y1 + tw * 0.25
                    seats.append((sx, sy, 'inner'))

        elif section_idx == 3:  # Left straight (vertical)
            section_length = section.y2 - section.y1
            start_y = section.y1
            for i in range(6):
                sy = start_y + (i + 0.5) * section_length / 6
                if i % 2 == 0:  # Outer side (left wall)
                    sx = section.x1 + tw * 0.25
                    seats.append((sx, sy, 'outer'))
                else:  # Inner side (inner wall)
                    sx = section.x2 - tw * 0.25
                    seats.append((sx, sy, 'inner'))

        return seats

    def _randomize_pillars(self):
        """
        Randomize pillar placement following WRO rules:
        1. One section gets a single sign (color by coin toss)
        2. Other 3 sections get pillars from random card draws
        3. Each card defines 0-2 pillars with positions and colors
        """
        self.pillars = []

        # Step 1: Choose which section gets the single sign
        single_section = random.randint(0, 3)

        # Step 2: Color of single sign
        single_color = random.choice(['red', 'green'])

        # Step 3: Place single sign in that section
        seats = self._get_seat_positions(single_section)
        seat_idx = random.randint(0, len(seats) - 1)
        sx, sy, side = seats[seat_idx]
        self.pillars.append(Pillar(
            x=sx, y=sy, color=single_color,
            section_idx=single_section, seat_idx=seat_idx
        ))

        # Step 4: For remaining 3 sections, draw random card configurations
        # Simplified card system: each card places 1-2 pillars at random seats
        other_sections = [i for i in range(4) if i != single_section]

        for sec_idx in other_sections:
            seats = self._get_seat_positions(sec_idx)
            # Random number of pillars (1 or 2, matching card distribution)
            num_pillars = random.choice([1, 1, 2, 2, 2])  # Most cards have 2

            # Choose random seats (no duplicates)
            chosen_seats = random.sample(range(len(seats)), min(num_pillars, len(seats)))

            for seat_idx in chosen_seats:
                sx, sy, side = seats[seat_idx]
                # Color: roughly equal probability, but ensure
                # we don't have all same color
                color = random.choice(['red', 'green'])
                self.pillars.append(Pillar(
                    x=sx, y=sy, color=color,
                    section_idx=sec_idx, seat_idx=seat_idx
                ))

    def _place_parking_lot(self):
        """
        Place parking lot in the starting section.
        Rules: parking lot is always in starting section, 20cm wide,
        1.5× robot length long, against the outer wall.
        """
        sec_idx = self.starting_section_idx
        section = self.straight_sections[sec_idx]

        parking_length = PARKING_LENGTH_FACTOR * ROBOT_LENGTH  # 285mm
        parking_width = PARKING_WIDTH  # 200mm

        ox, oy = WALL_OFFSET, WALL_OFFSET
        tw = self.track_width
        ts = TRACK_INNER_SIZE

        # Place parking lot against the outer wall, centered in the section
        if sec_idx == 0:  # Top straight
            mid_x = (section.x1 + section.x2) / 2
            px = mid_x - parking_length / 2
            py = oy  # Against outer (top) wall
            pw = parking_length
            ph = parking_width
        elif sec_idx == 1:  # Right straight
            mid_y = (section.y1 + section.y2) / 2
            px = ox + ts - parking_width  # Against outer (right) wall
            py = mid_y - parking_length / 2
            pw = parking_width
            ph = parking_length
        elif sec_idx == 2:  # Bottom straight
            mid_x = (section.x1 + section.x2) / 2
            px = mid_x - parking_length / 2
            py = oy + ts - parking_width  # Against outer (bottom) wall
            pw = parking_length
            ph = parking_width
        elif sec_idx == 3:  # Left straight
            mid_y = (section.y1 + section.y2) / 2
            px = ox  # Against outer (left) wall
            py = mid_y - parking_length / 2
            pw = parking_width
            ph = parking_length

        self.parking_lot = ParkingLot(
            x=px, y=py, width=pw, length=ph,
            section_idx=sec_idx
        )

        # Move any pillars in the starting section closer to inner wall
        for pillar in self.pillars:
            if pillar.section_idx == sec_idx:
                if sec_idx == 0:  # Top: move pillars toward bottom (inner)
                    pillar.y = oy + tw * 0.75
                elif sec_idx == 1:  # Right: move toward left (inner)
                    pillar.x = ox + ts - tw * 0.75
                elif sec_idx == 2:  # Bottom: move toward top (inner)
                    pillar.y = oy + ts - tw * 0.75
                elif sec_idx == 3:  # Left: move toward right (inner)
                    pillar.x = ox + tw * 0.75
                pillar.original_x = pillar.x
                pillar.original_y = pillar.y

    def get_wall_segments(self) -> List[Tuple[float, float, float, float]]:
        """Get all wall segments for collision/sensor checking."""
        return self.walls_outer + self.walls_inner

    def get_section_at_position(self, x: float, y: float) -> Optional[int]:
        """Determine which section a position is in."""
        for i, section in enumerate(self.sections):
            if (section.x1 <= x <= section.x2 and
                section.y1 <= y <= section.y2):
                return i
        return None

    def get_local_track_width(self, x: float, y: float) -> float:
        """Get the track width relevant to a position.

        For straight sections returns that section's width.
        For corners returns the minimum of the two adjacent section widths
        (conservative — the tightest dimension constrains the turn).
        """
        section_idx = self.get_section_at_position(x, y)
        if section_idx is None:
            return max(self.section_widths)

        section = self.sections[section_idx]
        if section.section_type == 'straight':
            # Sections list: C0(0), S0(1), C1(2), S1(3), C2(4), S2(5), C3(6), S3(7)
            straight_to_width = {1: 0, 3: 1, 5: 2, 7: 3}
            return self.section_widths[straight_to_width[section_idx]]
        else:
            # Corner: min of two adjacent straight section widths
            corner_to_widths = {
                0: (3, 0),  # C0 between S3(left) and S0(top)
                2: (0, 1),  # C1 between S0(top) and S1(right)
                4: (1, 2),  # C2 between S1(right) and S2(bottom)
                6: (2, 3),  # C3 between S2(bottom) and S3(left)
            }
            a, b = corner_to_widths[section_idx]
            return min(self.section_widths[a], self.section_widths[b])

    def is_on_track(self, x: float, y: float) -> bool:
        """Check if a position is on the track (between inner and outer walls)."""
        orx1, ory1, orx2, ory2 = self.outer_rect
        irx1, iry1, irx2, iry2 = self.inner_rect
        
        in_outer = orx1 <= x <= orx2 and ory1 <= y <= ory2
        in_inner = irx1 <= x <= irx2 and iry1 <= y <= iry2
        
        return in_outer and not in_inner

    def get_line_at_position(self, x: float, y: float, threshold: float = 30) -> Optional[str]:
        """Check if position is on an orange or blue line (diagonal-aware)."""
        for line in self.orange_blue_lines:
            lx1, ly1, lx2, ly2 = line['pos']
            # Point-to-line-segment distance
            dx = lx2 - lx1
            dy = ly2 - ly1
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq < 1e-6:
                continue
            t = ((x - lx1) * dx + (y - ly1) * dy) / seg_len_sq
            t = max(0.0, min(1.0, t))
            closest_x = lx1 + t * dx
            closest_y = ly1 + t * dy
            d = math.sqrt((x - closest_x) ** 2 + (y - closest_y) ** 2)
            if d < threshold:
                return line['color']
        return None

    def check_parking(self, robot_x: float, robot_y: float,
                      robot_angle: float, robot_length: float,
                      robot_width: float) -> str:
        """
        Check parking status.
        Returns: 'fully_parked', 'partly_parked', 'not_parked', or 'touching_marker'
        """
        if self.parking_lot is None:
            return 'not_parked'

        pl = self.parking_lot
        # Get robot corners
        corners = self._get_robot_corners(robot_x, robot_y, robot_angle,
                                          robot_length, robot_width)

        # Check if all corners are inside parking area
        all_inside = all(
            pl.x <= cx <= pl.x + pl.width and
            pl.y <= cy <= pl.y + pl.length
            for cx, cy in corners
        )

        any_inside = any(
            pl.x <= cx <= pl.x + pl.width and
            pl.y <= cy <= pl.y + pl.length
            for cx, cy in corners
        )

        if all_inside:
            # Check parallel: robot must be aligned with the wall in either direction
            sec_idx = pl.section_idx
            norm_angle = robot_angle % 360
            if sec_idx in [0, 2]:  # Horizontal section — parallel at 0° or 180°
                diff_0 = abs(norm_angle)
                diff_0 = min(diff_0, 360 - diff_0)
                diff_180 = abs(norm_angle - 180)
                diff_180 = min(diff_180, 360 - diff_180)
                angle_diff = min(diff_0, diff_180)
            else:  # Vertical section — parallel at 90° or 270°
                diff_90 = abs(norm_angle - 90)
                diff_90 = min(diff_90, 360 - diff_90)
                diff_270 = abs(norm_angle - 270)
                diff_270 = min(diff_270, 360 - diff_270)
                angle_diff = min(diff_90, diff_270)

            max_angle = math.degrees(math.atan2(PARKING_PARALLEL_TOLERANCE, WHEELBASE))
            if angle_diff <= max_angle:
                return 'fully_parked'
            else:
                return 'partly_parked'
        elif any_inside:
            return 'partly_parked'
        else:
            return 'not_parked'

    def _get_robot_corners(self, x, y, angle, length, width):
        """Get the 4 corners of the robot given center position and angle."""
        rad = math.radians(angle)
        cos_a = math.cos(rad)
        sin_a = math.sin(rad)
        hl = length / 2
        hw = width / 2

        corners = [
            (x + hl * cos_a - hw * sin_a, y + hl * sin_a + hw * cos_a),
            (x + hl * cos_a + hw * sin_a, y + hl * sin_a - hw * cos_a),
            (x - hl * cos_a + hw * sin_a, y - hl * sin_a - hw * cos_a),
            (x - hl * cos_a - hw * sin_a, y - hl * sin_a + hw * cos_a),
        ]
        return corners
