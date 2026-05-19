"""
Isolated PD tuning test for wall-following stability.
Places robot mid-corridor with heading error, runs physics+controller,
checks whether heading converges to cardinal without wall collision.

Phase 1: Pure straight-line PD stability
Phase 2: Corner exit → stabilization
Phase 3: Full 48-case verification (via test_headless.py --open)
"""

import math
import sys
from config import *
from track import Track
from robot import Robot
from controller import Controller, State, _angle_diff


# Scoring: each test case awards points based on pass/fail
POINTS_CONVERGENCE = 3      # Heading converges within 3° and no wall collision
POINTS_NO_COLLISION = 2     # Test ran but didn't fully converge or had collision risk
POINTS_FAILURE = 0          # Complete failure


def calc_test_points(result: dict) -> int:
    """Calculate points for a test based on its result dict."""
    if result.get('converged') or result.get('completed'):
        return POINTS_CONVERGENCE
    elif result.get('wall_collision'):
        return 0
    else:
        return POINTS_NO_COLLISION


def run_pd_stability_test(track_width=1000, heading_error=5, duration=3.0,
                          speed_fraction=SPEED_OPEN_CRUISE, verbose=False):
    """Place robot mid-corridor with heading error, run controller, check convergence.

    Returns dict with:
      converged: bool (heading within 3 degrees of cardinal at end)
      max_heading_error: float (peak heading deviation during run)
      wall_collision: bool
      final_heading_error: float
      oscillations: int (number of zero-crossings of heading error)
    """
    section_widths = [track_width] * 4
    track = Track(
        challenge_type='open',
        section_widths=section_widths,
        driving_direction=1,
        starting_section_idx=0,
    )

    # Place robot near the start of top straight section so it has max room
    # before hitting the corner entry line (which would trigger state change)
    section = track.straight_sections[0]
    cx = section.x1 + 100  # Near left edge — gives ~900mm before corner
    cy = (section.y1 + section.y2) / 2
    start_heading = 0 + heading_error

    robot = Robot(cx, cy, start_heading)
    robot.imu_heading = start_heading

    controller = Controller(driving_direction=1, challenge_type='open')
    controller.state = State.WALL_FOLLOWING

    physics_dt = 1.0 / SIM_FPS
    control_dt = 1.0 / CONTROL_HZ
    control_accumulator = 0.0

    max_heading_err = 0.0
    wall_collision = False
    heading_errors = []

    steps = int(duration * SIM_FPS)
    for step in range(steps):
        control_accumulator += physics_dt
        while control_accumulator >= control_dt:
            control_accumulator -= control_dt
            sensors = robot.get_sensors(track)
            controller.update(sensors, robot, track, control_dt)

        robot.update(physics_dt)

        if robot.check_wall_collision(track):
            wall_collision = True
            break

        # Stop measuring if controller left WALL_FOLLOWING or entered corner approach
        if controller.state != State.WALL_FOLLOWING:
            break
        if controller._corner_approach_ticks > 0:
            break

        h_err = _angle_diff(0, robot.imu_heading)
        heading_errors.append(h_err)
        max_heading_err = max(max_heading_err, abs(h_err))

    final_err = heading_errors[-1] if heading_errors else heading_error

    # Count oscillations (zero-crossings)
    oscillations = 0
    for i in range(1, len(heading_errors)):
        if heading_errors[i] * heading_errors[i-1] < 0:
            oscillations += 1

    converged = abs(final_err) < 3.0 and not wall_collision

    if verbose:
        print(f"  width={track_width}mm err={heading_error:+.0f}° → "
              f"final={final_err:+.1f}° max={max_heading_err:.1f}° "
              f"osc={oscillations} wall={'HIT' if wall_collision else 'ok'} "
              f"{'PASS' if converged else 'FAIL'}")

    return {
        'converged': converged,
        'max_heading_error': max_heading_err,
        'wall_collision': wall_collision,
        'final_heading_error': final_err,
        'oscillations': oscillations,
    }


def run_corner_exit_test(track_width=1000, exit_heading_error=5, duration=3.0,
                         verbose=False):
    """Simulate robot just exited a corner — in cooldown state with heading offset.

    Tests the cooldown → wall-following transition specifically.
    """
    section_widths = [track_width] * 4
    track = Track(
        challenge_type='open',
        section_widths=section_widths,
        driving_direction=1,
        starting_section_idx=0,
    )

    # Place robot at start of top straight (just after corner exit)
    section = track.straight_sections[0]
    # Near the left edge of the straight (just exited corner C0)
    cx = section.x1 + 150
    cy = (section.y1 + section.y2) / 2
    start_heading = 0 + exit_heading_error

    robot = Robot(cx, cy, start_heading)
    robot.imu_heading = start_heading
    robot.speed = SPEED_OPEN_CRUISE * MAX_SPEED

    controller = Controller(driving_direction=1, challenge_type='open')
    controller.state = State.WALL_FOLLOWING
    controller._corner_cooldown = 30  # Simulate just exited corner

    physics_dt = 1.0 / SIM_FPS
    control_dt = 1.0 / CONTROL_HZ
    control_accumulator = 0.0

    wall_collision = False
    heading_errors = []

    steps = int(duration * SIM_FPS)
    for step in range(steps):
        control_accumulator += physics_dt
        while control_accumulator >= control_dt:
            control_accumulator -= control_dt
            sensors = robot.get_sensors(track)
            controller.update(sensors, robot, track, control_dt)

        robot.update(physics_dt)

        if robot.check_wall_collision(track):
            wall_collision = True
            break

        h_err = _angle_diff(0, robot.imu_heading)
        heading_errors.append(h_err)

    final_err = heading_errors[-1] if heading_errors else exit_heading_error
    converged = abs(final_err) < 3.0 and not wall_collision

    if verbose:
        max_err = max(abs(e) for e in heading_errors) if heading_errors else 0
        print(f"  corner_exit width={track_width}mm err={exit_heading_error:+.0f}° → "
              f"final={final_err:+.1f}° max={max_err:.1f}° "
              f"wall={'HIT' if wall_collision else 'ok'} "
              f"{'PASS' if converged else 'FAIL'}")

    return {
        'converged': converged,
        'wall_collision': wall_collision,
        'final_heading_error': final_err,
    }


def run_offset_stability_test(track_width=1000, heading_error=5, lateral_offset=0,
                              duration=3.0, verbose=False):
    """Place robot off-center with heading error, run controller, check convergence.

    lateral_offset: mm from center (positive = toward right wall, negative = toward left wall)
    """
    section_widths = [track_width] * 4
    track = Track(
        challenge_type='open',
        section_widths=section_widths,
        driving_direction=1,
        starting_section_idx=0,
    )

    section = track.straight_sections[0]
    cx = section.x1 + 100
    cy = (section.y1 + section.y2) / 2 + lateral_offset
    start_heading = 0 + heading_error

    robot = Robot(cx, cy, start_heading)
    robot.imu_heading = start_heading

    controller = Controller(driving_direction=1, challenge_type='open')
    controller.state = State.WALL_FOLLOWING

    physics_dt = 1.0 / SIM_FPS
    control_dt = 1.0 / CONTROL_HZ
    control_accumulator = 0.0

    max_heading_err = 0.0
    wall_collision = False
    heading_errors = []

    steps = int(duration * SIM_FPS)
    for step in range(steps):
        control_accumulator += physics_dt
        while control_accumulator >= control_dt:
            control_accumulator -= control_dt
            sensors = robot.get_sensors(track)
            controller.update(sensors, robot, track, control_dt)

        robot.update(physics_dt)

        if robot.check_wall_collision(track):
            wall_collision = True
            break

        if controller.state != State.WALL_FOLLOWING:
            break

        if controller._corner_approach_ticks > 0:
            break

        h_err = _angle_diff(0, robot.imu_heading)
        heading_errors.append(h_err)
        max_heading_err = max(max_heading_err, abs(h_err))

    final_err = heading_errors[-1] if heading_errors else heading_error

    oscillations = 0
    for i in range(1, len(heading_errors)):
        if heading_errors[i] * heading_errors[i-1] < 0:
            oscillations += 1

    converged = abs(final_err) < 3.0 and not wall_collision

    if verbose:
        print(f"  width={track_width}mm offset={lateral_offset:+.0f}mm "
              f"err={heading_error:+.0f}° → "
              f"final={final_err:+.1f}° max={max_heading_err:.1f}° "
              f"osc={oscillations} wall={'HIT' if wall_collision else 'ok'} "
              f"{'PASS' if converged else 'FAIL'}")

    return {
        'converged': converged,
        'max_heading_error': max_heading_err,
        'wall_collision': wall_collision,
        'final_heading_error': final_err,
        'oscillations': oscillations,
    }


def phase1_sweep(verbose=True):
    """Phase 1: Pure straight-line PD stability across errors and widths."""
    print("\n" + "=" * 60)
    print("PHASE 1: Straight-line PD stability")
    print("=" * 60)

    errors = [3, 5, 8, 12, 15]
    widths = [600, 1000]
    passed = 0
    total = 0
    total_points = 0

    for w in widths:
        for err in errors:
            for sign in [1, -1]:
                total += 1
                r = run_pd_stability_test(
                    track_width=w, heading_error=err * sign,
                    verbose=verbose)
                if r['converged']:
                    passed += 1
                points = calc_test_points(r)
                total_points += points

    print(f"\n  Phase 1 results: {passed}/{total} passed")
    print(f"  Points: {total_points}/{total * POINTS_CONVERGENCE}")
    return passed, total, total_points


def phase2_sweep(verbose=True):
    """Phase 2: Corner exit → stabilization."""
    print("\n" + "=" * 60)
    print("PHASE 2: Corner exit → stabilization")
    print("=" * 60)

    errors = [5, 8, 10, 15]
    widths = [600, 1000]
    passed = 0
    total = 0
    total_points = 0

    for w in widths:
        for err in errors:
            for sign in [1, -1]:
                total += 1
                r = run_corner_exit_test(
                    track_width=w, exit_heading_error=err * sign,
                    verbose=verbose)
                if r['converged']:
                    passed += 1
                points = calc_test_points(r)
                total_points += points

    print(f"\n  Phase 2 results: {passed}/{total} passed")
    print(f"  Points: {total_points}/{total * POINTS_CONVERGENCE}")
    return passed, total, total_points


def phase3_sweep(verbose=True):
    """Phase 3: Off-center start — robot placed left/right of midline.

    Uses 5s duration (realistic straight traversal time) and offsets
    matching real placement tolerance (~50-75mm).
    """
    print("\n" + "=" * 60)
    print("PHASE 3: Off-center lateral offset + heading error")
    print("=" * 60)

    heading_errors = [0, 5, 10]
    widths = [600, 1000]
    offsets_by_width = {600: [30, 50, 75], 1000: [50, 75, 100]}
    passed = 0
    total = 0
    total_points = 0

    for w in widths:
        for offset in offsets_by_width[w]:
            for offset_sign in [1, -1]:
                for err in heading_errors:
                    for err_sign in [1, -1] if err != 0 else [1]:
                        total += 1
                        r = run_offset_stability_test(
                            track_width=w,
                            heading_error=err * err_sign,
                            lateral_offset=offset * offset_sign,
                            duration=5.0,
                            verbose=verbose)
                        if r['converged']:
                            passed += 1
                        points = calc_test_points(r)
                        total_points += points

    print(f"\n  Phase 3 results: {passed}/{total} passed")
    print(f"  Points: {total_points}/{total * POINTS_CONVERGENCE}")
    return passed, total, total_points


def phase4_sweep(verbose=True):
    """Phase 4: Combined lateral offset + starting angle error (worst-case placement)."""
    print("\n" + "=" * 60)
    print("PHASE 4: Combined offset + angle (real-world placement)")
    print("=" * 60)

    widths = [600, 1000]
    offsets_by_width = {600: [50, 75], 1000: [75, 100]}
    angles = [5, -5]
    passed = 0
    total = 0
    total_points = 0

    for w in widths:
        for offset in offsets_by_width[w]:
            for offset_sign in [1, -1]:
                for angle in angles:
                    total += 1
                    r = run_offset_stability_test(
                        track_width=w,
                        heading_error=angle,
                        lateral_offset=offset * offset_sign,
                        duration=5.0,
                        verbose=verbose)
                    if r['converged']:
                        passed += 1
                    points = calc_test_points(r)
                    total_points += points

    print(f"\n  Phase 4 results: {passed}/{total} passed")
    print(f"  Points: {total_points}/{total * POINTS_CONVERGENCE}")
    return passed, total, total_points


def run_corner_entry_offset_test(track_width=1000, entry_heading_error=5,
                                 lateral_offset=0, direction=1, duration=8.0,
                                 verbose=False):
    """Place robot right at corner entry with heading offset, run through the turn.

    Tests whether the corner turn logic handles non-cardinal entry heading.
    Robot is placed just before the corner detection line with a heading error
    so it enters the turn skewed.

    Returns dict with: completed (bool), wall_collision (bool), exit_heading_error (float)
    """
    section_widths = [track_width] * 4
    track = Track(
        challenge_type='open',
        section_widths=section_widths,
        driving_direction=direction,
        starting_section_idx=0,
    )

    # Place robot near the end of section 0, just before corner entry line
    section = track.straight_sections[0]
    if direction == 1:
        # CW: traveling East (0°), corner is at right end
        cx = section.x2 - 50
        cardinal = 0
    else:
        # CCW: traveling West (180°), corner is at left end
        cx = section.x1 + 50
        cardinal = 180

    cy = (section.y1 + section.y2) / 2 + lateral_offset
    start_heading = cardinal + entry_heading_error

    robot = Robot(cx, cy, start_heading)
    robot.imu_heading = start_heading
    robot.speed = SPEED_OPEN_CRUISE * MAX_SPEED

    controller = Controller(driving_direction=direction, challenge_type='open')
    controller.state = State.WALL_FOLLOWING

    physics_dt = 1.0 / SIM_FPS
    control_dt = 1.0 / CONTROL_HZ
    control_accumulator = 0.0

    wall_collision = False
    completed_turn = False
    exit_heading = None

    # Expected exit cardinal after the corner
    # CW: entry 0° (East) + 90° turn right = 90° (South)
    # CCW: entry 180° (West) - 90° turn left = 90° (South)
    exit_cardinal = 90

    steps = int(duration * SIM_FPS)
    entered_corner = False
    exited_corner = False

    for step in range(steps):
        control_accumulator += physics_dt
        while control_accumulator >= control_dt:
            control_accumulator -= control_dt
            sensors = robot.get_sensors(track)
            controller.update(sensors, robot, track, control_dt)

        robot.update(physics_dt)

        if robot.check_wall_collision(track):
            wall_collision = True
            # Push out so we can see if it recovers
            orx1, ory1, orx2, ory2 = track.outer_rect
            irx1, iry1, irx2, iry2 = track.inner_rect
            in_inner = (irx1 <= robot.x <= irx2 and iry1 <= robot.y <= iry2)
            if in_inner:
                ocx, ocy = (irx1 + irx2) / 2, (iry1 + iry2) / 2
            else:
                ocx, ocy = (orx1 + orx2) / 2, (ory1 + ory2) / 2
            dx = ocx - robot.x
            dy = ocy - robot.y
            dist = max(1, (dx * dx + dy * dy) ** 0.5)
            robot.x += (dx / dist) * 20
            robot.y += (dy / dist) * 20

        if controller.state == State.CORNER_TURN:
            entered_corner = True
        elif entered_corner and controller.state == State.WALL_FOLLOWING:
            exited_corner = True
            exit_heading = robot.imu_heading
            # Give it 2 more seconds to stabilize after corner
            remaining = int(2.0 * SIM_FPS)
            for _ in range(remaining):
                control_accumulator += physics_dt
                while control_accumulator >= control_dt:
                    control_accumulator -= control_dt
                    sensors = robot.get_sensors(track)
                    controller.update(sensors, robot, track, control_dt)
                robot.update(physics_dt)
                if robot.check_wall_collision(track):
                    wall_collision = True
                    break
            break

    if exited_corner:
        final_heading = robot.imu_heading
        # Check how close to exit cardinal
        exit_err = _angle_diff(exit_cardinal, final_heading)
        completed_turn = abs(exit_err) < 15.0 and not wall_collision
    else:
        exit_err = 999.0
        completed_turn = False

    if verbose:
        d = "CW" if direction == 1 else "CCW"
        print(f"  {d} width={track_width}mm entry_err={entry_heading_error:+.0f}° "
              f"offset={lateral_offset:+.0f}mm → "
              f"exit_err={exit_err:+.1f}° "
              f"wall={'HIT' if wall_collision else 'ok'} "
              f"turn={'done' if exited_corner else 'STUCK'} "
              f"{'PASS' if completed_turn else 'FAIL'}")

    return {
        'completed': completed_turn,
        'wall_collision': wall_collision,
        'exit_heading_error': exit_err,
        'exited_corner': exited_corner,
    }


def phase5_sweep(verbose=True):
    """Phase 5: Corner entry with heading offset — does the turn complete cleanly?"""
    print("\n" + "=" * 60)
    print("PHASE 5: Corner entry with non-cardinal heading")
    print("=" * 60)

    widths = [600, 1000]
    entry_errors = [5, 8, 10, 12]
    offsets_by_width = {600: [0, 30], 1000: [0, 50]}
    passed = 0
    total = 0
    total_points = 0

    for w in widths:
        for err in entry_errors:
            for sign in [1, -1]:
                for offset in offsets_by_width[w]:
                    for offset_sign in [1, -1] if offset != 0 else [1]:
                        for direction in [1, -1]:
                            total += 1
                            r = run_corner_entry_offset_test(
                                track_width=w,
                                entry_heading_error=err * sign,
                                lateral_offset=offset * offset_sign,
                                direction=direction,
                                verbose=verbose)
                            if r['completed']:
                                passed += 1
                            points = calc_test_points(r)
                            total_points += points

    print(f"\n  Phase 5 results: {passed}/{total} passed")
    print(f"  Points: {total_points}/{total * POINTS_CONVERGENCE}")
    return passed, total, total_points


def trace_single(track_width=1000, heading_error=12, duration=3.0):
    """Detailed per-frame trace for debugging a single case."""
    print(f"\n{'='*60}")
    print(f"TRACE: width={track_width} heading_error={heading_error}")
    print(f"{'='*60}")

    section_widths = [track_width] * 4
    track = Track(
        challenge_type='open',
        section_widths=section_widths,
        driving_direction=1,
        starting_section_idx=0,
    )

    section = track.straight_sections[0]
    cx = section.x1 + 100
    cy = (section.y1 + section.y2) / 2
    start_heading = 0 + heading_error

    robot = Robot(cx, cy, start_heading)
    robot.imu_heading = start_heading

    controller = Controller(driving_direction=1, challenge_type='open')
    controller.state = State.WALL_FOLLOWING

    physics_dt = 1.0 / SIM_FPS
    control_dt = 1.0 / CONTROL_HZ
    control_accumulator = 0.0

    steps = int(duration * SIM_FPS)
    frame = 0
    for step in range(steps):
        control_accumulator += physics_dt
        while control_accumulator >= control_dt:
            control_accumulator -= control_dt
            sensors = robot.get_sensors(track)

            # Trace every 10th control step
            if frame % 10 == 0:
                error = sensors.tof_right - sensors.tof_left
                heading = sensors.imu_heading
                cardinal = (round(heading / 90) * 90) % 360
                h_err = _angle_diff(cardinal, heading)
                print(f"  t={frame/CONTROL_HZ:.2f}s pos=({robot.x:.0f},{robot.y:.0f}) "
                      f"hdg={heading:.1f} h_err={h_err:.1f} "
                      f"wall_err={error:.0f} "
                      f"tof_L={sensors.tof_left:.0f} tof_R={sensors.tof_right:.0f} "
                      f"state={controller.state.name} "
                      f"approach={controller._corner_approach_ticks}")

            controller.update(sensors, robot, track, control_dt)
            frame += 1

        robot.update(physics_dt)

        if robot.check_wall_collision(track):
            print(f"  WALL COLLISION at step {step}")
            break
        if controller.state != State.WALL_FOLLOWING:
            print(f"  State changed to {controller.state.name} at step {step}")
            break


if __name__ == '__main__':
    print("PD Tuning Test Suite")
    print("=" * 50)

    v = '--verbose' in sys.argv or '-v' in sys.argv

    if '--trace' in sys.argv:
        trace_single(track_width=1000, heading_error=12)
    else:
        p1_pass, p1_total, p1_pts = phase1_sweep(verbose=v)
        p2_pass, p2_total, p2_pts = phase2_sweep(verbose=v)
        p3_pass, p3_total, p3_pts = phase3_sweep(verbose=v)
        p4_pass, p4_total, p4_pts = phase4_sweep(verbose=v)
        p5_pass, p5_total, p5_pts = phase5_sweep(verbose=v)

        print("\n" + "=" * 60)
        print("SUMMARY")
        print(f"  Phase 1 (straight-line):  {p1_pass}/{p1_total} tests, {p1_pts} pts")
        print(f"  Phase 2 (corner exit):    {p2_pass}/{p2_total} tests, {p2_pts} pts")
        print(f"  Phase 3 (off-center):     {p3_pass}/{p3_total} tests, {p3_pts} pts")
        print(f"  Phase 4 (offset+angle):   {p4_pass}/{p4_total} tests, {p4_pts} pts")
        print(f"  Phase 5 (corner entry):   {p5_pass}/{p5_total} tests, {p5_pts} pts")
        all_pass = p1_pass + p2_pass + p3_pass + p4_pass + p5_pass
        all_total = p1_total + p2_total + p3_total + p4_total + p5_total
        all_pts = p1_pts + p2_pts + p3_pts + p4_pts + p5_pts
        max_pts = all_total * POINTS_CONVERGENCE
        print(f"  Total:                    {all_pass}/{all_total} tests, {all_pts}/{max_pts} pts")
        print("=" * 60)

        if all_pass == all_total:
            print("\nAll PD stability tests PASSED")
            print(f"Perfect score: {all_pts}/{max_pts}")
        else:
            print(f"\nSome tests FAILED — {all_pts}/{max_pts} points achieved")
            print(f"Pass rate: {100*all_pass/all_total:.1f}%")
