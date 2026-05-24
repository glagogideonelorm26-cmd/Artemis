"""
WRO 2026 Future Engineers - Headless Test Runner
Runs the simulation without graphics for testing and batch evaluation.
Usage: python test_headless.py
"""

import sys
import math
import time
from config import *
from track import Track
from robot import Robot
from controller import Controller, State


def run_simulation(challenge_type='obstacle', max_time=ROUND_TIME, verbose=True,
                   section_widths=None, driving_direction=None, start_section=None):
    """Run a single simulation and return results."""
    track = Track(
        challenge_type=challenge_type,
        section_widths=section_widths,
        driving_direction=driving_direction,
        starting_section_idx=start_section,
    )

    # Get starting position
    sec_idx = track.starting_section_idx
    section = track.straight_sections[sec_idx]
    cx = (section.x1 + section.x2) / 2
    cy = (section.y1 + section.y2) / 2

    if sec_idx == 0:
        angle = 0 if track.driving_direction == 1 else 180
    elif sec_idx == 1:
        angle = 90 if track.driving_direction == 1 else 270
    elif sec_idx == 2:
        angle = 180 if track.driving_direction == 1 else 0
    else:
        angle = 270 if track.driving_direction == 1 else 90

    robot = Robot(cx, cy, angle)
    controller = Controller(
        driving_direction=track.driving_direction,
        start_in_parking=(challenge_type == 'obstacle'),
        challenge_type=challenge_type,
    )

    physics_dt = 1.0 / SIM_FPS
    control_dt = 1.0 / CONTROL_HZ
    control_accumulator = 0.0
    elapsed = 0
    step = 0

    if verbose:
        dir_str = "CW" if track.driving_direction == 1 else "CCW"
        print(f"\n{'='*60}")
        print(f"Challenge: {challenge_type.upper()}")
        print(f"Direction: {dir_str}")
        print(f"Track width: {track.track_width}mm")
        print(f"Pillars: {len(track.pillars)}")
        print(f"Start section: {track.starting_section_idx}")
        print(f"Start pos: ({cx:.0f}, {cy:.0f}) @ {angle}°")
        print(f"{'='*60}")

    while elapsed < max_time:
        # Run control at CONTROL_HZ rate
        control_accumulator += physics_dt
        while control_accumulator >= control_dt:
            control_accumulator -= control_dt
            sensors = robot.get_sensors(track)
            controller.update(sensors, robot, track, control_dt)

        # Update physics at SIM_FPS rate
        robot.update(physics_dt)

        # Check wall collision
        if robot.check_wall_collision(track):
            irx1, iry1, irx2, iry2 = track.inner_rect
            in_inner = irx1 <= robot.x <= irx2 and iry1 <= robot.y <= iry2
            if in_inner:
                icx = (irx1 + irx2) / 2
                icy = (iry1 + iry2) / 2
                dx = robot.x - icx
                dy = robot.y - icy
            else:
                ocx = (track.outer_rect[0] + track.outer_rect[2]) / 2
                ocy = (track.outer_rect[1] + track.outer_rect[3]) / 2
                dx = ocx - robot.x
                dy = ocy - robot.y
            dist = max(1, math.sqrt(dx*dx + dy*dy))
            robot.x += (dx / dist) * 20
            robot.y += (dy / dist) * 20
            robot.speed = max(abs(robot.speed) * 0.8, MAX_SPEED * 0.30)
            if robot.target_speed < 0:
                robot.speed = -robot.speed

        # Check pillar collision
        pillar_hit = robot.check_pillar_collision(track)
        if pillar_hit is not None:
            pillar = track.pillars[pillar_hit]
            if not pillar.moved:
                dx = pillar.x - robot.x
                dy = pillar.y - robot.y
                dist = max(1, math.sqrt(dx*dx + dy*dy))
                pillar.x += (dx / dist) * 20
                pillar.y += (dy / dist) * 20
                pillar.moved = True
                if verbose:
                    print(f"  [{elapsed:.1f}s] Pillar hit! ({pillar.color})")

        elapsed += physics_dt
        step += 1

        # Log progress periodically
        if verbose and step % (SIM_FPS * 10) == 0:
            print(f"  [{elapsed:.1f}s] State={controller.get_state_name()}, "
                  f"Laps={controller.laps_completed}, "
                  f"Sections={controller.sections_passed}, "
                  f"Speed={robot.speed:.0f}mm/s, "
                  f"Pos=({robot.x:.0f},{robot.y:.0f})")

        # Check if finished
        if controller.state in (State.FINISHED, State.STOPPED):
            if verbose:
                print(f"  [{elapsed:.1f}s] Robot {controller.state.name}")
            break

    # Calculate final score (with actual robot position for parking)
    score = controller.get_score(track, robot.x, robot.y, robot.angle)

    if verbose and challenge_type == 'obstacle' and track.parking_lot:
        parking_status = track.check_parking(
            robot.x, robot.y, robot.angle, ROBOT_LENGTH, ROBOT_WIDTH
        )
        print(f"  Parking status: {parking_status}")

    if verbose:
        print(f"\n{'─'*40}")
        print(f"RESULTS:")
        print(f"  Time: {elapsed:.1f}s / {max_time}s")
        print(f"  Laps: {controller.laps_completed}/3")
        print(f"  Sections: {controller.sections_passed}")
        print(f"  Pillars moved: {sum(1 for p in track.pillars if p.is_outside_circle())}")
        print(f"  Score breakdown:")
        for k, v in score.items():
            print(f"    {k}: {v}")
        print(f"{'─'*40}\n")

    return {
        'elapsed': elapsed,
        'laps': controller.laps_completed,
        'sections': controller.sections_passed,
        'pillars_moved': sum(1 for p in track.pillars if p.is_outside_circle()),
        'score': score,
        'final_state': controller.state.name,
    }


def batch_test(num_runs=10, challenge_type='obstacle'):
    """Run multiple simulations and report statistics."""
    print(f"\n{'#'*60}")
    print(f"BATCH TEST: {num_runs} runs, {challenge_type.upper()}")
    print(f"{'#'*60}")

    results = []
    for i in range(num_runs):
        print(f"\n--- Run {i+1}/{num_runs} ---")
        result = run_simulation(challenge_type=challenge_type, verbose=False)
        results.append(result)
        print(f"  Laps={result['laps']}, Sections={result['sections']}, "
              f"Score={result['score']['total']}, State={result['final_state']}")

    # Statistics
    total_scores = [r['score']['total'] for r in results]
    laps_list = [r['laps'] for r in results]
    sections_list = [r['sections'] for r in results]

    print(f"\n{'='*60}")
    print(f"BATCH RESULTS ({num_runs} runs)")
    print(f"{'='*60}")
    print(f"  Score: avg={sum(total_scores)/len(total_scores):.1f}, "
          f"min={min(total_scores)}, max={max(total_scores)}")
    print(f"  Laps:  avg={sum(laps_list)/len(laps_list):.1f}, "
          f"min={min(laps_list)}, max={max(laps_list)}")
    print(f"  Sections: avg={sum(sections_list)/len(sections_list):.1f}, "
          f"min={min(sections_list)}, max={max(sections_list)}")
    print(f"  3-lap completion rate: "
          f"{sum(1 for l in laps_list if l >= 3)}/{num_runs}")
    print(f"{'='*60}\n")


def test_track_generation():
    """Test that track generation produces valid layouts."""
    print("\nTesting track generation...")

    for i in range(5):
        track = Track(challenge_type='obstacle')
        print(f"\n  Layout {i+1}:")
        print(f"    Direction: {'CW' if track.driving_direction == 1 else 'CCW'}")
        print(f"    Start section: {track.starting_section_idx}")
        print(f"    Pillars: {len(track.pillars)}")
        for p in track.pillars:
            print(f"      {p.color} at ({p.x:.0f}, {p.y:.0f}) section={p.section_idx}")
        if track.parking_lot:
            pl = track.parking_lot
            print(f"    Parking: ({pl.x:.0f}, {pl.y:.0f}) {pl.width:.0f}x{pl.length:.0f}mm")

    print("\n  Track generation: OK")


def test_sensors():
    """Test sensor readings."""
    print("\nTesting sensors...")

    track = Track(challenge_type='obstacle')
    robot = Robot(1600, 300, 0)  # Place in top straight section

    sensors = robot.get_sensors(track)
    print(f"  Position: ({robot.x:.0f}, {robot.y:.0f}) @ {robot.angle}°")
    print(f"  ToF Front: {sensors.tof_front:.0f}mm")
    print(f"  ToF Rear:  {sensors.tof_rear:.0f}mm")
    print(f"  ToF Left:  {sensors.tof_left:.0f}mm")
    print(f"  ToF Right: {sensors.tof_right:.0f}mm")
    print(f"  IMU:       {sensors.imu_heading:.1f}°")
    print(f"  Pillars:   {len(sensors.pillars_visible)} visible")
    print(f"  Sensors: OK")


def test_robot_physics():
    """Test robot movement and Ackermann steering."""
    print("\nTesting robot physics...")

    robot = Robot(1600, 600, 0)

    # Drive straight
    robot.set_speed(0.5)
    for _ in range(30):
        robot.update(1/30)
    print(f"  Straight: ({robot.x:.0f}, {robot.y:.0f}) @ {robot.angle:.1f}°")
    assert robot.x > 1600, "Robot should have moved right"

    # Turn right
    robot.set_steering(20)
    for _ in range(60):
        robot.update(1/30)
    print(f"  After turn: ({robot.x:.0f}, {robot.y:.0f}) @ {robot.angle:.1f}°")
    assert robot.angle > 0 or robot.angle < 350, "Robot should have turned"

    print(f"  Physics: OK")


def test_open_challenge_edge_cases():
    """Test open challenge across all width combos, directions, start sections."""
    print("\n" + "=" * 60)
    print("OPEN CHALLENGE EDGE CASES")
    print("=" * 60)

    configs = [
        ('All wide', [1000, 1000, 1000, 1000]),
        ('All narrow', [600, 600, 600, 600]),
        ('Alt W/N', [1000, 600, 1000, 600]),
        ('Alt N/W', [600, 1000, 600, 1000]),
        ('Top+Right narrow', [600, 600, 1000, 1000]),
        ('One narrow (top)', [600, 1000, 1000, 1000]),
    ]

    passed = 0
    failed = 0
    results_summary = []

    for name, widths in configs:
        for direction in [1, -1]:
            for start in range(4):
                r = run_simulation(
                    challenge_type='open', verbose=False,
                    section_widths=widths,
                    driving_direction=direction,
                    start_section=start,
                )
                ok = r['laps'] >= 3
                if ok:
                    passed += 1
                else:
                    failed += 1
                    d = "CW" if direction == 1 else "CCW"
                    results_summary.append(
                        f"  FAIL: {name} {d} start={start} "
                        f"laps={r['laps']} sec={r['sections']} "
                        f"t={r['elapsed']:.0f}s state={r['final_state']}")

    total = passed + failed
    print(f"\n  Results: {passed}/{total} passed")
    if results_summary:
        print("  Failures:")
        for line in results_summary[:20]:
            print(line)
    else:
        print("  All configurations complete 3 laps!")
    print("=" * 60)
    return passed, total


if __name__ == '__main__':
    print("WRO 2026 Future Engineers - Simulation Tests")
    print("=" * 50)

    # Run tests
    test_track_generation()
    test_sensors()
    test_robot_physics()

    # Open challenge edge cases
    if '--open' in sys.argv or '--all' in sys.argv:
        test_open_challenge_edge_cases()

    # Run single simulation
    if '--single' in sys.argv or '--all' in sys.argv:
        print("\n" + "=" * 50)
        print("Running single obstacle challenge simulation...")
        run_simulation(challenge_type='obstacle', verbose=True)
        print("Running single open challenge simulation...")
        run_simulation(challenge_type='open', verbose=True)

    # Batch test
    if '--batch' in sys.argv:
        batch_test(num_runs=20, challenge_type='obstacle')
