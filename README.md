# Artemis — WRO 2026 Future Engineers Autonomous Driving

**Team:** UGhana Robotics  
**Challenge:** Open Challenge (Autonomous Lap Completion with Parking)

This repository contains the autonomous driving simulation and control system for the WRO 2026 Future Engineers competition.

## Overview

Artemis is an autonomous vehicle designed to complete the WRO 2026 Open Challenge. The system uses:
- **PD wall-following control** for precision lateral centering
- **Real-time sensor fusion** (ToF + IMU + color detection)
- **Adaptive corner navigation** for varying track widths (600mm–1000mm)
- **State machine** for robust multi-phase driving

## Directory Structure

```
artemis/
├── README.md                    # This file
├── src/                         # Control software
│   ├── README.md
│   ├── open-challenge-sim/      # Open challenge simulation
│   │   ├── config.py            # Constants and PD gains
│   │   ├── controller.py        # State machine and control
│   │   ├── robot.py             # Robot model and sensors
│   │   ├── track.py             # Track and collision detection
│   │   ├── sim_viewer.py        # Real-time Pygame visualizer
│   │   ├── test_pd_tuning.py    # 108-config test suite
│   │   └── test_headless.py     # Headless integration test
│   ├── obstacle-sim/            # Obstacle challenge sim (planned)
│   └── robot/                   # Physical robot code (planned)
├── schemes/                     # Electromechanical diagrams
├── models/                      # CAD models for 3D printing
├── v-photos/                    # Vehicle photos
├── t-photos/                    # Team photos
└── video/                       # Competition video link
```

## Quick Start

### Requirements
- Python 3.8+
- pygame (for visualization)

### Run Simulation
```bash
cd src/open-challenge-sim
python sim_viewer.py
```

### Run Tests
```bash
cd src/open-challenge-sim
python test_pd_tuning.py
```

## Simulation Architecture

### Core Modules

1. **config.py** — Global constants
   - Control parameters (PD gains, speed limits)
   - Track dimensions and sensor specs
   - Challenge-specific settings

2. **robot.py** — Robot model
   - 2D kinematic simulation
   - Sensor simulation (ToF, IMU, color detection)
   - Motor command processing

3. **track.py** — Track representation
   - Outer/inner walls and sections
   - Color line detection zones
   - Collision detection

4. **controller.py** — Autonomous control
   - State machine (WALL_FOLLOWING, CORNER_TURN, PARKING, etc.)
   - PD wall-following algorithm
   - Pillar avoidance and corner navigation
   - Three-point turn and parking maneuvers

5. **test_pd_tuning.py** — Validation suite
   - 108 test configurations covering:
     - Centered placements
     - Lateral offset tolerances (±100mm)
     - Heading entry errors (±12°)
     - Mixed offset + angle worst-cases

6. **sim_viewer.py** — Real-time visualizer
   - Live track rendering
   - Robot trajectory visualization
   - Sensor ray display
   - Real-time telemetry (position, heading, state, sensors)

## Performance

**Current Status:** 182/208 tests passing (87.5% success rate)
- Phase 1 (straight-line): 20/20 ✓
- Phase 2 (corner exit): 16/16 ✓
- Phase 3 (off-center): 57/60 (3 failures on 1000mm wide with negative offset)
- Phase 4 (offset+angle): 16/16 ✓
- Phase 5 (corner entry angle): 73/96 (23 failures on 1000mm wide at ±5° to ±12°)

## Known Challenges

1. **Wide track corner entry** — Non-cardinal heading entry on 1000mm tracks requires tuning
2. **Negative lateral offset** — Left-wall pressure on wide tracks shows systematic bias

## Controls (Simulator)

| Key | Action |
|-----|--------|
| SPACE | Pause/Resume |
| R | Restart current config |
| ←/→ | Previous/Next config |
| A | Auto-play all 108 tests |
| T | Toggle sensor rays |
| P | Toggle path trail |
| G | Toggle grid |
| L | Open config list |
| S | Screenshot |
| Q | Quit |

## Next Steps

- [ ] Fine-tune PD gains for wide-track corner entry
- [ ] Optimize heading correction during corner turns
- [ ] Complete obstacle challenge implementation
- [ ] Hardware integration and real-world testing

---

*Last updated: 2026-05-18*
