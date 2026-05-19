Control Software
====

This directory contains the control software for Team Artemis's WRO 2026 Future Engineers vehicle.

## Structure

```
src/
├── README.md                      # This file
├── open-challenge-sim/            # Open challenge simulation & testing
│   ├── config.py                  # Simulation constants and PD gains
│   ├── controller.py              # State machine and control logic
│   ├── robot.py                   # Robot model and sensor simulation
│   ├── track.py                   # Track layout and collision detection
│   ├── sim_viewer.py              # Real-time Pygame visualizer
│   ├── test_pd_tuning.py          # 108-config test suite
│   └── test_headless.py           # Headless integration test
├── obstacle-sim/                  # Obstacle challenge simulation (planned)
└── robot/                         # Physical robot control code (planned)
```

## Open Challenge Simulation

See [open-challenge-sim/README.md](open-challenge-sim/README.md) for full documentation.

```bash
cd open-challenge-sim
python sim_viewer.py        # real-time visualizer
python test_pd_tuning.py    # run test suite
```
