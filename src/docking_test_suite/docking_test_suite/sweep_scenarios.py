"""Step generators for sensor sweep test scenarios.

Each function returns a list of step dicts:
    {
        'label':       str   # file-safe identifier, e.g. 'dist_08p0m'
        'value':       float # distance in m or angle in degrees
        'instruction': str   # human-readable placement instruction
        'scenario':    str   # 'distance' | 'angular'
    }

The step list is ordered from first to last capture.
"""


def distance_sweep_steps(start_m: float = 10.0,
                          stop_m: float = 1.0,
                          step_m: float = 1.0) -> list:
    """Return ordered steps for the distance sweep.

    Default: 10 m → 1 m in 1 m increments (10 steps, robot stays fixed,
    operator moves the target closer each time).

    Parameters
    ----------
    start_m : float
        Starting distance in metres (farthest).
    stop_m : float
        Final distance in metres (closest).
    step_m : float
        Increment between steps (positive).
    """
    n_steps = round((start_m - stop_m) / step_m)
    distances = [start_m - i * step_m for i in range(n_steps + 1)]
    steps = []
    for i, d in enumerate(distances):
        # Label: dist_08p0m  (dot replaced with 'p' for filesystem safety)
        label = f"dist_{d:04.1f}m".replace('.', 'p')
        instr = (
            f"[DISTANCE SWEEP | step {i + 1}/{len(distances)}]  "
            f"Place the AprilTag target at {d:.1f} m directly in front of "
            f"the robot.  Robot stays fixed — only you move."
        )
        steps.append({
            'label': label,
            'value': d,
            'instruction': instr,
            'scenario': 'distance',
        })
    return steps


def angular_sweep_steps(fixed_dist_m: float = 3.0,
                         start_deg: int = 0,
                         stop_deg: int = 180,
                         step_deg: int = 20) -> list:
    """Return ordered steps for the angular sweep.

    Default: 0° → 180° in 20° increments (10 steps) at a fixed 3 m distance.
    Angles are measured CCW (left) from the robot's forward axis.
    The robot stays fixed — you carry the target around.

    Parameters
    ----------
    fixed_dist_m : float
        Distance at which the target is placed for every step.
    start_deg : int
        Starting angle in degrees (0 = directly in front).
    stop_deg : int
        Final angle in degrees.
    step_deg : int
        Increment between steps.
    """
    angles = list(range(start_deg, stop_deg + step_deg, step_deg))
    steps = []
    for i, a in enumerate(angles):
        sign = 'p' if a >= 0 else 'n'
        label = f"ang_{sign}{abs(a):03d}deg"

        if a == 0:
            direction_str = "0° — directly in front of the robot"
        elif a == 180:
            direction_str = "180° — directly behind the robot"
        elif a > 0:
            direction_str = f"{a}° to the LEFT (CCW) of the robot's forward axis"
        else:
            direction_str = f"{abs(a)}° to the RIGHT (CW) of the robot's forward axis"

        instr = (
            f"[ANGULAR SWEEP | step {i + 1}/{len(angles)}]  "
            f"Place the AprilTag target at {fixed_dist_m:.1f} m, "
            f"{direction_str}.  "
            f"Keep the tag face pointing toward the robot.  "
            f"Robot stays fixed — only you move."
        )
        steps.append({
            'label': label,
            'value': float(a),
            'instruction': instr,
            'scenario': 'angular',
            'fixed_dist_m': fixed_dist_m,
        })
    return steps
