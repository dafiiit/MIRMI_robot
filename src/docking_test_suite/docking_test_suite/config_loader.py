"""Load and validate the docking test suite YAML configuration."""

import os
import yaml


DEFAULT_CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'config', 'test_config.yaml',
)


def load_config(path: str = None) -> dict:
    """Load the YAML config file and return the parsed dict.

    Parameters
    ----------
    path : str, optional
        Absolute or ``~``-relative path.  Falls back to the shipped
        ``config/test_config.yaml`` inside the package source tree, then to
        the colcon-installed share directory.
    """
    if path is None:
        path = DEFAULT_CONFIG_PATH
        if not os.path.isfile(path):
            # Try the installed share location
            try:
                from ament_index_python.packages import get_package_share_directory
                path = os.path.join(
                    get_package_share_directory('docking_test_suite'),
                    'config', 'test_config.yaml',
                )
            except Exception:
                pass

    path = os.path.expanduser(path)
    if not os.path.isfile(path):
        raise FileNotFoundError(f'Config file not found: {path}')

    with open(path, 'r') as f:
        cfg = yaml.safe_load(f)

    _validate(cfg)
    return cfg


def _validate(cfg: dict):
    """Minimal sanity checks on required keys."""
    # Only the 'topics' section is checked for the original tests.
    # The sweep_test node only needs 'sweep_test' + 'google_drive', which are
    # validated at runtime inside sensor_sweep_node.py, not here.
    if 'topics' not in cfg:
        return  # sweep-only config — skip topic checks

    t = cfg['topics']
    for key in ['apriltag_detections', 'camera_info', 'cmd_vel', 'detection_msg_type']:
        if key not in t:
            raise KeyError(f'Missing required topic config: topics.{key}')

    if t['detection_msg_type'] not in ('apriltag_msgs', 'isaac_ros'):
        raise ValueError(
            f"detection_msg_type must be 'apriltag_msgs' or 'isaac_ros', "
            f"got '{t['detection_msg_type']}'")

    # Calibration checks only apply to the original tests that use Vicon
    cal = cfg.get('calibration', {})
    for offset_key in ['vicon_target_to_real_target', 'vicon_robot_to_camera']:
        off = cal.get(offset_key)
        if off is None:
            continue  # optional when not using Vicon
        if 'translation' not in off or 'rotation' not in off:
            raise KeyError(
                f'calibration.{offset_key} must define translation and rotation')
        if len(off['translation']) != 3:
            raise ValueError(f'calibration.{offset_key}.translation must have 3 elements')
        if len(off['rotation']) != 4:
            raise ValueError(f'calibration.{offset_key}.rotation must have 4 elements')



def get_output_dir(cfg: dict) -> str:
    """Return the expanded and created output directory."""
    d = os.path.expanduser(cfg['recording']['output_dir'])
    os.makedirs(d, exist_ok=True)
    return d
