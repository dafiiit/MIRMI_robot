#!/usr/bin/env python3
"""
rtk_sweep_capture — drive the rover to the docking-station sweep grid using ONLY
RTK GPS + compass (no map, no nav2, no depth) and trigger a sensor capture at
every stop.  Facing the station at each capture is verified with GPS + compass,
and optionally cross-checked with the LIDAR.

Motion model (built for a TRACKED rover that starts/stops abruptly)
-------------------------------------------------------------------
* TURN and DRIVE are STRICTLY separated — the rover never turns while moving and
  never drives while turning.  To re-aim it stops, rotates in place, then drives
  straight.  This avoids the curved, jerky paths skid-steer tracks produce.
* All commands are slew-rate limited (software accel limit) so starts/stops are
  gentle.
* "rotate-in-place → drive straight → re-check" loop, with hysteresis so it does
  not thrash between turning and driving.

Facing the station (critical for capture)
-----------------------------------------
At each stop the desired heading is the LIVE RTK bearing from the rover to the
station (GPS), and it is aligned against the EKF/compass heading.  If
--lidar-verify is set and the station is within LIDAR range, forward points in a
height band (default 0.2–0.8 m) are used to confirm — and fine-center — the
heading before capturing.

For each (ray, distance) target:
  1. Refuse to move unless RTK is FIXED and eph <= --rtk-eph (default 5 cm).
  2. TURN in place toward the target, then DRIVE straight (re-aim by stopping).
  3. TURN in place to face the station (GPS bearing + compass, optional LIDAR).
  4. Trigger the docking_test_suite capture and wait for it to finish.

Stepping (default is MANUAL)
----------------------------
By default the run is hand-stepped so you can readjust the rover between arriving
and capturing:
  * publish std_msgs/Empty to  /start_next_measure_point  -> drive to the next point
    and face the station, then WAIT (the node is silent, so you can teleop/readjust).
  * publish std_msgs/Empty to  /start_capture             -> capture at the current pose.
Pass --auto for the hands-off loop (drive+face+capture every point automatically).

SAFETY
------
* Publishing ANY /cmd_vel auto-arms the rover and enters OFFBOARD (cmdvel_node3),
  so this script does NOT move unless you pass --go.  Without --go it runs a dry
  run: prints every target + the live RTK/heading status, no motion.
* If RTK degrades below the gate at any moment, the rover is slewed to a STOP and
  waits until the fix recovers.
* Ctrl-C / completion / abort slews to zero and publishes /stop.

Examples
--------
  python3 rtk_sweep_capture.py --rays 0                       # dry run, 0° ray
  python3 rtk_sweep_capture.py --rays 0 --go --lidar-verify   # small test, for real
  python3 rtk_sweep_capture.py --rays all --go --lidar-verify # full campaign

Prerequisites (process_manager /start_* topics):
  /start_rtk_ntrip   /start_docking_test_suite   (+ livox driver if --lidar-verify)
"""

import argparse
import math
import os
import threading
import time

from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import yaml
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Empty, Bool, Float32
from sensor_msgs.msg import PointCloud2
from px4_msgs.msg import SensorGps, VehicleLocalPosition

R_EARTH = 6_371_000.0
RAY_ORDER = [0, 30, 45, 90, 135, 180, 225, 270, 315, 330]
DIST_ORDER = [10, 9, 8, 7, 6, 5, 4, 3, 2]


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def slew(cur, tgt, max_step):
    """Move cur toward tgt by at most max_step."""
    if tgt > cur:
        return min(tgt, cur + max_step)
    return max(tgt, cur - max_step)


class RtkSweepCapture(Node):
    def __init__(self, args):
        super().__init__('rtk_sweep_capture')
        self.args = args

        cfg_path = args.station or os.path.join(
            os.path.dirname(__file__), '..', 'config', 'dock_station.yaml')
        cfg_path = os.path.realpath(cfg_path)
        with open(cfg_path) as f:
            st = yaml.safe_load(f)['dock_station']
        self.st_lat = float(st['lat'])
        self.st_lon = float(st['lon'])
        self.head_enu = math.radians(float(st['heading_enu_deg']))
        
        self.get_logger().info(
            f'Station loaded lat={self.st_lat:.8f} lon={self.st_lon:.8f}')
            
        # Shift the station centerpoint by 1m in the 180° direction
        shift_dist = 1.5
        shift_angle = self.head_enu + math.pi
        de = shift_dist * math.cos(shift_angle)
        dn = shift_dist * math.sin(shift_angle)
        
        # Convert to lat/lon offsets using the sphere radius R_EARTH
        lat_rad = math.radians(self.st_lat)
        dlat = math.degrees(dn / R_EARTH)
        dlon = math.degrees(de / (R_EARTH * math.cos(lat_rad)))
        
        self.st_lat += dlat
        self.st_lon += dlon
        
        self.get_logger().info(
            f'Station shifted (1m @ 180°) lat={self.st_lat:.8f} lon={self.st_lon:.8f} '
            f'heading_enu={math.degrees(self.head_enu):.2f}°  ({cfg_path})')

        # live state
        self._gps = None          # (lat, lon, fix, eph, sats, stamp)
        self._yaw = None          # (yaw_enu_rad, stamp)
        self._yawhist = deque(maxlen=25)   # (t, yaw_enu) for turn-rate estimation
        self._cloud = None        # (PointCloud2, stamp)
        self._st_det = (False, 0.0)        # YOLO station (detected, stamp)
        self._st_brg = (0.0, 0.0)          # YOLO station (bearing_rad, stamp)
        self._datum = None
        self._lock = threading.Lock()
        self._capture_done = threading.Event()
        self._capture_label = None
        # manual-stepping triggers
        self._ev_next = threading.Event()
        self._ev_capture = threading.Event()

        # commanded velocities (for slew limiting)
        self._vx = 0.0
        self._wz = 0.0

        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(SensorGps, '/fmu/out/vehicle_gps_position',
                                 self._gps_cb, be)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self._lpos_cb, be)
        self.create_subscription(String, '/sweep_test/step_complete',
                                 self._step_cb, be)
        self.create_subscription(Empty, '/start_next_measure_point',
                                 self._next_cb, be)
        self.create_subscription(Empty, '/start_capture', self._capture_cb, be)
        self.create_subscription(Bool, '/station/detected', self._det_cb, be)
        self.create_subscription(Float32, '/station/bearing', self._brg_cb, be)
        if args.lidar_verify:
            self.create_subscription(PointCloud2, args.lidar_topic,
                                     self._cloud_cb, be)

        self._pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self._pub_stop = self.create_publisher(Empty, '/stop', 10)
        self._pub_cfg = self.create_publisher(String, '/sweep_test/cmd/configure', 10)

    # ── callbacks ────────────────────────────────────────────────────────
    def _gps_cb(self, m: SensorGps):
        with self._lock:
            self._gps = (m.latitude_deg, m.longitude_deg, int(m.fix_type),
                         float(m.eph), int(m.satellites_used), time.time())
            if self._datum is None and m.fix_type >= 3:
                self._datum = (m.latitude_deg, m.longitude_deg)

    def _lpos_cb(self, m: VehicleLocalPosition):
        now = time.time()
        y = wrap_pi(math.pi / 2.0 - float(m.heading))
        with self._lock:
            self._yaw = (y, now)
            self._yawhist.append((now, y))

    def _det_cb(self, m: Bool):
        with self._lock:
            self._st_det = (bool(m.data), time.time())

    def _brg_cb(self, m: Float32):
        with self._lock:
            self._st_brg = (float(m.data), time.time())

    def _cloud_cb(self, m: PointCloud2):
        with self._lock:
            self._cloud = (m, time.time())

    def _step_cb(self, m: String):
        self.get_logger().info(f'[capture] step_complete: {m.data[:140]}')
        if self._capture_label is None or self._capture_label in m.data:
            self._capture_done.set()

    def _next_cb(self, _m: Empty):
        self.get_logger().info('[trigger] /start_next_measure_point received')
        self._ev_next.set()

    def _capture_cb(self, _m: Empty):
        self.get_logger().info('[trigger] /start_capture received')
        self._ev_capture.set()

    # ── geometry ─────────────────────────────────────────────────────────
    def _enu(self, lat, lon):
        dlat, dlon = self._datum
        e = R_EARTH * math.radians(lon - dlon) * math.cos(math.radians(dlat))
        n = R_EARTH * math.radians(lat - dlat)
        return e, n

    def target_enu(self, ray_deg, dist_m):
        st_e, st_n = self._enu(self.st_lat, self.st_lon)
        d = self.head_enu + math.radians(ray_deg)
        return st_e + dist_m * math.cos(d), st_n + dist_m * math.sin(d)

    def snapshot(self):
        with self._lock:
            return self._gps, self._yaw, self._datum

    def rtk_ok(self, gps):
        if gps is None:
            return False, 'no GPS'
        _, _, fix, eph, sats, stamp = gps
        if time.time() - stamp > 1.0:
            return False, 'GPS stale'
        if fix < self.args.rtk_fix:
            return False, f'fix={fix}(<{self.args.rtk_fix})'
        if eph > self.args.rtk_eph:
            return False, f'eph={eph*100:.1f}cm(>{self.args.rtk_eph*100:.0f})'
        return True, f'fix={fix} eph={eph*100:.1f}cm sats={sats}'

    # ── low-level command with slew limiting ─────────────────────────────
    def _cmd(self, tvx, twz, dt):
        a = self.args
        self._vx = slew(self._vx, tvx, a.max_lin_accel * dt)
        self._wz = slew(self._wz, twz, a.max_yaw_accel * dt)
        t = Twist()
        t.linear.x = float(self._vx)
        t.angular.z = float(self._wz)
        self._pub_cmd.publish(t)

    def _moving(self):
        return abs(self._vx) > 0.02

    def _turn_cmd(self, err):
        """Yaw command toward heading error, with a stiction floor + deadband."""
        a = self.args
        mag = clamp(a.yaw_gain * abs(err), a.min_turn, a.max_yaw)
        return math.copysign(mag, err)

    def stop(self):
        # hard stop (no slew) on shutdown
        self._vx = self._wz = 0.0
        self._pub_cmd.publish(Twist())
        self._pub_stop.publish(Empty())

    # ── motion primitive: TURN-then-DRIVE to ENU target ──────────────────
    def drive_to(self, te, tn, tag):
        """Reach an ENU target with clean AIM → straight-SEGMENT → re-AIM cycles.
        The rover never steers while driving (tracks are abrupt); instead it drives a
        short straight leg, then stops and re-aims with the no-overshoot turn primitive.
        Near the target it locks on and commits straight to pos_tol."""
        a = self.args
        dt = 1.0 / a.rate_hz
        deadline = time.time() + a.target_timeout

        def brg_err():
            gps, yawd, datum = self.snapshot()
            if gps is None or yawd is None or datum is None:
                return 0.0, False
            if not self.rtk_ok(gps)[0] or time.time() - yawd[1] > 1.0:
                return 0.0, False
            ce, cn = self._enu(gps[0], gps[1])
            return wrap_pi(math.atan2(tn - cn, te - ce) - yawd[0]), True

        while rclpy.ok():
            if time.time() > deadline:
                self.get_logger().error(f'[{tag}] TIMEOUT — abort target')
                self._slew_to_stop(dt)
                return False
            gps, _, datum = self.snapshot()
            if gps is None or datum is None:
                self._cmd(0.0, 0.0, dt)
                time.sleep(dt)
                continue
            ce, cn = self._enu(gps[0], gps[1])
            dist = math.hypot(te - ce, tn - cn)
            if dist <= a.pos_tol:
                self._slew_to_stop(dt)
                self.get_logger().info(f'[{tag}] reached (dist={dist:.2f}m)')
                return True
            lockon = dist <= a.lockon_dist

            # AIM cleanly toward the target (skip when locked on)
            if not lockon:
                e0, valid = brg_err()
                if valid and abs(e0) > math.radians(a.align_tol_deg):
                    self._turn_to(brg_err, math.radians(a.align_tol_deg),
                                 tag + ' aim', allow_pulse=True)

            # DRIVE a straight segment (no steering)
            seg_e, seg_n = ce, cn
            last_log = 0.0
            while rclpy.ok():
                if time.time() > deadline:
                    self._slew_to_stop(dt)
                    return False
                gps, yawd, datum = self.snapshot()
                ok, why = self.rtk_ok(gps)
                yaw_fresh = yawd is not None and time.time() - yawd[1] < 1.0
                if not ok or not yaw_fresh or datum is None:
                    self._cmd(0.0, 0.0, dt)
                    if time.time() - last_log > 2.0:
                        self.get_logger().warn(
                            f'[{tag}] HOLD: {why}{"" if yaw_fresh else " /no-heading"}')
                        last_log = time.time()
                    time.sleep(dt)
                    continue
                ce, cn = self._enu(gps[0], gps[1])
                dist = math.hypot(te - ce, tn - cn)
                herr = wrap_pi(math.atan2(tn - cn, te - ce) - yawd[0])
                if dist <= a.pos_tol:
                    self._slew_to_stop(dt)
                    self.get_logger().info(f'[{tag}] reached (dist={dist:.2f}m)')
                    return True
                lockon = dist <= a.lockon_dist
                seg = math.hypot(ce - seg_e, cn - seg_n)
                if (not lockon) and (abs(herr) > math.radians(a.redirect_tol_deg)
                                     or seg > a.max_seg_m):
                    self._slew_to_stop(dt)
                    if not self._moving():
                        break                       # re-aim at the top
                else:
                    tvx = clamp(a.lin_gain * dist, a.min_creep, a.max_lin)
                    self._cmd(tvx, 0.0, dt)
                if time.time() - last_log > 1.0:
                    self.get_logger().info(
                        f'[{tag}] DRIVE dist={dist:.2f}m herr={math.degrees(herr):+.0f}° '
                        f'seg={seg:.1f}m v={self._vx:.2f} | {why}')
                    last_log = time.time()
                time.sleep(dt)
        return False

    def _slew_to_stop(self, dt):
        self._cmd(0.0, 0.0, dt)

    # ── turn-rate estimate + station snapshot ────────────────────────────
    def _yaw_rate(self):
        """Estimate yaw rate [rad/s] from recent heading history."""
        with self._lock:
            h = list(self._yawhist)
        if len(h) < 2:
            return 0.0
        now = time.time()
        recent = [(t, y) for t, y in h if now - t < 0.4]
        if len(recent) < 2:
            recent = h[-2:]
        t0, y0 = recent[0]
        t1, y1 = recent[-1]
        if t1 - t0 < 1e-3:
            return 0.0
        return wrap_pi(y1 - y0) / (t1 - t0)

    def _station_snapshot(self):
        """Return (detected_and_fresh, bearing_rad, stamp)."""
        with self._lock:
            det, dts = self._st_det
            brg, bts = self._st_brg
        fresh = (time.time() - bts < 1.0) and (time.time() - dts < 1.0)
        return (det and fresh), brg, bts

    def _wait_station(self, timeout):
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            if self._station_snapshot()[0]:
                return True
            time.sleep(0.05)
        return False

    # ── generic in-place turn primitive: coast-to-stop + pulse ───────────
    def _turn_to(self, get_err, tol, label, allow_pulse=True, pulse_only=False):
        """Rotate in place until get_err() (signed rad; wz=copysign(mag,err) reduces it)
        is within tol. Coast-to-stop near the target (uses measured turn rate) and
        pulse for the final approach to avoid the tracks over/undershooting.
        pulse_only=True → never do a fast continuous turn (used for the visual servo,
        where overshooting would lose the station from the camera frame)."""
        a = self.args
        dt = 1.0 / a.rate_hz
        deadline = time.time() + a.face_timeout
        settle_t = None
        pulse_t0 = time.time()
        last_log = 0.0
        while rclpy.ok():
            if time.time() > deadline:
                self.get_logger().error(f'[{label}] turn timeout')
                self._slew_to_stop(dt)
                return False
            e, valid = get_err()
            if not valid:
                self._cmd(0.0, 0.0, dt)
                time.sleep(dt)
                continue
            omega = self._yaw_rate()
            if abs(e) <= tol:
                self._cmd(0.0, 0.0, dt)
                if abs(omega) < a.settle_rate:
                    settle_t = settle_t or time.time()
                    if time.time() - settle_t > a.settle_face_s:
                        self.get_logger().info(f'[{label}] aligned (err={math.degrees(e):+.1f}°)')
                        return True
                else:
                    settle_t = None
                time.sleep(dt)
                continue
            settle_t = None
            if self._moving():                       # never drive while turning
                self._cmd(0.0, 0.0, dt)
                time.sleep(dt)
                continue
            lead = abs(omega) * a.turn_lead_s
            if (not pulse_only) and abs(e) <= lead and abs(omega) > 0.05:
                twz = 0.0                            # coast — momentum carries to target
            elif (not pulse_only) and abs(e) > math.radians(a.turn_coarse_deg):
                twz = math.copysign(a.max_yaw, e)    # far → continuous fast turn
            elif allow_pulse or pulse_only:
                ph = (time.time() - pulse_t0) % (a.pulse_on_s + a.pulse_off_s)
                twz = math.copysign(a.min_turn, e) if ph < a.pulse_on_s else 0.0
            else:
                twz = math.copysign(a.min_turn, e)   # slow continuous
            self._cmd(0.0, twz, dt)
            if time.time() - last_log > 1.0:
                self.get_logger().info(
                    f'[{label}] err={math.degrees(e):+.1f}° w={omega:+.2f} cmd={twz:+.2f}')
                last_log = time.time()
            time.sleep(dt)
        return False

    # ── acquire: pulse-sweep toward the station, STOP the instant YOLO sees it ──
    def _acquire_station(self, gps_err, tag):
        """Rotate (slow pulses) toward the GPS bearing until the YOLO detector sees
        the station, then stop.  Stopping on detection (not on reaching the imperfect
        GPS/compass bearing) means we never overshoot the station out of the frame."""
        a = self.args
        dt = 1.0 / a.rate_hz
        if not a.use_yolo:
            return self._turn_to(gps_err, math.radians(a.acquire_tol_deg),
                                 tag + ' acquire', allow_pulse=True)
        if self._station_snapshot()[0]:
            self.get_logger().info(f'[{tag}] station already in view')
            return True
        self.get_logger().info(f'[{tag}] acquiring station (slow pulse-sweep) ...')
        deadline = time.time() + a.acquire_timeout
        pulse_t0 = time.time()
        last_log = 0.0
        while rclpy.ok() and time.time() < deadline:
            if self._station_snapshot()[0]:
                # stop and settle so the first servo frame is steady
                t0 = time.time()
                while time.time() - t0 < 0.5:
                    self._cmd(0.0, 0.0, dt)
                    time.sleep(dt)
                self.get_logger().info(f'[{tag}] station ACQUIRED by YOLO')
                return True
            e, valid = gps_err()
            direction = math.copysign(1.0, e) if (valid and abs(e) > math.radians(5)) else 1.0
            ph = (time.time() - pulse_t0) % (a.acquire_pulse_on + a.acquire_pulse_off)
            twz = direction * a.min_turn if ph < a.acquire_pulse_on else 0.0
            self._cmd(0.0, twz, dt)
            if time.time() - last_log > 1.5:
                self.get_logger().info(
                    f'[{tag}] acquiring... gps_err='
                    f'{math.degrees(e):.0f}°' if valid else f'[{tag}] acquiring... (no RTK bearing)')
                last_log = time.time()
            time.sleep(dt)
        self.get_logger().warn(f'[{tag}] acquire timeout — station not seen by YOLO')
        return False

    # ── face the station: acquire (YOLO-gated) → pulsed visual servo ─────
    def face_station(self, dist_hint, tag):
        a = self.args

        def gps_err():
            gps, yawd, datum = self.snapshot()
            if gps is None or yawd is None or datum is None:
                return 0.0, False
            if not self.rtk_ok(gps)[0] or time.time() - yawd[1] > 1.0:
                return 0.0, False
            ce, cn = self._enu(gps[0], gps[1])
            se, sn = self._enu(self.st_lat, self.st_lon)
            return wrap_pi(math.atan2(sn - cn, se - ce) - yawd[0]), True

        # 1) swing to the GPS bearing (already points at the station within ~10°, well
        #    inside the camera FOV) using the no-overshoot pulsed turn → station in view
        acquired = self._turn_to(gps_err, math.radians(a.acquire_tol_deg),
                                 tag + ' acquire', allow_pulse=True)

        # 2) PULSE-ONLY visual servo on the YOLO bearing (cannot overshoot out of frame)
        if a.use_yolo and acquired:
            def vis_err():
                det, brg, _ = self._station_snapshot()
                if not det:
                    return 0.0, False
                return -brg, True            # bearing>0 = station right → turn right (wz<0)
            if self._turn_to(vis_err, math.radians(a.face_vis_tol_deg),
                            tag + ' YOLO', pulse_only=True):
                det, brg, _ = self._station_snapshot()
                ok_lidar = self._lidar_sanity(dist_hint)
                self.get_logger().info(
                    f'[{tag}] facing CONFIRMED by YOLO (bearing={math.degrees(brg):+.1f}°'
                    f'{", LIDAR ok" if ok_lidar else ""})')
                return True
            self.get_logger().warn(f'[{tag}] YOLO servo timeout — GPS+compass fallback')
        elif a.use_yolo:
            self.get_logger().warn(f'[{tag}] station not acquired by YOLO — GPS+compass fallback')

        # 3) fallback: GPS+compass facing (camera unavailable)
        self._turn_to(gps_err, math.radians(a.face_tol_deg), tag + ' gps-face',
                     allow_pulse=True)
        return True

    def _lidar_sanity(self, dist_hint):
        """True if the LIDAR sees a forward object near the expected range."""
        if not self.args.lidar_verify or dist_hint > self.args.lidar_max_check:
            return False
        n, _ = self._lidar_forward(dist_hint)
        return n >= self.args.lidar_min_points

    def _lidar_forward(self, dist_hint):
        """Return (n_points, mean_azimuth_rad) of forward points in the height band
        and a range window around dist_hint.  Azimuth 0 = straight ahead (+x)."""
        a = self.args
        with self._lock:
            entry = self._cloud
        if entry is None or time.time() - entry[1] > 1.0:
            return 0, 0.0
        try:
            import numpy as np
            from sensor_msgs_py import point_cloud2
            pts = point_cloud2.read_points(entry[0], field_names=['x', 'y', 'z'],
                                           skip_nans=True)
            if len(pts) == 0:
                return 0, 0.0
            x = np.asarray(pts['x'], dtype=float)
            y = np.asarray(pts['y'], dtype=float)
            z = np.asarray(pts['z'], dtype=float)
        except Exception as e:
            self.get_logger().warn(f'LIDAR parse failed: {e}')
            return 0, 0.0
        rng = np.hypot(x, y)
        az = np.arctan2(y, x)
        rmin = max(0.3, dist_hint - a.lidar_range_tol)
        rmax = dist_hint + a.lidar_range_tol
        mask = ((z >= a.lidar_z_min) & (z <= a.lidar_z_max) &
                (rng >= rmin) & (rng <= rmax) &
                (np.abs(az) <= math.radians(a.lidar_cone_deg)))
        n = int(np.count_nonzero(mask))
        if n == 0:
            return 0, 0.0
        return n, float(np.median(az[mask]))

    # ── capture ──────────────────────────────────────────────────────────
    def capture(self, ray_deg, dist_m):
        if self.args.no_capture:
            self.get_logger().info('[capture] skipped (--no-capture)')
            return True
        sign = 'p' if ray_deg >= 0 else 'n'
        self._capture_label = (f"dist_{float(dist_m):04.1f}m_ang_"
                               f"{sign}{abs(int(ray_deg)):03d}deg").replace('.', 'p')
        self._capture_done.clear()
        payload = ('{"mode":"outdoor","scenario":"custom","sensors":"both",'
                   f'"distance":{float(dist_m)},"angle":{float(ray_deg)}}}')
        self.get_logger().info(f'[capture] configure -> {payload}')
        for _ in range(3):
            self._pub_cfg.publish(String(data=payload))
            time.sleep(0.1)
        if self._capture_done.wait(timeout=self.args.capture_timeout):
            self.get_logger().info('[capture] done')
            return True
        self.get_logger().error('[capture] TIMEOUT')
        return False

    # ── mission ──────────────────────────────────────────────────────────
    def _await_datum(self):
        self.get_logger().info('Waiting for first RTK fix + heading ...')
        t0 = time.time()
        while rclpy.ok():
            _, yawd, datum = self.snapshot()
            if datum is not None and yawd is not None:
                return True
            if time.time() - t0 > 30:
                self.get_logger().error('No RTK datum / heading after 30 s.')
                return False
            time.sleep(0.2)
        return False

    def _report_target(self, tag, te, tn):
        gps, _, _ = self.snapshot()
        ce, cn = self._enu(gps[0], gps[1])
        ok, why = self.rtk_ok(gps)
        self.get_logger().info(
            f'\n--- TARGET {tag} ---  target ENU=({te:.2f},{tn:.2f})  '
            f'range_now={math.hypot(te-ce, tn-cn):.2f}m  '
            f'RTK:{why} [{"OK" if ok else "BLOCKED"}]')

    def _drive_and_face(self, ray, dist, tag):
        """Drive to a target and face the station. Returns True on success."""
        if not self.args.go:
            self.get_logger().info(f'[{tag}] dry run — no motion')
            return True
        te, tn = self.target_enu(ray, dist)
        if not self.drive_to(te, tn, tag):
            return False
        if self.args.no_face:
            self.get_logger().info(f'[{tag}] reached — facing left to operator (--no-face)')
            return True
        if not self.face_station(dist, tag) and not self.args.keep_going:
            return False
        return True

    def _do_capture(self, ray, dist, tag):
        if not self.args.go:
            self.get_logger().info(f'[{tag}] dry run — would capture (no trigger sent)')
            return True
        time.sleep(self.args.settle_s)
        return self.capture(ray, dist)

    def run_mission(self, rays, dists):
        if not self._await_datum():
            return
        targets = [(r, d) for r in rays for d in dists]
        if self.args.auto:
            self._run_auto(targets)
        else:
            self._run_manual(targets)
        self.stop()
        self.get_logger().info('\nMission finished — rover stopped.')

    def _run_auto(self, targets):
        self.get_logger().info(
            f'\n{"="*64}\n  {"DRY RUN (no motion)" if not self.args.go else "LIVE DRIVE"}'
            f'  —  AUTO  —  {len(targets)} targets'
            f'{"  | LIDAR verify ON" if self.args.lidar_verify else ""}\n{"="*64}')
        for i, (ray, dist) in enumerate(targets):
            if i < self.args.start_index:
                continue
            tag = f'{i+1}/{len(targets)} ray{ray}/{dist}m'
            self._report_target(tag, *self.target_enu(ray, dist))
            if not self.args.go:
                continue
            if not self._drive_and_face(ray, dist, tag):
                if self.args.keep_going:
                    self.get_logger().warn(f'[{tag}] drive/facing failed — skipping (--keep-going)')
                    continue
                self.get_logger().error('Drive/facing failed — stopping mission.')
                break
            if not self._do_capture(ray, dist, tag) and not self.args.keep_going:
                self.get_logger().error('Capture failed — stopping mission.')
                break

    def _run_manual(self, targets):
        n = len(targets)
        self.get_logger().info(
            f'\n{"="*64}\n  {"DRY RUN (no motion)" if not self.args.go else "LIVE DRIVE"}'
            f'  —  MANUAL stepping  —  {n} targets'
            f'{"  | LIDAR verify ON" if self.args.lidar_verify else ""}\n'
            f'  ros2 topic pub --once /start_next_measure_point std_msgs/Empty "{{}}"\n'
            f'  ros2 topic pub --once /start_capture            std_msgs/Empty "{{}}"\n{"="*64}')
        next_idx = 0
        current = None     # (ray, dist, tag) of the point we have driven to
        self._ev_next.clear()
        self._ev_capture.clear()
        while rclpy.ok():
            if self._ev_next.is_set():
                self._ev_next.clear()
                self._ev_capture.clear()           # drop stale capture presses
                if next_idx >= n:
                    self.get_logger().info('All points done — nothing to drive to.')
                else:
                    ray, dist = targets[next_idx]
                    tag = f'{next_idx+1}/{n} ray{ray}/{dist}m'
                    self._report_target(tag, *self.target_enu(ray, dist))
                    if self._drive_and_face(ray, dist, tag):
                        current = (ray, dist, tag)
                        next_idx += 1
                        self.get_logger().info(
                            f'[{tag}] AT POINT — readjust if needed, then '
                            f'publish /start_capture.  (/start_next_measure_point to skip.)')
                    else:
                        self.get_logger().error(f'[{tag}] drive/facing failed — '
                                                f'fix and retry /start_next_measure_point.')
                        self.stop()
            elif self._ev_capture.is_set():
                self._ev_capture.clear()
                if current is None:
                    self.get_logger().warn('[capture] no point reached yet — '
                                           'publish /start_next_measure_point first.')
                else:
                    ray, dist, tag = current
                    self._do_capture(ray, dist, tag)
                    self.get_logger().info(
                        f'[{tag}] captured.  /start_next_measure_point for the next point, '
                        f'or /start_capture to capture again.')
            else:
                time.sleep(0.1)


def parse_rays(s):
    return RAY_ORDER if s.strip().lower() == 'all' \
        else [int(x) for x in s.split(',') if x.strip()]


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--rays', default='0', help='comma list of ray angles or "all"')
    p.add_argument('--distances', default=','.join(str(d) for d in DIST_ORDER))
    p.add_argument('--go', action='store_true', help='ACTUALLY drive (default: dry run)')
    p.add_argument('--auto', action='store_true',
                   help='hands-off: drive+face+capture every point automatically. '
                        'Default is MANUAL stepping via /start_next_measure_point + /start_capture.')
    p.add_argument('--no-capture', action='store_true')
    p.add_argument('--no-face', dest='no_face', action='store_true',
                   help='do NOT auto-rotate to face the station after arriving '
                        '(operator orients the rover manually before capturing)')
    p.add_argument('--keep-going', action='store_true')
    p.add_argument('--station', default='')
    # RTK gate (drive only when accuracy is good enough)
    p.add_argument('--rtk-fix', type=int, default=5,
                   help='min fix_type to drive: 5=RTK_FLOAT (eph-gated, default), '
                        '6=RTK_FIXED (stricter)')
    p.add_argument('--rtk-eph', type=float, default=0.05,
                   help='max horizontal accuracy eph in m to drive (default 0.05 = 5 cm)')
    p.add_argument('--start-index', dest='start_index', type=int, default=0,
                   help='skip the first N targets (resume an interrupted auto run)')
    # control — cmd_vel values are NORMALIZED (1.0 = full, like the Foxglove teleop).
    # Tracks have high stiction: commands below ~0.5 don't move it.  turn/drive separated.
    p.add_argument('--max-lin', dest='max_lin', type=float, default=0.7,
                   help='forward cmd_vel.linear.x while driving (normalized)')
    p.add_argument('--max-yaw', dest='max_yaw', type=float, default=0.8,
                   help='cmd_vel.angular.z while turning (normalized)')
    p.add_argument('--min-creep', dest='min_creep', type=float, default=0.5,
                   help='drive stiction floor — must exceed what makes the tracks move')
    p.add_argument('--min-turn', dest='min_turn', type=float, default=0.7,
                   help='turn stiction floor — must exceed what makes it rotate')
    p.add_argument('--max-lin-accel', dest='max_lin_accel', type=float, default=2.0)
    p.add_argument('--max-yaw-accel', dest='max_yaw_accel', type=float, default=3.0)
    p.add_argument('--lin-gain', dest='lin_gain', type=float, default=0.7)
    p.add_argument('--yaw-gain', dest='yaw_gain', type=float, default=1.5)
    p.add_argument('--pos-tol', dest='pos_tol', type=float, default=0.30,
                   help='stop within this many metres of the target (coarse; operator '
                        'fine-positions). Tracks overshoot, so keep generous.')
    p.add_argument('--lockon-dist', dest='lockon_dist', type=float, default=0.5,
                   help='within this range, commit to a straight line and stop (no re-aim)')
    p.add_argument('--align-tol-deg', dest='align_tol_deg', type=float, default=6.0,
                   help='aim to within this before each straight drive segment')
    p.add_argument('--redirect-tol-deg', dest='redirect_tol_deg', type=float, default=12.0,
                   help='stop & re-aim if heading drifts past this while driving')
    p.add_argument('--max-seg-m', dest='max_seg_m', type=float, default=1.5,
                   help='re-aim at least every this many metres of straight driving')
    p.add_argument('--face-tol-deg', dest='face_tol_deg', type=float, default=5.0)
    # facing: GPS coarse acquire -> YOLO visual servo
    p.add_argument('--use-yolo', dest='use_yolo', action='store_true', default=True,
                   help='use the YOLO /station/bearing for accurate facing (default on)')
    p.add_argument('--no-yolo', dest='use_yolo', action='store_false')
    p.add_argument('--acquire-tol-deg', dest='acquire_tol_deg', type=float, default=12.0,
                   help='GPS+compass acquire tolerance when YOLO is unavailable')
    p.add_argument('--acquire-pulse-on', dest='acquire_pulse_on', type=float, default=0.25,
                   help='acquire sweep: turn-pulse duration (slow so YOLO catches it)')
    p.add_argument('--acquire-pulse-off', dest='acquire_pulse_off', type=float, default=0.8,
                   help='acquire sweep: settle gap (≥ one YOLO frame) between pulses')
    p.add_argument('--acquire-timeout', dest='acquire_timeout', type=float, default=45.0)
    p.add_argument('--face-vis-tol-deg', dest='face_vis_tol_deg', type=float, default=3.5,
                   help='YOLO visual facing tolerance (station centred to this)')
    p.add_argument('--yolo-wait-s', dest='yolo_wait_s', type=float, default=3.0)
    # turn primitive (anti over/undershoot)
    p.add_argument('--turn-coarse-deg', dest='turn_coarse_deg', type=float, default=25.0,
                   help='above this error → continuous fast turn; below → coast/pulse')
    p.add_argument('--turn-lead-s', dest='turn_lead_s', type=float, default=0.5,
                   help='coast-to-stop lead time (× turn rate = early-stop angle)')
    p.add_argument('--pulse-on-s', dest='pulse_on_s', type=float, default=0.4)
    p.add_argument('--pulse-off-s', dest='pulse_off_s', type=float, default=0.5)
    p.add_argument('--settle-rate', dest='settle_rate', type=float, default=0.06,
                   help='|yaw rate| below this counts as stopped (rad/s)')
    p.add_argument('--settle-face-s', dest='settle_face_s', type=float, default=0.4)
    p.add_argument('--dwell-s', dest='dwell_s', type=float, default=0.4)
    p.add_argument('--rate-hz', dest='rate_hz', type=float, default=50.0,
                   help='cmd_vel publish rate; MUST stay above cmdvel_node3 deadman (20 Hz) '
                        'incl. loop overhead, so default 50 Hz')
    p.add_argument('--settle-s', dest='settle_s', type=float, default=1.5)
    p.add_argument('--target-timeout', dest='target_timeout', type=float, default=180.0)
    p.add_argument('--face-timeout', dest='face_timeout', type=float, default=60.0)
    p.add_argument('--capture-timeout', dest='capture_timeout', type=float, default=60.0)
    # LIDAR facing cross-check
    p.add_argument('--lidar-verify', action='store_true',
                   help='confirm/fine-center facing with the LIDAR')
    p.add_argument('--lidar-require', action='store_true',
                   help='refuse to capture unless the LIDAR confirms an object ahead')
    p.add_argument('--lidar-topic', default='/livox/lidar')
    p.add_argument('--lidar-z-min', dest='lidar_z_min', type=float, default=0.2)
    p.add_argument('--lidar-z-max', dest='lidar_z_max', type=float, default=0.8)
    p.add_argument('--lidar-cone-deg', dest='lidar_cone_deg', type=float, default=15.0)
    p.add_argument('--lidar-range-tol', dest='lidar_range_tol', type=float, default=1.5)
    p.add_argument('--lidar-center-tol-deg', dest='lidar_center_tol_deg', type=float, default=3.0)
    p.add_argument('--lidar-min-points', dest='lidar_min_points', type=int, default=8)
    p.add_argument('--lidar-max-check', dest='lidar_max_check', type=float, default=6.0,
                   help='only use LIDAR facing-check when station is within this range')
    args = p.parse_args()

    rays = parse_rays(args.rays)
    dists = [float(x) for x in args.distances.split(',') if x.strip()]

    rclpy.init()
    node = RtkSweepCapture(args)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        node.run_mission(rays, dists)
    except KeyboardInterrupt:
        node.get_logger().warn('Interrupted — stopping rover.')
    finally:
        try:
            node.stop()
            time.sleep(0.2)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
