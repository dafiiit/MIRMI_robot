#!/usr/bin/env python3
"""
compute_dock_pose.py

Determines the docking station's GPS position and orientation from sweep test
data collected at known distances and angles around the station.

Geometric principle:
  At every measurement pose the robot faces the docking station, so:
    station_ENU = robot_ENU + D * forward_ENU(yaw)
  where yaw is derived from the PX4 quaternion and D is the known distance.
  All estimates are GPS-quality-weighted and averaged.

Station orientation:
  The station's 0° axis direction θ_ds satisfies:
    θ_ds = (robot_yaw_ENU + π) − angle_rad
  for each measurement. We take the weighted circular mean.

Usage:
  python3 compute_dock_pose.py

Output:
  Prints a residual report and writes:
    src/navigation/config/dock_station.yaml
"""

import os, sys, math, glob, csv
import numpy as np

SESSIONS = [
    '/home/holybro/sweep_test_data/combined_series/20260521_140405_out_cust_both',
    '/home/holybro/sweep_test_data/combined_series/20260521_151623_out_cust_both',
]

# ── helpers ──────────────────────────────────────────────────────────────────

def parse_filename(path):
    """
    dist_05p0m_ang_p045deg.csv  →  (5.0, 45.0)
    Negative angles encoded as 'n': ang_n030deg → -30°
    """
    base = os.path.basename(path).replace('.csv', '')
    parts = base.split('_')
    dist_m = float(parts[1].replace('p', '.').replace('m', ''))
    ang_raw = parts[3].replace('deg', '')
    if ang_raw.startswith('n'):
        ang_deg = -float(ang_raw[1:])
    else:
        ang_deg = float(ang_raw.lstrip('p'))
    return dist_m, ang_deg

def quat_to_yaw_ned(qw, qx, qy, qz):
    """Extract yaw in NED frame (radians, 0=North, CW positive)."""
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))

def yaw_ned_to_enu(yaw_ned):
    """NED yaw → ENU yaw (CCW from East, radians)."""
    return math.pi / 2.0 - yaw_ned

def gps_to_enu(lat, lon, datum_lat, datum_lon):
    """Flat-Earth WGS84 → ENU (east_m, north_m) relative to datum."""
    R = 6_371_000.0
    east  = R * math.radians(lon - datum_lon) * math.cos(math.radians(datum_lat))
    north = R * math.radians(lat - datum_lat)
    return east, north

def enu_to_gps(east, north, datum_lat, datum_lon):
    """ENU offset → absolute GPS lat/lon."""
    R = 6_371_000.0
    lat = datum_lat + math.degrees(north / R)
    lon = datum_lon + math.degrees(east / (R * math.cos(math.radians(datum_lat))))
    return lat, lon

def circular_mean(angles_rad, weights=None):
    """Weighted circular mean of a list of angles."""
    if weights is None:
        weights = [1.0] * len(angles_rad)
    sw = sum(w * math.sin(a) for a, w in zip(angles_rad, weights))
    cw = sum(w * math.cos(a) for a, w in zip(angles_rad, weights))
    return math.atan2(sw, cw)

# ── per-file loader ───────────────────────────────────────────────────────────

def load_measurement(csv_path):
    """
    Load one CSV and return a dict with:
      lat, lon, alt       — median GPS position
      yaw_enu             — median robot heading in ENU (radians)
      gps_quality         — 1 / GPS position std-dev (higher = better GPS)
      dist_m, ang_deg     — from filename
      n_rows              — number of valid rows
    Returns None if there are fewer than 5 usable rows.
    """
    rows = []
    with open(csv_path, newline='') as f:
        for r in csv.DictReader(f):
            try:
                lat = float(r['gps_lat'])
                lon = float(r['gps_lon'])
                alt = float(r['gps_alt_msl'])
                qw  = float(r['px4_qw'])
                qx  = float(r['px4_qx'])
                qy  = float(r['px4_qy'])
                qz  = float(r['px4_qz'])
            except (ValueError, KeyError):
                continue
            if math.isnan(lat) or math.isnan(lon) or (lat == 0 and lon == 0):
                continue
            rows.append((lat, lon, alt, qw, qx, qy, qz))

    if len(rows) < 5:
        return None

    lats = [r[0] for r in rows]
    lons = [r[1] for r in rows]
    alts = [r[2] for r in rows]
    yaws_enu = [yaw_ned_to_enu(quat_to_yaw_ned(r[3], r[4], r[5], r[6]))
                for r in rows]

    med_lat = float(np.median(lats))
    med_lon = float(np.median(lons))
    med_alt = float(np.median(alts))

    # GPS quality: inverse of positional spread in metres
    R = 6_371_000.0
    lat_m_std = np.std(lats) * math.pi / 180.0 * R
    lon_m_std = np.std(lons) * math.pi / 180.0 * R * math.cos(math.radians(med_lat))
    pos_std_m = math.sqrt(lat_m_std**2 + lon_m_std**2) + 1e-6
    gps_quality = 1.0 / pos_std_m

    med_yaw = circular_mean(yaws_enu)
    dist_m, ang_deg = parse_filename(csv_path)

    return dict(
        lat=med_lat, lon=med_lon, alt=med_alt,
        yaw_enu=med_yaw,
        gps_quality=gps_quality,
        dist_m=dist_m, ang_deg=ang_deg,
        n_rows=len(rows),
    )

# ── main analysis ─────────────────────────────────────────────────────────────

def main():
    print("Loading measurement files …\n")
    measurements = []
    for session in SESSIONS:
        for path in sorted(glob.glob(os.path.join(session, '*.csv'))):
            m = load_measurement(path)
            if m is None:
                continue
            measurements.append(m)
            print(f"  {os.path.basename(path):38s}  "
                  f"d={m['dist_m']:4.1f}m  a={m['ang_deg']:6.1f}°  "
                  f"gps_qual={m['gps_quality']:8.1f}  n={m['n_rows']}")

    if not measurements:
        sys.exit("No measurements loaded — check SESSIONS paths.")

    print(f"\n{len(measurements)} files loaded.\n")

    # ── Datum: quality-weighted centroid of all robot GPS positions ──────────
    total_q = sum(m['gps_quality'] for m in measurements)
    datum_lat = sum(m['lat'] * m['gps_quality'] for m in measurements) / total_q
    datum_lon = sum(m['lon'] * m['gps_quality'] for m in measurements) / total_q
    datum_alt = sum(m['alt'] * m['gps_quality'] for m in measurements) / total_q
    print(f"Datum (weighted centroid of robot positions):")
    print(f"  lat={datum_lat:.8f}  lon={datum_lon:.8f}  alt={datum_alt:.1f} m\n")

    # ── Estimate station position from every measurement ─────────────────────
    # station_ENU = robot_ENU + D * [cos(yaw_enu), sin(yaw_enu)]
    # Weight = gps_quality² * n_rows  (squared to strongly down-weight bad GPS)
    easts, norths, weights = [], [], []
    theta_ds_list, theta_ds_weights = [], []

    for m in measurements:
        robot_east, robot_north = gps_to_enu(m['lat'], m['lon'], datum_lat, datum_lon)
        ye = m['yaw_enu']
        D  = m['dist_m']
        A  = math.radians(m['ang_deg'])
        w  = m['gps_quality']**2 * m['n_rows']

        if D == 0.0:
            # Robot IS at the station → GPS directly gives station position
            # Give this 10× extra weight since it's the most direct measurement
            easts.append(robot_east)
            norths.append(robot_north)
            weights.append(w * 10.0)
        else:
            st_east  = robot_east  + D * math.cos(ye)
            st_north = robot_north + D * math.sin(ye)
            easts.append(st_east)
            norths.append(st_north)
            weights.append(w)

        # Station orientation
        theta_ds = (ye + math.pi) - A
        theta_ds_list.append(theta_ds)
        theta_ds_weights.append(w)

    w_arr = np.array(weights)
    w_arr /= w_arr.sum()
    st_east  = float(np.dot(w_arr, easts))
    st_north = float(np.dot(w_arr, norths))

    theta_ds = circular_mean(theta_ds_list, theta_ds_weights)

    # ── Convert back to GPS ──────────────────────────────────────────────────
    station_lat, station_lon = enu_to_gps(st_east, st_north, datum_lat, datum_lon)
    station_alt = datum_alt  # assume same altitude as measurement area

    # ENU heading → compass (degrees CW from North)
    heading_compass = (90.0 - math.degrees(theta_ds)) % 360.0
    heading_enu_deg = math.degrees(theta_ds) % 360.0

    # ── Residual report ──────────────────────────────────────────────────────
    print("Residuals (distance between predicted and weighted-mean station):\n")
    residuals = []
    for m, e, n in zip(measurements, easts, norths):
        res = math.sqrt((e - st_east)**2 + (n - st_north)**2)
        residuals.append(res)

    sorted_items = sorted(zip(residuals, measurements), key=lambda x: -x[0])
    for res, m in sorted_items:
        flag = " ← large" if res > 0.5 else ""
        print(f"  d={m['dist_m']:4.1f}m  a={m['ang_deg']:6.1f}°  "
              f"residual={res:.3f}m  gps_qual={m['gps_quality']:.1f}{flag}")

    mean_res = float(np.mean(residuals))
    rms_res  = float(np.sqrt(np.mean(np.array(residuals)**2)))
    print(f"\n  Mean residual : {mean_res:.3f} m")
    print(f"  RMS  residual : {rms_res:.3f} m")

    print(f"\n{'='*60}")
    print(f"  DOCKING STATION RESULT")
    print(f"{'='*60}")
    print(f"  GPS:          lat={station_lat:.8f}  lon={station_lon:.8f}")
    print(f"  Altitude:     {station_alt:.1f} m (MSL, from measurement area median)")
    print(f"  ENU from datum: east={st_east:.2f} m  north={st_north:.2f} m")
    print(f"  Orientation:  {heading_enu_deg:.1f}° ENU  ({heading_compass:.1f}° compass)")
    print(f"  Station 0° axis faces: {_compass_label(heading_compass)}")
    print(f"{'='*60}\n")

    # ── Write yaml ───────────────────────────────────────────────────────────
    out_dir = os.path.join(os.path.dirname(__file__), '..', 'config')
    out_path = os.path.realpath(os.path.join(out_dir, 'dock_station.yaml'))
    with open(out_path, 'w') as f:
        f.write(f"""\
# Docking station pose — auto-generated by compute_dock_pose.py
# Source: {len(measurements)} sweep measurements from 2 sessions (2026-05-21)
# RMS position residual: {rms_res:.3f} m

dock_station:
  # GPS position of the docking station
  lat: {station_lat:.8f}
  lon: {station_lon:.8f}
  alt: {station_alt:.2f}          # metres MSL

  # Direction the station's "front" (0° axis) faces.
  # A robot at angle 0°, distance D will be D metres along this direction.
  # heading_compass_deg: degrees clockwise from North  (compass convention)
  # heading_enu_deg:     degrees CCW from East         (ROS/ENU convention)
  heading_compass_deg: {heading_compass:.2f}
  heading_enu_deg:     {heading_enu_deg:.2f}

  # Quality metadata
  n_measurements: {len(measurements)}
  rms_residual_m: {rms_res:.4f}
""")
    print(f"Wrote: {out_path}")


def _compass_label(deg):
    labels = ['N','NNE','NE','ENE','E','ESE','SE','SSE',
              'S','SSW','SW','WSW','W','WNW','NW','NNW']
    idx = round(deg / 22.5) % 16
    return labels[idx]


if __name__ == '__main__':
    main()
