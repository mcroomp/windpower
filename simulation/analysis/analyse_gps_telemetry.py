#!/usr/bin/env python3
"""
analyse_gps_telemetry.py

Compares GPS_RAW_INT packets in mavlink.jsonl against the simulation
ground truth in telemetry.csv to detect bad or stale GPS data.

What it checks:
  1. Duplicate positions -- same lat/lon sent in consecutive packets
  2. Zero-dt packets     -- same time_usec (stale packets replayed)
  3. Velocity accuracy   -- GPS vel/cog vs simulation sens_vel_n/sens_vel_e
  4. Heading accuracy    -- GPS cog vs simulation heading
  5. Position drift      -- GPS position advance vs sim position advance

Usage:
  python analyse_gps_telemetry.py simulation/logs/<test>/
"""

import csv
import json
import math
import os
import sys

M_PER_DEG_LAT = 111_320.0


def load_telemetry(csv_path):
    rows = []
    STRING_COLS = {'phase', 'note'}
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append({
                k: (v if k in STRING_COLS else (float(v) if v != '' else float('nan')))
                for k, v in r.items()
            })
    return rows


def interp_telem(rows, t, fields):
    lo, hi = 0, len(rows) - 1
    if t <= rows[lo]['t_sim']:
        return {f: rows[lo][f] for f in fields}
    if t >= rows[hi]['t_sim']:
        return {f: rows[hi][f] for f in fields}
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if rows[mid]['t_sim'] <= t:
            lo = mid
        else:
            hi = mid
    t0, t1 = rows[lo]['t_sim'], rows[hi]['t_sim']
    a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
    return {f: rows[lo][f] * (1 - a) + rows[hi][f] * a for f in fields}


def load_gps(jsonl_path):
    out = []
    with open(jsonl_path) as f:
        for line in f:
            r = json.loads(line)
            if r.get('mavpackettype') == 'GPS_RAW_INT':
                out.append(r)
    return out


def main(log_dir):
    csv_path   = os.path.join(log_dir, 'telemetry.csv')
    jsonl_path = os.path.join(log_dir, 'mavlink.jsonl')

    print(f'Telemetry : {csv_path}')
    print(f'MAVLink   : {jsonl_path}')
    print()

    telem = load_telemetry(csv_path)
    all_gps = load_gps(jsonl_path)
    gps = [r for r in all_gps if r['lat'] != 0]

    print(f'Telemetry rows : {len(telem)}  '
          f't=[{telem[0]["t_sim"]:.1f}, {telem[-1]["t_sim"]:.1f}]s')
    print(f'GPS packets    : {len(gps)} valid (lat!=0) out of {len(all_gps)}')
    print()

    FIELDS = ['pos_x', 'pos_y', 'pos_z',
              'vel_x', 'vel_y', 'vel_z',
              'sens_vel_n', 'sens_vel_e', 'sens_vel_d',
              'rpy_yaw', 'v_horiz_ms']

    # Use first valid RTK packet as reference
    ref = gps[1]
    ref_t      = ref['time_usec'] * 1e-6
    ref_lat    = ref['lat']
    ref_lon    = ref['lon']
    m_per_deg_lon = M_PER_DEG_LAT * math.cos(math.radians(ref_lat / 1e7))
    sim_at_ref = interp_telem(telem, ref_t, FIELDS)

    print(f'Reference origin: t={ref_t:.3f}s  lat={ref_lat/1e7:.6f}  lon={ref_lon/1e7:.7f}')
    print()

    records = []
    prev = None
    for r in gps:
        t = r['time_usec'] * 1e-6
        if t <= 0:
            prev = r
            continue

        sim = interp_telem(telem, t, FIELDS)

        # GPS speed and heading from vel/cog
        gps_speed = r['vel'] / 100.0
        cog_deg   = r['cog'] / 100.0
        cog_rad   = math.radians(cog_deg)
        gps_vN    = gps_speed * math.cos(cog_rad)
        gps_vE    = gps_speed * math.sin(cog_rad)

        # Simulation values
        sim_vN    = sim['sens_vel_n']
        sim_vE    = sim['sens_vel_e']
        sim_speed = math.hypot(sim_vN, sim_vE)
        sim_cog   = (math.degrees(math.atan2(sim_vE, sim_vN)) % 360
                     if sim_speed > 0.05 else float('nan'))

        speed_err = abs(gps_speed - sim_speed)
        hdg_err   = ((cog_deg - sim_cog + 180) % 360 - 180
                     if not math.isnan(sim_cog) else float('nan'))

        # Position in NE metres relative to ref
        gps_N = (r['lat'] - ref_lat) / 1e7 * M_PER_DEG_LAT
        gps_E = (r['lon'] - ref_lon) / 1e7 * m_per_deg_lon

        # Sim position delta: pos_x=East, pos_y=North (typical ENU)
        sim_N = sim['pos_y'] - sim_at_ref['pos_y']
        sim_E = sim['pos_x'] - sim_at_ref['pos_x']

        is_dup    = (prev is not None and
                     r['lat'] == prev['lat'] and r['lon'] == prev['lon'])
        dt_usec   = ((r['time_usec'] - prev['time_usec']) / 1e6
                     if prev is not None else 0.0)
        stale     = (prev is not None and r['time_usec'] == prev['time_usec'])

        records.append(dict(
            t=t, gps_speed=gps_speed, sim_speed=sim_speed,
            speed_err=speed_err, gps_cog=cog_deg, sim_cog=sim_cog,
            hdg_err=hdg_err, gps_N=gps_N, gps_E=gps_E,
            sim_N=sim_N, sim_E=sim_E,
            pos_err=math.hypot(gps_N - sim_N, gps_E - sim_E),
            is_dup=is_dup, stale=stale, dt=dt_usec,
            lat=r['lat'], lon=r['lon'],
        ))
        prev = r

    # ------------------------------------------------------------------ #
    sep = '=' * 72

    print(sep)
    print('  DUPLICATE / STALE PACKETS')
    print(sep)
    dups   = [r for r in records if r['is_dup']]
    stales = [r for r in records if r['stale']]
    print(f'  Same lat/lon (dup pos) : {len(dups):4d} / {len(records)} '
          f'({100*len(dups)/max(1,len(records)):.1f}%)')
    print(f'  Same time_usec (stale) : {len(stales):4d} / {len(records)} '
          f'({100*len(stales)/max(1,len(records)):.1f}%)')
    print()
    if dups:
        print('  Duplicate positions (first 30):')
        print(f'  {"t_sim":>8}  {"lat":>12}  {"dt":>7}  {"gps_spd":>8}  {"sim_spd":>8}  stale')
        for r in dups[:30]:
            print(f'  {r["t"]:8.3f}s  {r["lat"]:12d}  {r["dt"]:7.3f}s  '
                  f'{r["gps_speed"]:8.3f}  {r["sim_speed"]:8.3f}  '
                  f'{"YES" if r["stale"] else "no"}')

    print()
    print(sep)
    print('  VELOCITY ACCURACY  (GPS vel/cog  vs  simulation sens_vel_n/e)')
    print(sep)
    nondups = [r for r in records if not r['is_dup'] and r['t'] > 0]
    se = sorted(r['speed_err'] for r in nondups)
    he = sorted(abs(r['hdg_err']) for r in nondups if not math.isnan(r['hdg_err']))
    if se:
        n = len(se)
        print(f'  Speed error  |GPS - sim|:  '
              f'median={se[n//2]:.4f}  p95={se[int(.95*n)]:.4f}  '
              f'max={se[-1]:.4f} m/s')
    if he:
        n = len(he)
        print(f'  Heading error |GPS - sim|: '
              f'median={he[n//2]:.2f}  p95={he[int(.95*n)]:.2f}  '
              f'max={he[-1]:.2f} deg')
    print()

    bad = [r for r in nondups if r['speed_err'] > 0.2]
    if bad:
        print(f'  Speed error > 0.2 m/s ({len(bad)} samples):')
        print(f'  {"t_sim":>8}  {"gps_spd":>8}  {"sim_spd":>8}  {"err":>6}  '
              f'{"gps_cog":>8}  {"sim_cog":>8}  {"hdg_err":>8}')
        for r in bad[:25]:
            sc = f'{r["sim_cog"]:8.1f}' if not math.isnan(r['sim_cog']) else '     nan'
            he = f'{r["hdg_err"]:8.1f}' if not math.isnan(r['hdg_err']) else '     nan'
            print(f'  {r["t"]:8.3f}s  {r["gps_speed"]:8.3f}  {r["sim_speed"]:8.3f}  '
                  f'{r["speed_err"]:6.3f}  {r["gps_cog"]:8.1f}  {sc}  {he}')
    else:
        print('  [OK] All speed errors below 0.2 m/s')

    print()
    print(sep)
    print('  POSITION DRIFT  (GPS NE vs sim NE, relative to reference t)')
    print(sep)
    print(f'  Reference point: t={ref_t:.2f}s')
    print(f'  Note: assumes sim pos_x=East, pos_y=North')
    print()
    print(f'  {"t_sim":>8}  {"gps_N":>8}  {"gps_E":>8}  {"sim_N":>8}  {"sim_E":>8}  '
          f'{"pos_err":>8}  {"dup":>5}')
    step = max(1, len(records) // 40)
    for r in records[::step]:
        flag = 'DUP' if r['is_dup'] else ''
        print(f'  {r["t"]:8.3f}s  {r["gps_N"]:8.2f}  {r["gps_E"]:8.2f}  '
              f'{r["sim_N"]:8.2f}  {r["sim_E"]:8.2f}  '
              f'{r["pos_err"]:8.2f}  {flag:>5}')

    print()
    print(sep)
    print('  GPS TIMING')
    print(sep)
    dts = [r['dt'] for r in records[1:] if r['dt'] > 0]
    if dts:
        dts_s = sorted(dts)
        n = len(dts_s)
        print(f'  dt between packets:  '
              f'median={dts_s[n//2]:.4f}s  '
              f'min={dts_s[0]:.4f}s  '
              f'max={dts_s[-1]:.4f}s  '
              f'expected=0.200s')
        gaps = [r for r in records if r['dt'] > 0.35]
        if gaps:
            print(f'  Gaps > 0.35s ({len(gaps)}):')
            for r in gaps:
                print(f'    t={r["t"]:8.3f}s  dt={r["dt"]:.3f}s')
        else:
            print('  [OK] No gaps > 0.35s')


if __name__ == '__main__':
    log_dir = sys.argv[1] if len(sys.argv) > 1 else 'simulation/logs/test_lua_flight_steady'
    main(log_dir)
