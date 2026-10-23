#!/usr/bin/env python3
"""Acceptance checks for an Easy GUI export (point_cloud.laz + trajectory.csv).

    check_export.py <easy_gui output folder> [--step2 <folder with all_step_2.laz + trajectories.csv>]

Needs laspy with a LAZ backend, numpy and scipy (pip install "laspy[lazrs]" numpy scipy).

Checks:
  fields     LAS 1.4 / PF6, GPS time type bit, named extra bytes (range_m float32, point_origin uint8),
             point_source_id = scan index, user_data = lidar id, gps_time is Unix seconds.
  range      histogram of range_m; nothing below the near limit, nothing at the sensor origin.
  alignment  only with an export made with --check-band: nearest-neighbour distance from each check-band point
             (point_origin 3: raw point placed the near-point way) to step 1's own points. A few mm median = same
             transform chain.
  gap points nearest-neighbour distance from each gap point (point_origin 2) to step 1's points of the same and the
             neighbouring scans, next to a control: step 1's own points from the 5 ms before each gap, queried against
             step 1's points without them (a correctly placed missing slice).
  reference  with --step2: the point_origin = 0 points against step 2's full-resolution export of the same session
             (multi_view_tls_registration_step_2 <session.mjs> --export <dir>): count, XYZ, intensity, trajectory.
Exit code 1 if a check fails.
"""

import argparse
import sys
from pathlib import Path

import laspy
import numpy as np
from scipy.spatial import cKDTree

CHUNK = 5_000_000
failures = []


def check(ok, message):
    print(("  ok    " if ok else "  FAIL  ") + message)
    if not ok:
        failures.append(message)


def check_fields(laz):
    print("fields")
    with laspy.open(laz) as f:
        h = f.header
        check(str(h.version) == "1.4" and h.point_format.id == 6, f"LAS {h.version}, point format {h.point_format.id}")
        check(not h.global_encoding.gps_time_type, "global encoding bit 0 (GPS time type) = 0")
        extra = {d.name: d for d in h.point_format.extra_dimensions}
        check(set(extra) == {"range_m", "point_origin"}, f"extra bytes: {sorted(extra)}")
        check(np.dtype(extra["range_m"].dtype) == np.float32, f"range_m is {np.dtype(extra['range_m'].dtype)}")
        check(np.dtype(extra["point_origin"].dtype) == np.uint8, f"point_origin is {np.dtype(extra['point_origin'].dtype)}")
        print(f"        {h.point_count:,} points, bounds {h.mins.round(1)} .. {h.maxs.round(1)}, {h.generating_software}")


def scan(laz):
    """One streaming pass: per-flag counts, range histogram, sensor-origin and time checks, and the points needed for
    the alignment check (step 1 points up to 3 m + check-band points), all with bounded memory. 3 m, not the band's
    1.2 m: the band limits the horizontal range, steep points reach ~2 m in 3D."""
    stats = {"count": np.zeros(4, np.int64), "hist": np.zeros(1201, np.int64), "range_min": [np.inf] * 4,
             "range_max": [-np.inf] * 4, "t_min": np.inf, "t_max": -np.inf, "psid": set(), "user_data": set(),
             "origin": 0, "close_odo": [], "band": [], "big_range": []}
    with laspy.open(laz) as f:
        for p in f.chunk_iterator(CHUNK):
            flag = np.asarray(p.point_origin)
            r = np.asarray(p.range_m)
            xyz = np.column_stack([p.x, p.y, p.z])
            for k in range(4):
                m = flag == k
                if m.any():
                    stats["count"][k] += m.sum()
                    stats["range_min"][k] = min(stats["range_min"][k], float(r[m].min()))
                    stats["range_max"][k] = max(stats["range_max"][k], float(r[m].max()))
            stats["hist"] += np.bincount(np.minimum((r / 0.1).astype(np.int64), 1200), minlength=1201)
            t = np.asarray(p.gps_time)
            stats["t_min"], stats["t_max"] = min(stats["t_min"], t.min()), max(stats["t_max"], t.max())
            stats["psid"].update(np.unique(np.asarray(p.point_source_id)).tolist())
            stats["user_data"].update(np.unique(np.asarray(p.user_data)).tolist())
            stats["origin"] += int((np.abs(xyz).sum(axis=1) == 0).sum()) + int((r == 0).sum())
            m = (flag == 0) & (r < 3.0)
            stats["close_odo"].append((xyz[m], r[m]))
            stats["band"].append(xyz[flag == 3])
            far = r > 70
            if far.any():
                stats["big_range"].append(np.column_stack([r[far], xyz[far, 2], np.asarray(p.point_source_id)[far]]))
    return stats


def check_range(stats, near_min):
    print("range")
    names = ["step 1 (0)", "near point (1)", "gap point (2)", "check band (3)"]
    for k in range(4):
        if stats["count"][k]:
            print(f"        {names[k]:15s} {stats['count'][k]:>12,} points, range_m {stats['range_min'][k]:.3f} .. {stats['range_max'][k]:.3f} m")
    check(min(stats["range_min"]) >= near_min - 1e-6, f"no range_m below {near_min} m (min {min(stats['range_min']):.4f})")
    check(stats["origin"] == 0, f"no points at (0,0,0) or with range 0 ({stats['origin']})")
    h = stats["hist"]
    edges = [0, 0.4, 1.0, 1.5, 3, 10, 30, 50, 70, 80, 100, 120.1]
    print("        range_m histogram:")
    for a, b in zip(edges[:-1], edges[1:]):
        n = h[int(round(a * 10)):int(round(b * 10))].sum()
        print(f"          {a:6.1f} - {b:6.1f} m  {n:>12,}  {100 * n / h.sum():6.2f} %")
    beyond = h[700:].sum()
    print(f"        beyond 70 m: {beyond:,} points ({100 * beyond / h.sum():.4f} %)")
    if stats["big_range"]:
        # step 1's 70 m limit is horizontal, in the lidar frame: steep returns can be further away in 3D
        big = np.concatenate(stats["big_range"])
        print(f"        (step 1 limits the horizontal range only; these are in {len(np.unique(big[:, 2]))} scans, "
              f"z {big[:, 1].min():.1f} .. {big[:, 1].max():.1f} m)")


def check_alignment(stats):
    band = np.concatenate(stats["band"]) if stats["band"] else np.empty((0, 3))
    if len(band) == 0:
        return
    print("alignment (check-band points vs step 1 points)")
    band_r_max = stats["range_max"][3]
    odo = np.concatenate([o[0][o[1] <= band_r_max + 0.01] for o in stats["close_odo"]])
    d_mm = cKDTree(odo).query(band, k=1, workers=-1)[0] * 1000
    q = np.percentile(d_mm, [50, 90, 99])
    print(f"        {len(band):,} check-band points vs {len(odo):,} step 1 points up to {band_r_max:.2f} m")
    print(f"        NN distance: median {q[0]:.2f} mm, p90 {q[1]:.2f}, p99 {q[2]:.2f}, max {d_mm.max():.1f} mm")
    print(f"        exactly coincident (< 0.15 mm, same trajectory pose as step 1): {100 * (d_mm < 0.15).mean():.1f} %")
    check(q[0] < 5.0, f"median NN distance {q[0]:.2f} mm < 5 mm")


def check_gaps(laz, folder):
    """Gap points vs step 1's points, per scan (tree: step 1 points of scans s-1..s+1), next to a control: step 1's
    points from the last node interval before each gap ([node 18, node 19) of every 20-node piece), queried against the
    same tree without them."""
    print("gap points (vs step 1 points)")
    parts = []
    with laspy.open(laz) as f:
        for p in f.chunk_iterator(CHUNK):
            o = np.asarray(p.point_origin)
            m = (o == 0) | (o == 2)
            parts.append((np.column_stack([p.x[m], p.y[m], p.z[m]]).astype(np.float32), o[m], np.asarray(p.point_source_id)[m],
                          np.asarray(p.gps_time)[m], np.asarray(p.range_m)[m]))
    xyz, origin, psid, t, r = (np.concatenate([q[i] for q in parts]) for i in range(5))
    del parts
    if not (origin == 2).any():
        check(False, "gap points present")
        return
    traj = np.genfromtxt(folder / "trajectory.csv", delimiter=",", names=True, usecols=(0, 11))
    node_in_scan = np.zeros(len(traj), np.int64)
    for s in np.unique(traj["scan_index"]):
        m = np.where(traj["scan_index"] == s)[0]
        node_in_scan[m] = np.arange(len(m))
    node = np.searchsorted(traj["timestamp_unix_s"], t, side="left") - 1
    control = (origin == 0) & (node_in_scan[np.maximum(node, 0)] % 20 == 18)
    # trajectory.csv has 1 us time resolution, so points within 0.5 us of a pose may land on either side of it
    tg = t[origin == 2]
    at_or_before = np.searchsorted(traj["timestamp_unix_s"], tg + 1e-6, side="right") - 1
    before = np.searchsorted(traj["timestamp_unix_s"], tg - 1e-6, side="left") - 1
    piece_end = (node_in_scan[at_or_before] % 20 == 19) | (node_in_scan[np.maximum(before, 0)] % 20 == 19)
    exact = node_in_scan[np.maximum(node[origin == 2], 0)] % 20 == 19
    check(np.all(piece_end), f"every gap point lies after the last pose of a step 1 piece "
          f"({(~exact).sum()} within the CSV's 1 us rounding of a pose time)")
    order = np.argsort(psid, kind="stable")
    bounds = np.searchsorted(psid[order], np.arange(psid.max() + 2))
    d_gap = np.full(len(xyz), np.nan, np.float32); d_ctl = np.full(len(xyz), np.nan, np.float32)
    for s in range(psid.max() + 1):
        win = order[bounds[max(s - 1, 0)]:bounds[min(s + 2, len(bounds) - 1)]]
        own = order[bounds[s]:bounds[s + 1]]
        g = own[origin[own] == 2]; c = own[control[own]]
        base = win[(origin[win] == 0)]
        if len(g):
            d_gap[g] = cKDTree(xyz[base]).query(xyz[g], workers=-1)[0]
        if len(c):
            d_ctl[c] = cKDTree(xyz[base[~control[base]]]).query(xyz[c], workers=-1)[0]
    gm, cm = origin == 2, control

    def row(label, m_g, m_c):
        g = d_gap[m_g] * 1000; c = d_ctl[m_c] * 1000
        print(f"        {label:>8} {len(g):>10,} {np.median(g):>9.2f} {np.percentile(g, 99):>8.2f}   {len(c):>10,} {np.median(c):>9.2f} {np.percentile(c, 99):>8.2f}")

    print(f"        {'range':>8} {'gap pts':>10} {'median mm':>9} {'p99 mm':>8}   {'control':>10} {'median mm':>9} {'p99 mm':>8}")
    for lo, hi in ((1, 3), (3, 5), (5, 10), (10, 20), (20, 100)):
        row(f"{lo}-{hi} m", gm & (r >= lo) & (r < hi), cm & (r >= lo) & (r < hi))
    row("all", gm, cm)
    med_g, med_c = np.median(d_gap[gm]), np.median(d_ctl[cm])
    check(med_g <= 1.2 * med_c + 0.0005, f"gap points as close to step 1's points as a missing slice of step 1's own "
          f"(median {med_g * 1000:.2f} vs {med_c * 1000:.2f} mm)")


def check_reference(laz, step2_dir):
    print("reference (point_origin = 0 vs step 2 export)")
    ref_file = step2_dir / "all_step_2.laz"
    n_ref = laspy.open(ref_file).header.point_count
    with laspy.open(laz) as f, laspy.open(ref_file) as g:
        it_ref = g.chunk_iterator(CHUNK)
        pending = None
        n = 0
        max_d = np.zeros(3)
        intensity_diff = 0
        psid_diff = 0
        gps_scale = None
        for p in f.chunk_iterator(CHUNK):
            m = np.asarray(p.point_origin) == 0
            if not m.any():
                continue
            ours = np.column_stack([p.x[m], p.y[m], p.z[m]])
            ours_i = np.asarray(p.intensity)[m]
            ours_t = np.asarray(p.gps_time)[m]
            ours_s = np.asarray(p.point_source_id)[m]
            k = 0
            while k < len(ours):
                if pending is None or len(pending[0]) == 0:
                    q = next(it_ref)
                    pending = (np.column_stack([q.x, q.y, q.z]), np.asarray(q.intensity), np.asarray(q.gps_time),
                               np.asarray(q.point_source_id))
                take = min(len(ours) - k, len(pending[0]))
                max_d = np.maximum(max_d, np.abs(ours[k:k + take] - pending[0][:take]).max(axis=0))
                intensity_diff += int((ours_i[k:k + take] != pending[1][:take]).sum())
                psid_diff += int((ours_s[k:k + take] != pending[3][:take]).sum())
                if gps_scale is None:
                    gps_scale = pending[2][0] / ours_t[k]
                pending = tuple(a[take:] for a in pending)
                k += take
            n += len(ours)
    check(n == n_ref, f"point count {n:,} (step 2: {n_ref:,})")
    check(max_d.max() <= 0.0002, f"max |dXYZ| {max_d * 1000} mm <= 0.2 mm")
    check(intensity_diff == 0, f"intensity identical ({intensity_diff} differ)")
    check(psid_diff == 0, f"point_source_id identical ({psid_diff} differ)")
    print(f"        step 2 gps_time / ours = {gps_scale:.6g} (step 2 writes the session's ns times x 1e9)")


def check_trajectory(folder, step2_dir, stats):
    print("trajectory")
    traj = np.genfromtxt(folder / "trajectory.csv", delimiter=",", names=True)
    print(f"        {len(traj):,} poses, {traj['timestamp_unix_s'][0]:.6f} .. {traj['timestamp_unix_s'][-1]:.6f} s")
    check(np.all(np.diff(traj["timestamp_unix_s"]) > 0), "timestamps strictly increasing")
    check(traj["timestamp_unix_s"][0] <= stats["t_min"] + 0.02 and traj["timestamp_unix_s"][-1] >= stats["t_max"] - 0.02,
          "trajectory spans the point times")
    q = np.column_stack([traj["qw"], traj["qx"], traj["qy"], traj["qz"]])
    check(np.abs(np.linalg.norm(q, axis=1) - 1).max() < 1e-6, "unit quaternions")
    check(set(np.unique(traj["scan_index"]).astype(int).tolist()) == stats["psid"], "scan_index matches point_source_id")
    if step2_dir:
        ref = np.loadtxt(step2_dir / "trajectories.csv", delimiter=",")
        check(len(ref) == len(traj), f"pose count {len(traj):,} (step 2: {len(ref):,})")
        dt = np.abs(ref[:, 0] / 1e9 - traj["timestamp_unix_s"]).max()
        dxyz = np.abs(ref[:, 2:5] - np.column_stack([traj["x"], traj["y"], traj["z"]])).max()
        # step 2: qx qy qz qw; q and -q are the same rotation
        rq = ref[:, [8, 5, 6, 7]]
        dq = np.minimum(np.abs(rq - q).max(axis=1), np.abs(rq + q).max(axis=1)).max()
        check(dt < 1e-6, f"timestamps match step 2 (max diff {dt * 1e6:.3f} us)")
        check(dxyz <= 0.00005, f"positions match step 2 (max diff {dxyz * 1000:.3f} mm)")
        check(dq < 1e-8, f"orientations match step 2 (max quaternion diff {dq:.2e})")
        host = np.abs(ref[:, 1] / 1e3 - traj["host_clock_unix_s"]).max()
        check(host < 1e-3, f"host clock matches step 2's Unix column / 1000 (max diff {host:.4f} s)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder", type=Path)
    ap.add_argument("--step2", type=Path)
    ap.add_argument("--near-min", type=float, default=0.4)
    a = ap.parse_args()
    laz = a.folder / "point_cloud.laz"
    check_fields(laz)
    stats = scan(laz)
    check(stats["user_data"] and len(stats["user_data"]) == 1, f"user_data (lidar id) values {sorted(stats['user_data'])}")
    print(f"        point_source_id {min(stats['psid'])} .. {max(stats['psid'])} ({len(stats['psid'])} scans)")
    check(1.5e9 < stats["t_min"] < stats["t_max"] < 2.2e9, f"gps_time in Unix seconds ({stats['t_min']:.3f} .. {stats['t_max']:.3f})")
    check_range(stats, a.near_min)
    check_alignment(stats)
    check_gaps(laz, a.folder)
    if a.step2:
        check_reference(laz, a.step2)
    check_trajectory(a.folder, a.step2, stats)
    check((a.folder / "README.txt").is_file(), "README.txt present")
    print(f"\n{'ALL CHECKS PASSED' if not failures else f'{len(failures)} CHECK(S) FAILED'}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
