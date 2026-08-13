#!/usr/bin/env python3
"""Audit and compare Uni-Mapper dynamic-removal point clouds.

The evaluator intentionally uses only NumPy/SciPy/Matplotlib from Ubuntu.  It
supports the ASCII, binary and PCL binary_compressed PCD encodings used by the
source dataset and MapServer outputs.
"""

import argparse
import csv
import json
import math
import struct
from pathlib import Path

import numpy as np


def _read_header(stream):
    header = {}
    while True:
        line = stream.readline()
        if not line:
            raise ValueError("PCD header has no DATA line")
        text = line.decode("ascii", errors="strict").strip()
        if not text or text.startswith("#"):
            continue
        key, *values = text.split()
        header[key.upper()] = values
        if key.upper() == "DATA":
            break
    return header


def _lzf_decompress(source, expected_size):
    """Decode the LZF stream stored by PCL's binary_compressed writer."""
    target = bytearray()
    i = 0
    while i < len(source):
        control = source[i]
        i += 1
        if control < 32:
            length = control + 1
            target.extend(source[i : i + length])
            i += length
            continue
        length = control >> 5
        reference = len(target) - ((control & 0x1F) << 8) - 1
        if length == 7:
            length += source[i]
            i += 1
        reference -= source[i]
        i += 1
        length += 2
        if reference < 0:
            raise ValueError("Invalid LZF back reference")
        for _ in range(length):
            target.append(target[reference])
            reference += 1
    if len(target) != expected_size:
        raise ValueError(
            f"LZF size mismatch: decoded {len(target)}, expected {expected_size}"
        )
    return bytes(target)


def _numpy_type(type_code, size):
    kinds = {"F": "f", "I": "i", "U": "u"}
    if type_code not in kinds or size not in (1, 2, 4, 8):
        raise ValueError(f"Unsupported PCD scalar: TYPE={type_code} SIZE={size}")
    return np.dtype("<" + kinds[type_code] + str(size))


def read_pcd(path, fields=("x", "y", "z", "intensity")):
    path = Path(path)
    with path.open("rb") as stream:
        header = _read_header(stream)
        names = header["FIELDS"]
        sizes = [int(value) for value in header["SIZE"]]
        types = header["TYPE"]
        counts = [int(value) for value in header.get("COUNT", ["1"] * len(names))]
        points = int(header.get("POINTS", header["WIDTH"])[0])
        encoding = header["DATA"][0].lower()
        selected = [name for name in fields if name in names]
        if not selected:
            raise ValueError(f"No requested fields in {path}")

        if encoding == "ascii":
            values = np.loadtxt(stream, dtype=np.float64, ndmin=2)
            columns = {}
            offset = 0
            for name, count in zip(names, counts):
                if name in selected:
                    columns[name] = values[:, offset].astype(np.float32)
                offset += count
        elif encoding == "binary":
            dtype_fields = []
            for name, size, type_code, count in zip(names, sizes, types, counts):
                dtype = _numpy_type(type_code, size)
                dtype_fields.append((name, dtype) if count == 1 else (name, dtype, count))
            records = np.frombuffer(stream.read(), dtype=np.dtype(dtype_fields), count=points)
            columns = {name: np.asarray(records[name], dtype=np.float32) for name in selected}
        elif encoding == "binary_compressed":
            compressed_size, uncompressed_size = struct.unpack("<II", stream.read(8))
            unpacked = _lzf_decompress(stream.read(compressed_size), uncompressed_size)
            columns = {}
            offset = 0
            for name, size, type_code, count in zip(names, sizes, types, counts):
                byte_count = points * size * count
                if name in selected:
                    array = np.frombuffer(
                        unpacked, dtype=_numpy_type(type_code, size), count=points * count,
                        offset=offset,
                    )
                    columns[name] = array.reshape(points, count)[:, 0].astype(np.float32)
                offset += byte_count
        else:
            raise ValueError(f"Unsupported PCD DATA encoding: {encoding}")

    xyz = np.column_stack([columns[name] for name in ("x", "y", "z")])
    intensity = columns.get("intensity", np.zeros(points, dtype=np.float32))
    return xyz.astype(np.float32, copy=False), intensity, header


def pcd_point_count(path):
    with Path(path).open("rb") as stream:
        header = _read_header(stream)
    return int(header.get("POINTS", header["WIDTH"])[0])


def write_binary_pcd(path, xyz, intensity=None):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    xyz = np.asarray(xyz, dtype=np.float32)
    if intensity is None:
        intensity = np.zeros(len(xyz), dtype=np.float32)
    values = np.empty(len(xyz), dtype=[("x", "<f4"), ("y", "<f4"),
                                       ("z", "<f4"), ("intensity", "<f4")])
    values["x"], values["y"], values["z"] = xyz.T
    values["intensity"] = np.asarray(intensity, dtype=np.float32)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
        "FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
        f"COUNT 1 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\n"
        f"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(xyz)}\nDATA binary\n"
    )
    with path.open("wb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(values.tobytes())


def voxel_downsample(points, voxel_size):
    if not len(points):
        return points
    keys = np.floor(points / voxel_size).astype(np.int32)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(indices)]


def distribution(values):
    values = np.asarray(values, dtype=np.float64)
    return {
        "mean": float(np.mean(values)), "median": float(np.median(values)),
        "p90": float(np.percentile(values, 90)), "max": float(np.max(values)),
    }


def load_kitti_poses(path):
    values = np.loadtxt(path, dtype=np.float64, ndmin=2)
    if values.shape[1] != 12:
        raise ValueError(f"Expected 12-column KITTI poses in {path}")
    poses = np.tile(np.eye(4), (len(values), 1, 1))
    poses[:, :3, :4] = values.reshape(-1, 3, 4)
    return poses


def load_optimized_pose_csv(path):
    values = np.loadtxt(path, delimiter=",", dtype=np.float64, ndmin=2)
    if values.shape[1] != 8:
        raise ValueError(f"Expected index,x,y,z,qx,qy,qz,qw in {path}")
    poses = np.tile(np.eye(4), (len(values), 1, 1))
    poses[:, :3, 3] = values[:, 1:4]
    qx, qy, qz, qw = values[:, 4], values[:, 5], values[:, 6], values[:, 7]
    norm = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
    poses[:, 0, 0] = 1 - 2*(qy*qy + qz*qz)
    poses[:, 0, 1] = 2*(qx*qy - qz*qw)
    poses[:, 0, 2] = 2*(qx*qz + qy*qw)
    poses[:, 1, 0] = 2*(qx*qy + qz*qw)
    poses[:, 1, 1] = 1 - 2*(qx*qx + qz*qz)
    poses[:, 1, 2] = 2*(qy*qz - qx*qw)
    poses[:, 2, 0] = 2*(qx*qz - qy*qw)
    poses[:, 2, 1] = 2*(qy*qz + qx*qw)
    poses[:, 2, 2] = 1 - 2*(qx*qx + qy*qy)
    return values[:, 0].astype(int), poses


def build_raw_map(args):
    dataset = Path(args.dataset)
    scan_files = sorted((dataset / "Scans").glob("*.pcd"))
    indices, poses = load_optimized_pose_csv(args.poses)
    if len(scan_files) != len(poses) or not np.array_equal(indices, np.arange(len(indices))):
        raise ValueError("Optimized pose indices must be contiguous and match sorted scans")
    point_count = sum(pcd_point_count(path) for path in scan_files)
    output = Path(args.output); output.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
        "FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
        f"WIDTH {point_count}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {point_count}\nDATA binary\n"
    )
    dtype = [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")]
    with output.open("wb") as stream:
        stream.write(header.encode("ascii"))
        for path, pose in zip(scan_files, poses):
            xyz, intensity, _ = read_pcd(path)
            xyz_world = xyz @ pose[:3, :3].T + pose[:3, 3]
            records = np.empty(len(xyz), dtype=dtype)
            records["x"], records["y"], records["z"] = xyz_world.T
            records["intensity"] = intensity
            stream.write(records.tobytes())
    print(json.dumps({"output": str(output), "scans": len(scan_files),
                      "points": point_count, "poses": str(args.poses)}, indent=2))


def audit_dataset(args):
    if args.sample_scans < 1:
        raise ValueError("--sample-scans must be at least 1")
    dataset = Path(args.dataset)
    scan_files = sorted((dataset / "Scans").glob("*.pcd"))
    poses = load_kitti_poses(dataset / "optimized_poses.txt")
    if len(scan_files) != len(poses):
        raise ValueError(f"{len(scan_files)} scans != {len(poses)} poses")
    counts = np.asarray([pcd_point_count(path) for path in scan_files])
    delta_t = np.linalg.norm(np.diff(poses[:, :3, 3], axis=0), axis=1)
    relative_r = np.einsum("nij,njk->nik", poses[:-1, :3, :3].transpose(0, 2, 1),
                           poses[1:, :3, :3])
    cos_angle = np.clip((np.trace(relative_r, axis1=1, axis2=2) - 1) / 2, -1, 1)
    delta_r = np.degrees(np.arccos(cos_angle))

    sampled = []
    all_z = []
    frame_z = []
    stride = max(1, len(scan_files) // args.sample_scans)
    for index, path in enumerate(scan_files):
        xyz, _, _ = read_pcd(path)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if len(xyz):
            all_z.append(xyz[:, 2])
            frame_z.append(np.percentile(xyz[:, 2], (0, 1, 5, 50, 95, 99, 100)))
            if index % stride == 0:
                sampled.append(xyz[:: max(1, len(xyz) // 5000)])
    sample = np.concatenate(sampled) if sampled else np.empty((0, 3))
    global_z = np.concatenate(all_z)
    frame_z = np.asarray(frame_z)
    ranges = np.linalg.norm(sample, axis=1)
    elevation = np.degrees(np.arctan2(sample[:, 2], np.linalg.norm(sample[:, :2], axis=1)))
    report = {
        "dataset": str(dataset.resolve()),
        "scan_count": len(scan_files), "pose_count": len(poses),
        "first_scan": scan_files[0].name, "last_scan": scan_files[-1].name,
        "raw_points_from_headers": int(counts.sum()),
        "points_per_scan": distribution(counts),
        "translation_step_m": distribution(delta_t),
        "rotation_step_deg": distribution(delta_r),
        "time_step_sec": "UNKNOWN (no timestamps in converted KITTI poses or PCD headers)",
        "local_z_m_percentiles_exact": {str(q): float(np.percentile(global_z, q))
                                         for q in (0, 1, 5, 50, 95, 99, 100)},
        "per_frame_z_m_quantiles": {
            str(q): distribution(frame_z[:, column])
            for column, q in enumerate((0, 1, 5, 50, 95, 99, 100))
        },
        "local_range_m_percentiles": {str(q): float(np.percentile(ranges, q))
                                       for q in (0, 1, 50, 90, 99, 100)},
        "elevation_deg_percentiles": {str(q): float(np.percentile(elevation, q))
                                       for q in (0, 0.1, 1, 50, 99, 99.9, 100)},
        "frame_assessment": (
            "Scans are local/body-frame keyframes: their ranges are centered at the PCD "
            "origin, while KITTI pose translations place that origin in the world frame."
        ),
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(report, indent=2, ensure_ascii=False))


def plot_clouds(clouds, output, title, limits=None, max_points=600000):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    valid = [points[np.isfinite(points).all(axis=1)] for _, points in clouds]
    if limits is None:
        merged = np.concatenate(valid)
        limits = [float(np.percentile(merged[:, 0], 0.1)),
                  float(np.percentile(merged[:, 0], 99.9)),
                  float(np.percentile(merged[:, 1], 0.1)),
                  float(np.percentile(merged[:, 1], 99.9))]
    columns = min(3, len(clouds))
    rows = int(math.ceil(len(clouds) / columns))
    fig, axes = plt.subplots(rows, columns, figsize=(6 * columns, 6 * rows), squeeze=False)
    flat_axes = axes.ravel()
    for axis, (name, points) in zip(flat_axes, clouds):
        step = max(1, len(points) // max_points)
        shown = points[::step]
        axis.scatter(shown[:, 0], shown[:, 1], s=0.08, c=shown[:, 2], cmap="turbo",
                     rasterized=True)
        axis.set_title(f"{name}\n{len(points):,} points")
        axis.set_xlim(limits[0], limits[1]); axis.set_ylim(limits[2], limits[3])
        axis.set_aspect("equal", adjustable="box"); axis.set_xlabel("x [m]"); axis.set_ylabel("y [m]")
    for axis in flat_axes[len(clouds):]:
        axis.set_visible(False)
    fig.suptitle(title); fig.tight_layout()
    output = Path(output); output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180); plt.close(fig)
    return limits


def top_views(args):
    clouds = []
    for specification in args.cloud:
        if "=" not in specification:
            raise ValueError("--cloud must be LABEL=/path/to/cloud.pcd")
        label, path = specification.split("=", 1)
        xyz, _, _ = read_pcd(path)
        clouds.append((label, xyz[np.isfinite(xyz).all(axis=1)]))
    limits = args.limits
    if limits is None:
        reference = clouds[0][1]
        limits = [float(np.percentile(reference[:, 0], 0.1)),
                  float(np.percentile(reference[:, 0], 99.9)),
                  float(np.percentile(reference[:, 1], 0.1)),
                  float(np.percentile(reference[:, 1], 99.9))]
    plot_clouds(clouds, args.output, args.title, limits, args.max_points)
    print(json.dumps({"output": args.output, "xy_limits": limits,
                      "clouds": [label for label, _ in clouds]}, indent=2))


def compare_clouds(args):
    from scipy.spatial import cKDTree

    raw, _, _ = read_pcd(args.raw)
    static, _, _ = read_pcd(args.static)
    raw = raw[np.isfinite(raw).all(axis=1)]
    static = static[np.isfinite(static).all(axis=1)]
    raw_ds = voxel_downsample(raw, args.voxel_size)
    static_ds = voxel_downsample(static, args.voxel_size)
    distances, _ = cKDTree(static_ds).query(raw_ds, k=1, workers=-1)
    removed = raw_ds[distances > args.nn_threshold]
    write_binary_pcd(args.removed, removed, np.ones(len(removed), dtype=np.float32))
    limits = [float(np.percentile(raw_ds[:, 0], 0.1)),
              float(np.percentile(raw_ds[:, 0], 99.9)),
              float(np.percentile(raw_ds[:, 1], 0.1)),
              float(np.percentile(raw_ds[:, 1], 99.9))]
    plot_clouds(
        [("raw @ voxel", raw_ds), ("static @ voxel", static_ds),
         (f"removed approx > {args.nn_threshold:.2f} m", removed)],
        args.plot, args.title, limits,
    )
    metrics = {
        "method": args.method, "config": args.config,
        "raw_points": len(raw), "static_points": len(static),
        "removed_points": len(removed),
        "removed_ratio": len(removed) / len(raw_ds) if len(raw_ds) else 0.0,
        "runtime_sec": args.runtime_sec, "peak_memory_mb": args.peak_memory_mb,
        "voxel_size": args.voxel_size,
        "notes": (f"pre-voxel raw/static; removed is approximate NN comparison after "
                  f"{args.voxel_size:g} m voxelization, threshold={args.nn_threshold:g} m; "
                  f"post_voxel_raw={len(raw_ds)}, post_voxel_static={len(static_ds)}; {args.notes}"),
    }
    metrics_path = Path(args.metrics)
    metrics_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = ["method", "config", "raw_points", "static_points", "removed_points",
                  "removed_ratio", "runtime_sec", "peak_memory_mb", "voxel_size", "notes"]
    existing = []
    if metrics_path.exists():
        with metrics_path.open(newline="") as stream:
            existing = [row for row in csv.DictReader(stream)
                        if not (row["method"] == args.method and row["config"] == args.config)]
    with metrics_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader(); writer.writerows(existing); writer.writerow(metrics)
    summary = {**metrics, "post_voxel_raw": len(raw_ds),
               "post_voxel_static": len(static_ds), "xy_limits": limits}
    print(json.dumps(summary, indent=2))


def record_metrics(args):
    row = {
        "method": args.method, "config": args.config, "raw_points": args.raw_points,
        "static_points": args.static_points, "removed_points": args.removed_points,
        "removed_ratio": args.removed_ratio, "runtime_sec": args.runtime_sec,
        "peak_memory_mb": args.peak_memory_mb, "voxel_size": args.voxel_size,
        "notes": args.notes,
    }
    path = Path(args.metrics); path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(row)
    existing = []
    if path.exists():
        with path.open(newline="") as stream:
            existing = [item for item in csv.DictReader(stream)
                        if not (item["method"] == args.method and item["config"] == args.config)]
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(existing); writer.writerow(row)
    print(json.dumps(row, indent=2))


def roi_plots(args):
    raw, _, _ = read_pcd(args.raw)
    static, _, _ = read_pcd(args.static)
    removed, _, _ = read_pcd(args.removed)
    rois = json.loads(Path(args.rois).read_text())
    output = Path(args.output); output.mkdir(parents=True, exist_ok=True)
    for roi in rois:
        xmin, xmax, ymin, ymax = roi["bounds"]
        select = lambda p: p[(p[:, 0] >= xmin) & (p[:, 0] <= xmax) &
                             (p[:, 1] >= ymin) & (p[:, 1] <= ymax)]
        raw_roi, static_roi, removed_roi = select(raw), select(static), select(removed)
        roi_dir = output / f"roi_{roi['id']}_{roi['name']}"
        write_binary_pcd(roi_dir / "raw_roi.pcd", raw_roi)
        write_binary_pcd(roi_dir / "static_roi.pcd", static_roi)
        write_binary_pcd(roi_dir / "removed_roi.pcd", removed_roi,
                         np.ones(len(removed_roi), dtype=np.float32))
        plot_clouds([("raw", raw_roi), ("static", static_roi),
                     ("removed approx", removed_roi)],
                    output / f"roi_{roi['id']}_{roi['name']}.png",
                    f"ROI {roi['id']}: {roi['name']}", [xmin, xmax, ymin, ymax], 250000)


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    audit = sub.add_parser("audit")
    audit.add_argument("--dataset", required=True); audit.add_argument("--output", required=True)
    audit.add_argument("--sample-scans", type=int, default=120); audit.set_defaults(func=audit_dataset)
    build = sub.add_parser("build-raw")
    build.add_argument("--dataset", required=True); build.add_argument("--poses", required=True)
    build.add_argument("--output", required=True); build.set_defaults(func=build_raw_map)
    compare = sub.add_parser("compare")
    for name in ("raw", "static", "removed", "plot", "metrics", "method", "config"):
        compare.add_argument("--" + name.replace("_", "-"), required=True)
    compare.add_argument("--title", default="Dynamic-removal comparison")
    compare.add_argument("--voxel-size", type=float, default=0.2)
    compare.add_argument("--nn-threshold", type=float, default=0.2)
    compare.add_argument("--runtime-sec", type=float, default=-1)
    compare.add_argument("--peak-memory-mb", type=float, default=-1)
    compare.add_argument("--notes", default="")
    compare.set_defaults(func=compare_clouds)
    roi = sub.add_parser("rois")
    for name in ("raw", "static", "removed", "rois", "output"):
        roi.add_argument("--" + name, required=True)
    roi.set_defaults(func=roi_plots)
    top = sub.add_parser("top")
    top.add_argument("--cloud", action="append", required=True,
                     help="Repeat LABEL=/path/to/cloud.pcd")
    top.add_argument("--output", required=True); top.add_argument("--title", default="XY top view")
    top.add_argument("--limits", type=float, nargs=4, metavar=("XMIN", "XMAX", "YMIN", "YMAX"))
    top.add_argument("--max-points", type=int, default=600000); top.set_defaults(func=top_views)
    record = sub.add_parser("record")
    record.add_argument("--metrics", required=True); record.add_argument("--method", required=True)
    record.add_argument("--config", required=True)
    for name in ("raw_points", "static_points", "removed_points"):
        record.add_argument("--" + name.replace("_", "-"), type=int, required=True)
    for name in ("removed_ratio", "runtime_sec", "peak_memory_mb", "voxel_size"):
        record.add_argument("--" + name.replace("_", "-"), type=float, required=True)
    record.add_argument("--notes", default=""); record.set_defaults(func=record_metrics)
    return parser


def main():
    args = build_parser().parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
