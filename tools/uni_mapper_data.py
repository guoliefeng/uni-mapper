#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Convert:
    pose_graph.g2o
    pcd_buffer/<id>.pcd

to OpenLMM / Uni-Mapper format:

    Scans/
        000000.pcd
        000001.pcd
        ...
    optimized_poses.txt

Usage:

Single session:
    python3 uni_mapper_data.py \
        -i /path/to/session1 \
        -o /path/to/openlmm/session1

Multiple sessions:
    python3 uni_mapper_data.py \
        -i /path/to/test_qc \
        -o /path/to/openlmm_dataset

Input can be:

    test_qc/
        session1/
            pose_graph.g2o
            pcd_buffer/
        session2/
            pose_graph.g2o
            pcd_buffer/

The script automatically detects batch mode.
"""

import argparse
import csv
import json
import math
import os
import re
import shutil
import sys
from pathlib import Path


# ============================================================
# Utilities
# ============================================================

def log(msg):
    print(msg, flush=True)


def warn(msg):
    print(f"[WARN] {msg}", file=sys.stderr, flush=True)


def die(msg):
    print(f"[ERROR] {msg}", file=sys.stderr, flush=True)
    sys.exit(1)


# ============================================================
# G2O
# ============================================================

def parse_g2o(g2o_path):
    """
    Parse:

    VERTEX_SE3:QUAT id x y z qx qy qz qw

    Return:

    {
        id: (x, y, z, qx, qy, qz, qw)
    }
    """

    poses = {}

    with open(g2o_path, "r") as f:

        for line_num, line in enumerate(f, 1):

            line = line.strip()

            if not line:
                continue

            if line.startswith("#"):
                continue

            data = line.split()

            if data[0] not in (
                "VERTEX_SE3:QUAT",
                "VERTEX_SE3:QUATERNION",
            ):
                continue

            if len(data) < 9:
                die(
                    f"Invalid g2o vertex at "
                    f"{g2o_path}:{line_num}"
                )

            try:

                vertex_id = int(data[1])

                x = float(data[2])
                y = float(data[3])
                z = float(data[4])

                qx = float(data[5])
                qy = float(data[6])
                qz = float(data[7])
                qw = float(data[8])

            except ValueError:
                die(
                    f"Invalid numeric value at "
                    f"{g2o_path}:{line_num}"
                )

            # normalize quaternion
            norm = math.sqrt(
                qx * qx +
                qy * qy +
                qz * qz +
                qw * qw
            )

            if norm < 1e-12:
                die(
                    f"Invalid quaternion at "
                    f"vertex {vertex_id}"
                )

            qx /= norm
            qy /= norm
            qz /= norm
            qw /= norm

            poses[vertex_id] = (
                x, y, z,
                qx, qy, qz, qw
            )

    if not poses:
        die(
            f"No VERTEX_SE3:QUAT found in "
            f"{g2o_path}"
        )

    return poses


# ============================================================
# Quaternion -> KITTI
# ============================================================

def pose_to_kitti(pose):

    x, y, z, qx, qy, qz, qw = pose

    xx = qx * qx
    yy = qy * qy
    zz = qz * qz

    xy = qx * qy
    xz = qx * qz
    yz = qy * qz

    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    r00 = 1.0 - 2.0 * (yy + zz)
    r01 = 2.0 * (xy - wz)
    r02 = 2.0 * (xz + wy)

    r10 = 2.0 * (xy + wz)
    r11 = 1.0 - 2.0 * (xx + zz)
    r12 = 2.0 * (yz - wx)

    r20 = 2.0 * (xz - wy)
    r21 = 2.0 * (yz + wx)
    r22 = 1.0 - 2.0 * (xx + yy)

    return (
        r00, r01, r02, x,
        r10, r11, r12, y,
        r20, r21, r22, z,
    )


# ============================================================
# PCD
# ============================================================

def get_pcd_id(path):
    """
    Support:

        0.pcd
        000001.pcd
        keyframe_123.pcd

    Last integer in filename is used.
    """

    stem = path.stem

    if stem.isdigit():
        return int(stem)

    result = re.findall(r"\d+", stem)

    if not result:
        return None

    return int(result[-1])


def load_pcd_files(pcd_dir):

    result = {}

    for path in pcd_dir.iterdir():

        if not path.is_file():
            continue

        if path.suffix.lower() != ".pcd":
            continue

        frame_id = get_pcd_id(path)

        if frame_id is None:
            warn(
                f"Cannot extract frame id from "
                f"{path.name}, skipped"
            )
            continue

        if frame_id in result:
            die(
                f"Duplicate PCD frame id "
                f"{frame_id}"
            )

        result[frame_id] = path

    if not result:
        die(
            f"No valid PCD files found in "
            f"{pcd_dir}"
        )

    return result


# ============================================================
# File transfer
# ============================================================

def link_or_copy(src, dst, mode):

    if dst.exists() or dst.is_symlink():
        dst.unlink()

    if mode == "copy":

        shutil.copy2(src, dst)
        return "copy"

    if mode == "symlink":

        os.symlink(
            str(src.resolve()),
            str(dst)
        )

        return "symlink"

    if mode == "hardlink":

        os.link(src, dst)
        return "hardlink"

    # auto:
    # hardlink first to avoid duplicating huge point clouds
    try:

        os.link(src, dst)

        return "hardlink"

    except OSError:

        os.symlink(
            str(src.resolve()),
            str(dst)
        )

        return "symlink"


# ============================================================
# Locate session
# ============================================================

def is_session(path):

    if not path.is_dir():
        return False

    has_g2o = (
        (path / "pose_graph.g2o").exists()
        or bool(list(path.glob("*.g2o")))
    )

    has_pcd = (
        (path / "pcd_buffer").is_dir()
        or (path / "PCD_buffer").is_dir()
    )

    return has_g2o and has_pcd


def find_g2o(session_dir):

    default = session_dir / "pose_graph.g2o"

    if default.exists():
        return default

    files = sorted(
        session_dir.glob("*.g2o")
    )

    if len(files) == 1:
        return files[0]

    if not files:
        die(
            f"No g2o file found in "
            f"{session_dir}"
        )

    die(
        f"Multiple g2o files found in "
        f"{session_dir}"
    )


def find_pcd_dir(session_dir):

    p = session_dir / "pcd_buffer"

    if p.is_dir():
        return p

    p = session_dir / "PCD_buffer"

    if p.is_dir():
        return p

    die(
        f"pcd_buffer not found in "
        f"{session_dir}"
    )


# ============================================================
# Convert one session
# ============================================================

def convert_session(
    input_dir,
    output_dir,
    mode
):

    log("")
    log(
        f"========== {input_dir.name} =========="
    )

    g2o_path = find_g2o(input_dir)

    pcd_dir = find_pcd_dir(input_dir)

    log(f"G2O : {g2o_path}")
    log(f"PCD : {pcd_dir}")

    poses = parse_g2o(g2o_path)

    pcds = load_pcd_files(pcd_dir)

    pose_ids = set(poses.keys())
    pcd_ids = set(pcds.keys())

    matched_ids = sorted(
        pose_ids & pcd_ids
    )

    pose_only = sorted(
        pose_ids - pcd_ids
    )

    pcd_only = sorted(
        pcd_ids - pose_ids
    )

    if not matched_ids:

        die(
            "No matching IDs between "
            "g2o and pcd_buffer"
        )

    log(
        f"G2O vertices : {len(poses)}"
    )

    log(
        f"PCD files    : {len(pcds)}"
    )

    log(
        f"Matched      : {len(matched_ids)}"
    )

    if pose_only:

        warn(
            f"{len(pose_only)} g2o vertices "
            f"do not have PCD"
        )

        warn(
            f"Examples: {pose_only[:10]}"
        )

    if pcd_only:

        warn(
            f"{len(pcd_only)} PCD files "
            f"do not have g2o pose"
        )

        warn(
            f"Examples: {pcd_only[:10]}"
        )

    # --------------------------------------------------------
    # output
    # --------------------------------------------------------

    output_dir.mkdir(
        parents=True,
        exist_ok=True
    )

    scans_dir = output_dir / "Scans"

    if scans_dir.exists():

        shutil.rmtree(scans_dir)

    scans_dir.mkdir()

    pose_file = (
        output_dir /
        "optimized_poses.txt"
    )

    mapping_file = (
        output_dir /
        "frame_map.csv"
    )

    mode_counter = {
        "hardlink": 0,
        "symlink": 0,
        "copy": 0,
    }

    # --------------------------------------------------------
    # Write
    # --------------------------------------------------------

    with open(
        pose_file,
        "w"
    ) as pose_out, open(
        mapping_file,
        "w",
        newline=""
    ) as map_out:

        writer = csv.writer(map_out)

        writer.writerow([
            "output_index",
            "g2o_vertex_id",
            "source_pcd",
            "output_pcd",
        ])

        for index, vertex_id in enumerate(
            matched_ids
        ):

            # -----------------------------
            # pose
            # -----------------------------

            kitti = pose_to_kitti(
                poses[vertex_id]
            )

            pose_out.write(
                " ".join(
                    f"{x:.12g}"
                    for x in kitti
                )
                + "\n"
            )

            # -----------------------------
            # PCD
            # -----------------------------

            output_name = (
                f"{index:06d}.pcd"
            )

            output_pcd = (
                scans_dir /
                output_name
            )

            actual_mode = link_or_copy(
                pcds[vertex_id],
                output_pcd,
                mode
            )

            mode_counter[
                actual_mode
            ] += 1

            writer.writerow([
                index,
                vertex_id,
                str(
                    pcds[
                        vertex_id
                    ].resolve()
                ),
                f"Scans/{output_name}",
            ])

    # --------------------------------------------------------
    # sanity check
    # --------------------------------------------------------

    scan_count = len(
        list(
            scans_dir.glob("*.pcd")
        )
    )

    with open(pose_file) as f:

        pose_count = sum(
            1
            for line in f
            if line.strip()
        )

    if scan_count != pose_count:

        die(
            f"Internal error: "
            f"{scan_count} scans != "
            f"{pose_count} poses"
        )

    # --------------------------------------------------------
    # report
    # --------------------------------------------------------

    report = {

        "input": str(
            input_dir.resolve()
        ),

        "g2o": str(
            g2o_path.resolve()
        ),

        "pcd_buffer": str(
            pcd_dir.resolve()
        ),

        "output": str(
            output_dir.resolve()
        ),

        "g2o_vertices":
            len(poses),

        "pcd_files":
            len(pcds),

        "matched_frames":
            len(matched_ids),

        "g2o_only_ids":
            pose_only,

        "pcd_only_ids":
            pcd_only,

        "first_g2o_id":
            matched_ids[0],

        "last_g2o_id":
            matched_ids[-1],

        "pose_format":
            "kitti",

        "scan_type":
            "pcd",

        "materialization":
            mode_counter,
    }

    report_file = (
        output_dir /
        "conversion_report.json"
    )

    with open(
        report_file,
        "w"
    ) as f:

        json.dump(
            report,
            f,
            indent=2
        )

    log("")
    log(
        f"[OK] {len(matched_ids)} "
        f"frames converted"
    )

    log(
        f"Output: {output_dir}"
    )

    log(
        f"PCD mode: {mode_counter}"
    )


# ============================================================
# Main
# ============================================================

def main():

    parser = argparse.ArgumentParser(
        description=(
            "Convert "
            "pose_graph.g2o + pcd_buffer "
            "to OpenLMM dataset"
        )
    )

    parser.add_argument(
        "-i",
        "--input",
        required=True,
        help=(
            "Input session directory "
            "or directory containing "
            "multiple sessions"
        ),
    )

    parser.add_argument(
        "-o",
        "--output",
        required=True,
        help="Output directory",
    )

    parser.add_argument(
        "--mode",
        choices=[
            "auto",
            "hardlink",
            "symlink",
            "copy",
        ],
        default="auto",
        help=(
            "PCD transfer mode. "
            "Default auto: "
            "hardlink -> symlink"
        ),
    )

    args = parser.parse_args()

    input_path = Path(
        args.input
    ).expanduser().resolve()

    output_path = Path(
        args.output
    ).expanduser().resolve()

    if not input_path.is_dir():

        die(
            f"Input is not directory: "
            f"{input_path}"
        )

    # ========================================================
    # Single session
    # ========================================================

    if is_session(input_path):

        convert_session(
            input_path,
            output_path,
            args.mode,
        )

        root_dir = (
            output_path.parent
        )

        sub_dirs = [
            output_path.name
        ]

    # ========================================================
    # Multiple sessions
    # ========================================================

    else:

        sessions = []

        for path in sorted(
            input_path.iterdir()
        ):

            if is_session(path):

                sessions.append(path)

        if not sessions:

            die(
                "No session found.\n"
                "Expected:\n"
                "  input/session1/"
                "pose_graph.g2o\n"
                "  input/session1/"
                "pcd_buffer/"
            )

        log(
            f"Detected "
            f"{len(sessions)} sessions"
        )

        for session in sessions:

            convert_session(
                session,
                output_path /
                session.name,
                args.mode,
            )

        root_dir = output_path

        sub_dirs = [
            x.name
            for x in sessions
        ]

    # ========================================================
    # OpenLMM config hint
    # ========================================================

    hint = {

        "directory": {

            "root_dir_path":
                str(
                    root_dir.resolve()
                ),

            "sub_dir_list":
                sub_dirs,

            "root_save_dir":
                "/path/to/openlmm/output",
        },

        "data_loader": {

            "data_loader_type":
                "file_based",

            "pose_format":
                "kitti",

            "pose_file_name":
                "optimized_poses.txt",

            "extrinsic": [
                0, 0, 0,
                0, 0, 0, 1
            ],

            "scan_type":
                "pcd",

            "scan_dir_name":
                "Scans",

            "voxel_size":
                0.4,

            "min_range":
                5.0,

            "max_range":
                50.0,

            "delimiter":
                " ",
        },
    }

    hint_file = (
        output_path /
        "openlmm_config_hint.json"
    )

    output_path.mkdir(
        parents=True,
        exist_ok=True
    )

    with open(
        hint_file,
        "w"
    ) as f:

        json.dump(
            hint,
            f,
            indent=2
        )

    log("")
    log(
        "================================"
    )

    log(
        "Conversion finished."
    )

    log(
        f"Config hint: {hint_file}"
    )

    log("")
    log(
        "OpenLMM root_dir_path:"
    )

    log(
        str(root_dir.resolve())
    )

    log("")
    log(
        "sub_dir_list:"
    )

    log(
        str(sub_dirs)
    )


if __name__ == "__main__":
    main()