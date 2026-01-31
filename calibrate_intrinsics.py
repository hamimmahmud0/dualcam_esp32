#!/usr/bin/env python
"""Calibrate camera intrinsics from a checkerboard image set.

Example:
  python calibrate_intrinsics.py \
    --images /home/hamim-mahmud/Workspace/esp_dual_cam/master_calib/capture_1 \
    --cols 18 --rows 8 --square 1.0
"""

import argparse
import glob
import json
import os
from typing import List, Tuple

import cv2
import numpy as np


def find_images(images_dir: str) -> List[str]:
    exts = ("*.png", "*.jpg", "*.jpeg", "*.bmp", "*.tif", "*.tiff")
    paths = []
    for ext in exts:
        paths.extend(glob.glob(os.path.join(images_dir, ext)))
    return sorted(paths)


def calibrate(
    images: List[str],
    pattern_size: Tuple[int, int],
    square_size: float,
    use_sb: bool,
) -> Tuple[np.ndarray, np.ndarray, float, int, Tuple[int, int]]:
    if not images:
        raise ValueError("No images found.")

    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0 : pattern_size[0], 0 : pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size

    objpoints = []
    imgpoints = []
    image_size = None

    if use_sb:
        flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    else:
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    for path in images:
        img = cv2.imread(path)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        image_size = gray.shape[::-1]

        if use_sb:
            found, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags)
        else:
            found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)

        if not found:
            continue

        if corners is None or len(corners) == 0:
            continue

        if not use_sb:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        else:
            # SB already returns subpixel corners, but refine slightly for consistency.
            corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)

        objpoints.append(objp)
        imgpoints.append(corners)

    if not objpoints:
        raise RuntimeError("No checkerboard detections. Check --cols/--rows and image quality.")

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints,
        imgpoints,
        image_size,
        None,
        None,
    )

    # Compute mean reprojection error
    total_error = 0.0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        total_error += error
    mean_error = total_error / len(objpoints)

    return camera_matrix, dist_coeffs, mean_error, len(objpoints), image_size


def main() -> None:
    parser = argparse.ArgumentParser(description="Calibrate camera intrinsics from checkerboard images.")
    parser.add_argument("--images", required=True, help="Directory containing calibration images")
    parser.add_argument("--cols", type=int, required=True, help="Number of inner corners per row")
    parser.add_argument("--rows", type=int, required=True, help="Number of inner corners per column")
    parser.add_argument(
        "--square",
        type=float,
        default=1.0,
        help="Square size (any unit). Affects extrinsics scale only; intrinsics in pixels.",
    )
    parser.add_argument(
        "--no-sb",
        action="store_true",
        help="Use classic findChessboardCorners instead of the SB detector",
    )
    parser.add_argument(
        "--out",
        default="camera_intrinsics.json",
        help="Output path (.json, .yaml, or .yml)",
    )

    args = parser.parse_args()

    images = find_images(args.images)
    camera_matrix, dist_coeffs, mean_error, used, image_size = calibrate(
        images,
        (args.cols, args.rows),
        args.square,
        use_sb=not args.no_sb,
    )

    result = {
        "image_size": {"width": int(image_size[0]), "height": int(image_size[1])},
        "pattern_size": {"cols": args.cols, "rows": args.rows},
        "square_size": args.square,
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist_coeffs.tolist(),
        "mean_reprojection_error": float(mean_error),
        "used_images": used,
        "total_images": len(images),
    }

    def _dump_yaml(data, indent=0):
        lines = []
        pad = "  " * indent
        if isinstance(data, dict):
            for key, value in data.items():
                if isinstance(value, (dict, list)):
                    lines.append(f"{pad}{key}:")
                    lines.extend(_dump_yaml(value, indent + 1))
                else:
                    lines.append(f"{pad}{key}: {value}")
        elif isinstance(data, list):
            for item in data:
                if isinstance(item, (dict, list)):
                    lines.append(f\"{pad}-\")
                    lines.extend(_dump_yaml(item, indent + 1))
                else:
                    lines.append(f\"{pad}- {item}\")
        else:
            lines.append(f\"{pad}{data}\")
        return lines

    out_lower = args.out.lower()
    if out_lower.endswith((".yaml", ".yml")):
        try:
            import yaml  # type: ignore
        except Exception:
            yaml = None

        with open(args.out, "w", encoding="utf-8") as f:
            if yaml is not None:
                yaml.safe_dump(result, f, sort_keys=False)
            else:
                f.write("\\n".join(_dump_yaml(result)) + "\\n")
    else:
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2)

    print("Calibration done")
    print(f"Used {used}/{len(images)} images")
    print("Camera matrix:\n", camera_matrix)
    print("Dist coeffs:\n", dist_coeffs.ravel())
    print("Mean reprojection error:", mean_error)
    print("Saved:", args.out)


if __name__ == "__main__":
    main()
