#!/usr/bin/env python3

import argparse
import os
from pathlib import Path

import cv2
import numpy as np
import yaml

UNKNOWN = 205
FREE = 254
OCCUPIED = 0


def load_map_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def resolve_image_path(map_yaml_path: Path, image_field: str) -> Path:
    image_path = Path(image_field)
    if not image_path.is_absolute():
        image_path = map_yaml_path.parent / image_path
    return image_path


def binarize_map(img: np.ndarray, free_thresh: float, occupied_thresh: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if img.ndim != 2:
        raise ValueError("Map image must be grayscale")

    free_val = int(round(free_thresh * 255))
    occ_val = int(round(occupied_thresh * 255))

    occupied_mask = img <= occ_val
    free_mask = img >= free_val
    unknown_mask = ~(occupied_mask | free_mask)

    return occupied_mask, free_mask, unknown_mask


def clean_map(
    occupied_mask: np.ndarray,
    free_mask: np.ndarray,
    close_iter: int,
    open_iter: int,
    kernel: int,
    close_kernel: int,
    bridge_kernel: int,
) -> tuple[np.ndarray, np.ndarray]:
    k_free = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel, kernel))
    k_occ = cv2.getStructuringElement(cv2.MORPH_RECT, (close_kernel, close_kernel))

    occ = occupied_mask.astype(np.uint8) * 255
    free = free_mask.astype(np.uint8) * 255

    if close_iter > 0:
        occ = cv2.morphologyEx(occ, cv2.MORPH_CLOSE, k_occ, iterations=close_iter)
    if bridge_kernel > 0:
        k_bridge = cv2.getStructuringElement(cv2.MORPH_RECT, (bridge_kernel, bridge_kernel))
        occ = cv2.morphologyEx(occ, cv2.MORPH_CLOSE, k_bridge, iterations=1)
    if open_iter > 0:
        free = cv2.morphologyEx(free, cv2.MORPH_OPEN, k_free, iterations=open_iter)

    return occ > 0, free > 0


def reconstruct_map(occupied_mask: np.ndarray, free_mask: np.ndarray, unknown_mask: np.ndarray) -> np.ndarray:
    out = np.full(unknown_mask.shape, UNKNOWN, dtype=np.uint8)
    out[free_mask] = FREE
    out[occupied_mask] = OCCUPIED
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description="Clean a ROS2 occupancy grid map (PGM + YAML).")
    parser.add_argument("--input-yaml", required=True, help="Path to input map.yaml")
    parser.add_argument("--output-yaml", required=True, help="Path to output map.yaml")
    parser.add_argument("--kernel", type=int, default=3, help="Kernel size for free-space morphology")
    parser.add_argument("--close-kernel", type=int, default=5, help="Kernel size for occupied close")
    parser.add_argument("--bridge-kernel", type=int, default=0, help="Extra close kernel to bridge openings (0=off)")
    parser.add_argument("--close-iter", type=int, default=2, help="Close iterations on occupied mask")
    parser.add_argument("--open-iter", type=int, default=1, help="Open iterations on free mask")

    args = parser.parse_args()

    input_yaml = Path(args.input_yaml)
    output_yaml = Path(args.output_yaml)

    data = load_map_yaml(input_yaml)
    image_path = resolve_image_path(input_yaml, data["image"])

    img = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise RuntimeError(f"Failed to read map image: {image_path}")

    occupied_mask, free_mask, unknown_mask = binarize_map(
        img,
        free_thresh=float(data.get("free_thresh", 0.25)),
        occupied_thresh=float(data.get("occupied_thresh", 0.65)),
    )

    occupied_mask, free_mask = clean_map(
        occupied_mask,
        free_mask,
        close_iter=max(0, args.close_iter),
        open_iter=max(0, args.open_iter),
        kernel=max(1, args.kernel),
        close_kernel=max(1, args.close_kernel),
        bridge_kernel=max(0, args.bridge_kernel),
    )

    cleaned = reconstruct_map(occupied_mask, free_mask, unknown_mask)

    output_yaml.parent.mkdir(parents=True, exist_ok=True)
    output_image = output_yaml.with_suffix(".pgm")
    cv2.imwrite(str(output_image), cleaned)

    data["image"] = os.path.basename(output_image)
    with output_yaml.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, sort_keys=False)

    print(f"Wrote cleaned map: {output_image}")
    print(f"Wrote cleaned yaml: {output_yaml}")


if __name__ == "__main__":
    main()
