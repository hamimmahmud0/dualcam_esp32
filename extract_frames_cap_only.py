#!/usr/bin/env python3
import argparse
import os
import struct
import sys
from typing import Optional

PIXFORMAT_RGB565 = 0
HEADER_STRUCT = struct.Struct("<QIHHB3s")
HEADER_SIZE = HEADER_STRUCT.size


def rgb565_to_rgb888_py(data: bytes, width: int, height: int, endian: str) -> bytes:
    expected = width * height * 2
    if len(data) < expected:
        raise ValueError(f"Not enough data: {len(data)} bytes, expected {expected}")

    out = bytearray(width * height * 3)
    idx = 0
    for i in range(0, expected, 2):
        b0 = data[i]
        b1 = data[i + 1]
        value = b0 | (b1 << 8) if endian == "little" else (b1 | (b0 << 8))
        r = (value >> 11) & 0x1F
        g = (value >> 5) & 0x3F
        b = value & 0x1F
        out[idx] = (r * 255) // 31
        out[idx + 1] = (g * 255) // 63
        out[idx + 2] = (b * 255) // 31
        idx += 3
    return bytes(out)


def rgb565_to_rgb888_np(data: bytes, width: int, height: int, endian: str) -> Optional[bytes]:
    try:
        import numpy as np
    except Exception:
        return None

    expected = width * height * 2
    if len(data) < expected:
        raise ValueError(f"Not enough data: {len(data)} bytes, expected {expected}")

    dtype = "<u2" if endian == "little" else ">u2"
    arr = np.frombuffer(data[:expected], dtype=dtype)
    r = (arr >> 11) & 0x1F
    g = (arr >> 5) & 0x3F
    b = arr & 0x1F
    r = (r * 255) // 31
    g = (g * 255) // 63
    b = (b * 255) // 31
    rgb = np.stack([r, g, b], axis=-1).astype(np.uint8)
    return rgb.tobytes()


def write_png(path: str, width: int, height: int, rgb_bytes: bytes) -> bool:
    try:
        from PIL import Image
    except Exception:
        return False
    img = Image.frombytes("RGB", (width, height), rgb_bytes)
    img.save(path)
    return True


def write_ppm(path: str, width: int, height: int, rgb_bytes: bytes) -> None:
    with open(path, "wb") as f:
        header = f"P6\n{width} {height}\n255\n".encode("ascii")
        f.write(header)
        f.write(rgb_bytes)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Extract RGB565 frames from a .frames buffer file and write PNGs."
    )
    parser.add_argument("input", help="Path to .frames file")
    parser.add_argument("--out-dir", default="", help="Output directory (default: <input>_frames)")
    parser.add_argument("--prefix", default="frame", help="Output filename prefix")
    parser.add_argument("--max-frames", type=int, default=0, help="Stop after N frames (0 = no limit)")
    parser.add_argument("--endian", choices=["little", "big"], default="little")
    parser.add_argument("--force-size", action="store_true",
                        help="Ignore width/height vs data_len mismatch and use data_len")
    parser.add_argument("--ppm", action="store_true",
                        help="Write PPM files instead of PNG")
    args = parser.parse_args()

    input_path = args.input
    if not os.path.isfile(input_path):
        print(f"Input not found: {input_path}", file=sys.stderr)
        return 2

    out_dir = args.out_dir or (os.path.splitext(input_path)[0] + "_frames")
    os.makedirs(out_dir, exist_ok=True)

    frames_written = 0
    with open(input_path, "rb") as f:
        while True:
            header_bytes = f.read(HEADER_SIZE)
            if not header_bytes:
                break
            if len(header_bytes) < HEADER_SIZE:
                print("Incomplete header at end of file; stopping.")
                break

            timestamp_ms, data_len, width, height, fmt, _ = HEADER_STRUCT.unpack(header_bytes)
            if data_len == 0:
                print(f"Invalid frame length 0 at frame {frames_written}; stopping.")
                break

            data = f.read(data_len)
            if len(data) < data_len:
                print(f"Incomplete frame data at frame {frames_written}; stopping.")
                break

            if fmt != PIXFORMAT_RGB565:
                print(f"Skipping frame {frames_written}: unsupported format {fmt}")
                continue

            expected = width * height * 2
            if expected != data_len and not args.force_size:
                print(f"Skipping frame {frames_written}: size mismatch "
                      f"(header {data_len} vs expected {expected})")
                continue

            if expected > data_len:
                print(f"Skipping frame {frames_written}: not enough data for {width}x{height}")
                continue

            rgb = rgb565_to_rgb888_np(data, width, height, args.endian)
            if rgb is None:
                rgb = rgb565_to_rgb888_py(data, width, height, args.endian)

            name = f"{args.prefix}_{frames_written:06d}_{timestamp_ms}.png"
            out_path = os.path.join(out_dir, name)

            if args.ppm or not write_png(out_path, width, height, rgb):
                out_path = os.path.splitext(out_path)[0] + ".ppm"
                write_ppm(out_path, width, height, rgb)

            frames_written += 1
            if args.max_frames and frames_written >= args.max_frames:
                break

    print(f"Wrote {frames_written} frames to {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
