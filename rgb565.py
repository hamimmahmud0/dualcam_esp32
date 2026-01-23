#!/usr/bin/env python3
import argparse
import os
import subprocess
import sys

def rgb565_to_rgb888(value):
    r = (value >> 11) & 0x1F
    g = (value >> 5) & 0x3F
    b = value & 0x1F
    r = (r * 255) // 31
    g = (g * 255) // 63
    b = (b * 255) // 31
    return r, g, b

def read_rgb565(path):
    with open(path, "rb") as f:
        return f.read()

def write_ppm(path, width, height, rgb_bytes):
    with open(path, "wb") as f:
        header = f"P6\n{width} {height}\n255\n".encode("ascii")
        f.write(header)
        f.write(rgb_bytes)

def write_png(path, width, height, rgb_bytes):
    try:
        from PIL import Image
    except Exception:
        return False
    img = Image.frombytes("RGB", (width, height), rgb_bytes)
    img.save(path)
    return True

def open_viewer(path):
    if sys.platform.startswith("linux"):
        subprocess.Popen(["xdg-open", path])
    elif sys.platform == "darwin":
        subprocess.Popen(["open", path])
    elif os.name == "nt":
        os.startfile(path)


def main():
    parser = argparse.ArgumentParser(description="View/convert RGB565 raw frames")
    parser.add_argument("file", help="RGB565 raw file path")
    parser.add_argument("--width", type=int, required=True, help="Image width in pixels")
    parser.add_argument("--height", type=int, required=True, help="Image height in pixels")
    parser.add_argument("--stride", type=int, default=0, help="Bytes per row (default width*2)")
    parser.add_argument("--endian", choices=["little", "big"], default="big")
    parser.add_argument("--out", default="", help="Output image path (.ppm or .png)")
    parser.add_argument("--show", action="store_true", help="Open output file with system viewer")
    parser.add_argument("--plot", action="store_true", help="Plot image using numpy/matplotlib")
    args = parser.parse_args()

    data = read_rgb565(args.file)
    row_bytes = args.stride if args.stride else args.width * 2
    expected = row_bytes * args.height
    if len(data) < expected:
        print(f"Input too small: {len(data)} bytes, expected at least {expected}")
        return 2

    rgb = bytearray(args.width * args.height * 3)
    idx = 0
    offset = 0
    for _ in range(args.height):
        row = data[offset:offset + row_bytes]
        offset += row_bytes
        for x in range(args.width):
            b0 = row[x * 2]
            b1 = row[x * 2 + 1]
            value = b0 | (b1 << 8) if args.endian == "little" else (b1 | (b0 << 8))
            r, g, b = rgb565_to_rgb888(value)
            rgb[idx] = r
            rgb[idx + 1] = g
            rgb[idx + 2] = b
            idx += 3

    if args.plot:
        try:
            import numpy as np
            import matplotlib.pyplot as plt
        except Exception as exc:
            print(f"Plotting requires numpy and matplotlib: {exc}")
        else:
            arr = np.frombuffer(rgb, dtype=np.uint8).reshape((args.height, args.width, 3))
            plt.imshow(arr)
            plt.title(os.path.basename(args.file))
            plt.axis("off")
            plt.show()

    out_path = args.out
    if not out_path:
        out_path = os.path.splitext(args.file)[0] + ".png"

    if out_path.lower().endswith(".png"):
        if not write_png(out_path, args.width, args.height, bytes(rgb)):
            out_path = os.path.splitext(out_path)[0] + ".ppm"
            write_ppm(out_path, args.width, args.height, bytes(rgb))
    else:
        write_ppm(out_path, args.width, args.height, bytes(rgb))

    print(f"Wrote {out_path}")
    if args.show:
        open_viewer(out_path)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
