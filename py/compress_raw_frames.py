#!/usr/bin/env python3
from __future__ import annotations

import argparse
import struct
import sys
import zlib
from pathlib import Path


DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
CHANNELS = 3  # RGB8


def png_chunk(chunk_type: bytes, data: bytes) -> bytes:
    return (
        struct.pack(">I", len(data))
        + chunk_type
        + data
        + struct.pack(">I", zlib.crc32(chunk_type + data) & 0xFFFFFFFF)
    )


def raw_rgb_to_png_bytes(raw: bytes, width: int, height: int) -> bytes:
    expected_size = width * height * CHANNELS
    if len(raw) != expected_size:
        raise ValueError(
            f"Expected {expected_size} bytes for a {height}x{width} RGB frame, got {len(raw)}"
        )

    row_stride = width * CHANNELS
    filtered = bytearray()
    for row_start in range(0, len(raw), row_stride):
        filtered.append(0)  # PNG filter type 0: no filtering.
        filtered.extend(raw[row_start : row_start + row_stride])

    signature = b"\x89PNG\r\n\x1a\n"
    ihdr = png_chunk(
        b"IHDR",
        struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0),
    )
    idat = png_chunk(b"IDAT", zlib.compress(bytes(filtered), level=9))
    iend = png_chunk(b"IEND", b"")
    return signature + ihdr + idat + iend


def convert_file(raw_path: Path, output_dir: Path, width: int, height: int, overwrite: bool) -> Path:
    png_path = output_dir / f"{raw_path.stem}.png"
    if png_path.exists() and not overwrite:
        raise FileExistsError(f"{png_path} already exists (use --overwrite to replace it)")

    raw = raw_path.read_bytes()
    png_path.write_bytes(raw_rgb_to_png_bytes(raw, width, height))
    return png_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert recorded RGB8 .raw camera frames to PNG."
    )
    parser.add_argument(
        "frames_dir",
        type=Path,
        help="Directory containing .raw frame files.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Directory to write PNG files into. Defaults to a sibling png_frames directory.",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=DEFAULT_WIDTH,
        help=f"Frame width in pixels (default: {DEFAULT_WIDTH}).",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=DEFAULT_HEIGHT,
        help=f"Frame height in pixels (default: {DEFAULT_HEIGHT}).",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite existing PNG files.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    frames_dir = args.frames_dir.resolve()
    output_dir = (args.output_dir or frames_dir.parent / "png_frames").resolve()

    if not frames_dir.is_dir():
        print(f"error: {frames_dir} is not a directory", file=sys.stderr)
        return 1

    raw_files = sorted(frames_dir.glob("*.raw"))
    if not raw_files:
        print(f"error: no .raw files found in {frames_dir}", file=sys.stderr)
        return 1

    output_dir.mkdir(parents=True, exist_ok=True)

    converted = 0
    for raw_path in raw_files:
        try:
            png_path = convert_file(raw_path, output_dir, args.width, args.height, args.overwrite)
            converted += 1
            print(f"{raw_path.name} -> {png_path.name}")
        except Exception as exc:
            print(f"error: failed to convert {raw_path.name}: {exc}", file=sys.stderr)
            return 1

    print(f"Converted {converted} frame(s) to {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
