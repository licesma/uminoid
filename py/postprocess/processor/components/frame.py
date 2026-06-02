from pathlib import Path

from helpers.error_check import ensure
from helpers.rgb_to_png import rgb_to_png

WIDTH = 640
HEIGHT = 480
FRAMES_DIR = "frames"
OUTPUT_DIR = "png_frames"


def write_compressed(raw_path: Path, output_dir: Path) -> Path:
    out_path = output_dir / f"{raw_path.stem}.png"
    raw = raw_path.read_bytes()
    out_path.write_bytes(rgb_to_png(raw, WIDTH, HEIGHT))


def extract_frames(frames_dir: Path) -> list[Path]:
    ensure(frames_dir.is_dir(), f"{frames_dir} is not a directory")
    raw_files = sorted(frames_dir.glob("*.raw"))
    ensure(raw_files, f"no .raw files found in {frames_dir}")
    return raw_files


def compress_frames(episode_path: Path) -> None:
    frames_dir = (episode_path / FRAMES_DIR).resolve()
    output_dir = (episode_path / OUTPUT_DIR).resolve()

    raw_files = extract_frames(frames_dir)

    output_dir.mkdir(parents=True, exist_ok=True)

    for raw_path in raw_files:
        write_compressed(raw_path, output_dir)