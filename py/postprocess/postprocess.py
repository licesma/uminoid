import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # add py/ to sys.path

from processor.processor import Postprocessor
from processor.components.mode import Mode


def main() -> int:
    parser = argparse.ArgumentParser(description="Postprocess a recorded session.")
    parser.add_argument("episode_name", help="Session folder name under data/, e.g. april_19_16:31")
    parser.add_argument(
        "--mode",
        type=Mode,
        choices=list(Mode),
        default=Mode.TELEOP,
        help="Collection mode: teleop (use *_measured.csv) or mocap (use *_command.csv).",
    )
    parser.add_argument(
        "--instruction",
        type=str,
        required=True,
        help="Language instruction for the LeRobot task (e.g. 'pick up the blue marker and place it in the box').",
    )
    parser.add_argument(
        "--save-final",
        action="store_true",
        help="Also save the data.csv + frames copy under final_data/<episode>/ (off by default).",
    )
    args = parser.parse_args()

    try:
        Postprocessor(
            args.episode_name,
            args.mode,
            instruction=args.instruction,
            save_final=args.save_final,
        ).run()
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
