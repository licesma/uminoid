import argparse
import sys
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # add py/ to sys.path

from processor.processor import Postprocessor
from processor.components.mode import Mode


def main() -> int:
    parser = argparse.ArgumentParser(description="Postprocess one or more recorded sessions into a single LeRobot dataset.")
    parser.add_argument(
        "episode_names",
        nargs="+",
        help="One or more session folder names under data/, e.g. may_25_17:59 may_25_18:28",
    )
    parser.add_argument(
        "--name",
        type=str,
        required=True,
        help="Name of the combined LeRobot dataset directory under training_data/.",
    )
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
        help="Also save data.csv + frames copy under final_data/<episode>/ for each session (off by default).",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Print full traceback on error instead of just the message.",
    )
    args = parser.parse_args()

    try:
        Postprocessor(
            episode_names=args.episode_names,
            name=args.name,
            mode=args.mode,
            instruction=args.instruction,
            save_final=args.save_final,
        ).run()
    except Exception as exc:
        if args.debug:
            traceback.print_exc()
        else:
            print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
