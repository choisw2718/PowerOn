import argparse
from getpass import getpass
import json
from pathlib import Path

from . import DEFAULT_DIR, differences, current_differences
from .transfer import collect_pi, DEFAULT_REMOTE


def main():
    parser = argparse.ArgumentParser(description="Collect and compare drive sessions")
    commands = parser.add_subparsers(dest="action", required=True)
    sync = commands.add_parser("sync")
    sync.add_argument("target", help="Pi user@host")
    sync.add_argument("--remote-root", default=DEFAULT_REMOTE)
    sync.add_argument("--data-dir", type=Path, default=DEFAULT_DIR)
    sync.add_argument("--ask-password", action="store_true")
    compare = commands.add_parser("compare")
    compare.add_argument("first", type=Path)
    other = compare.add_mutually_exclusive_group()
    other.add_argument("second", type=Path, nargs="?")
    other.add_argument("--current", action="store_true", help="Compare the checkout's algorithm and hardware profile")
    args = parser.parse_args()
    if args.action == "sync":
        password = getpass("Pi password: ") if args.ask_password else None
        print(f"Received {collect_pi(args.target, args.remote_root, args.data_dir, password)} session(s).")
    else:
        first = json.loads((args.first / "manifest.json").read_text(encoding="utf-8"))
        if args.second:
            second = json.loads((args.second / "manifest.json").read_text(encoding="utf-8"))
            result = differences(first, second)
        else:
            result = current_differences(first)
            print("Current checkout comparison covers algorithm sources, hardware profile, CAD and declared revision.")
        print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
