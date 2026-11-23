"""Run the host C++ tests and all feature-level Python tests."""
from pathlib import Path
import os
import shutil
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "vehicle" / "firmware"
SKETCH_SRC = FIRMWARE / "PowerOnNano" / "src"
CPP_TESTS = FIRMWARE / "tests"
PREVIEW = ROOT / "vehicle" / "ir_wall_following" / "examples" / "ir_wall_follow_preview.csv"
PYTHON_TESTS = (
    "vehicle/manual_control/tests",
    "vehicle/drive_data/tests",
    "vehicle/sensor_monitor/tests",
    "vehicle/ir_wall_following/tests",
    "lidar/acquisition/tests",
    "lidar/autopilot/tests",
    "simulation/track/tests",
)


def find_compiler():
    compiler = shutil.which("g++")
    if compiler is None:
        candidate = Path(r"C:\Program Files\CodeBlocks\MinGW\bin\g++.exe")
        if candidate.is_file():
            return candidate
        raise SystemExit("A host g++ compiler is needed for the C++ tests.")
    return Path(compiler)


def run_cpp_tests(build_dir):
    compiler = find_compiler()
    env = dict(os.environ, PATH=str(compiler.parent) + os.pathsep + os.environ.get("PATH", ""))
    flags = ["-std=c++11", "-Wall", "-Wextra", "-Werror", "-pedantic",
             "-I", SKETCH_SRC, "-I", CPP_TESTS / "stubs"]

    for test, extra_sources, args in (
        ("ir_wall_following/wall_follower_test.cpp", (), (PREVIEW,)),
        ("ir_sensors/distance_sensors_test.cpp", (), ()),
        ("lidar_drive/drive_lease_test.cpp", (SKETCH_SRC / "lidar_drive" / "DriveLease.cpp",), ()),
        ("motor/rear_speed_test.cpp", (), ()),
    ):
        binary = build_dir / (Path(test).stem + (".exe" if os.name == "nt" else ""))
        subprocess.run([compiler, *flags, CPP_TESTS / test, *extra_sources, "-o", binary],
                       env=env, check=True)
        subprocess.run([binary, *args], env=env, check=True)


def main():
    subprocess.run([sys.executable, "-m", "hardware.generate_firmware_profile", "--check"],
                   cwd=ROOT, check=True)
    build = ROOT / "build"
    build.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(dir=build, prefix="poweron-tests-") as directory:
        run_cpp_tests(Path(directory))
    for test_directory in PYTHON_TESTS:
        subprocess.run([sys.executable, "-m", "unittest", "discover", "-s", test_directory],
                       cwd=ROOT, check=True)


if __name__ == "__main__":
    main()
