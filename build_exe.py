"""
Helper script to build a one-file Windows EXE from this project.

Usage (from this folder, with Python and PyInstaller installed):

    python build_exe.py

This will create:
  - build/  (PyInstaller intermediate files)
  - dist/PIG_DATA_EXTRACTION_UTILITY.exe

Firmware (firmware/*.uf2) and boards_config.json are NOT bundled into the EXE.
At runtime, keep them next to the EXE in the same folder, e.g.:

    PIG_DATA_EXTRACTION_UTILITY.exe
    boards_config.json
    firmware/
        DATA.uf2
        EGP.uf2
        MFL.uf2
        CMFL.uf2
"""

import os
import subprocess
import sys


def main() -> None:
    base_dir = os.path.dirname(os.path.abspath(__file__))
    script_path = os.path.join(base_dir, "main.py")

    if not os.path.exists(script_path):
        raise SystemExit(f"Could not find main script at {script_path}")

    dist_dir = os.path.join(base_dir, "dist")
    build_dir = os.path.join(base_dir, "build")

    os.makedirs(dist_dir, exist_ok=True)
    os.makedirs(build_dir, exist_ok=True)

    exe_name = "PIG_DATA_EXTRACTION_UTILITY"

    cmd = [
        sys.executable,
        "-m",
        "PyInstaller",
        "--noconfirm",
        "--clean",
        "--onefile",
        "--name",
        exe_name,
        "--distpath",
        dist_dir,
        "--workpath",
        build_dir,
        # No --add-data entries: firmware + boards_config.json remain external.
        script_path,
    ]

    print("Running PyInstaller:")
    print(" ", " ".join(cmd))
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        raise SystemExit(f"PyInstaller failed with exit code {e.returncode}") from e

    exe_path = os.path.join(dist_dir, exe_name + ".exe")
    if os.path.exists(exe_path):
        print(f"\nBuild OK. EXE created at:\n  {exe_path}")
        print(
            "\nMake sure the following are in the same directory when you run it:\n"
            "  - boards_config.json (optional, created on first run)\n"
            "  - firmware/ (containing DATA.uf2, EGP.uf2, MFL.uf2, CMFL.uf2)"
        )
    else:
        print("\nPyInstaller completed but EXE not found where expected:")
        print("  ", exe_path)


if __name__ == "__main__":
    main()

