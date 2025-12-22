PIG DATA EXTRACTION UTILITY - EXE BUILD NOTES
============================================

This project can be run directly with Python, or bundled into a single
Windows EXE using PyInstaller. Firmware files and the board registry
remain external to the EXE so they can be updated without rebuilding.

FILES
-----

- PIG_EXTRACTION_UPD.py
  Main application (PyQt GUI + CLI).

- build_exe.py
  Python helper script that invokes PyInstaller to create a one-file EXE.

- build_exe.bat
  One-click wrapper around build_exe.py. Double-click to build the EXE.

- firmware\
  Contains the UF2 firmware files:
    - DATA.uf2
    - EGP.uf2
    - MFL.uf2
    - CMFL.uf2

- boards_config.json
  Registry of boards (serial -> name, type, pipe_size, exclude slots).
  Created/updated by the app; not required on first run.

PREREQUISITES
-------------

1) Install Python (3.8+ recommended) and ensure "python" is on PATH.
2) Install PyInstaller once:

   pip install pyinstaller

BUILDING THE EXE
----------------

Option A: One-click (recommended)
  1) Make sure you are in the folder that contains build_exe.bat.
  2) Double-click build_exe.bat.
  3) Wait for the console window to show "Build OK" or an error.

Option B: Manual command
  1) Open a terminal in this folder.
  2) Run:

     python build_exe.py

OUTPUT
------

- dist\PIG_DATA_EXTRACTION_UTILITY.exe
  One-file Windows EXE built by PyInstaller.

- build\
  PyInstaller intermediate files. Can be deleted if not needed.

DEPLOYMENT LAYOUT
-----------------

On the target machine, keep the EXE and external resources in the same
directory, for example:

  PIG_DATA_EXTRACTION_UTILITY.exe
  boards_config.json          (optional; auto-created if missing)
  firmware\
      DATA.uf2
      EGP.uf2
      MFL.uf2
      CMFL.uf2

The application will look for firmware and the registry file next to
the EXE first, then in the current working directory, and finally in
the PyInstaller unpack directory (for older bundled builds).

