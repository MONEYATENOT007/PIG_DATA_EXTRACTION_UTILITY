@echo off
REM One-click builder for PIG DATA EXTRACTION UTILITY EXE
REM Double-click this file to build a one-file EXE using PyInstaller.

pushd "%~dp0"
echo Building PIG_DATA_EXTRACTION_UTILITY.exe using PyInstaller...
python build_exe.py
echo(
echo Done. Press any key to close this window.
pause >nul
popd

