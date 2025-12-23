@echo off                                                                                                                  
  REM One-click builder for PIG DATA EXTRACTION UTILITY EXE                                                                  
  REM Uses Python 3.13 explicitly via the py launcher.                                                                       
                                                                                                                             
  pushd "%~dp0"                                                                                                              
  echo Building PIG_DATA_EXTRACTION_UTILITY.exe using Python 3.13...                                                         
  py -3.13 build_exe.py                                                                                                      
  echo(                                                                                                                      
  echo Done. Press any key to close this window.                                                                             
  pause >nul                                                                                                                 
  popd            