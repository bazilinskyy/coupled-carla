@echo off
rem Windows: double-click this file to open the map maker in your browser (docs\make_a_map.md).
rem It uses the project's .venv when there is one (the one-time setup makes it), else the py launcher.
cd /d "%~dp0"
set "PY=py -3"
if exist ".venv\Scripts\python.exe" set "PY=.venv\Scripts\python.exe"
%PY% -m coupledcarla.mapmaker --web
pause
