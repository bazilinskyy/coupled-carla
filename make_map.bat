@echo off
rem Windows: double-click this file to open the map maker in your browser (docs\make_a_map.md).
rem It uses the project's .venv when there is one (the one-time setup makes it), else the py launcher's Python 3
rem (3.10 or newer), else python. Extra arguments go to the map maker (make_map.bat --no-browser --port 8790);
rem set MAPMAKER_NO_PAUSE=1 to skip the pause at the end (a test, a scheduled task).
setlocal
cd /d "%~dp0"
set "PY="
if exist ".venv\Scripts\python.exe" set "PY=.venv\Scripts\python.exe"
if not defined PY py -3 -c "import sys; sys.exit(sys.version_info < (3, 10))" >nul 2>&1 && set "PY=py -3"
if not defined PY python -c "import sys; sys.exit(sys.version_info < (3, 10))" >nul 2>&1 && set "PY=python"
if not defined PY goto nopython
%PY% -m coupledcarla.mapmaker --web %*
set "CODE=%ERRORLEVEL%"
goto done
:nopython
echo The map maker needs Python 3.10 or newer with its packages: do the one-time setup in docs\make_a_map.md ("Set up once").
set "CODE=3"
:done
if not defined MAPMAKER_NO_PAUSE pause
exit /b %CODE%
