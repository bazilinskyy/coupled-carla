@echo off
rem A participant station on Windows itself, not in WSL: the camera images come straight from the CARLA server on
rem this PC (WSL's network bridge caps them at about 140 MB/s: 17 fps at 1920x1080). Setup once:
rem tools\station_win_setup.cmd. The host (python -m coupledcarla.run ...) may run in WSL on the same PC: with
rem mirrored networking both reach 127.0.0.1. Any station option can follow, e.g.
rem     tools\station_win.cmd --role driver --input wheel --size 1920x1080 --fullscreen
setlocal
if "%CC58%"=="" set CC58=C:\cc58
rem the bot driver (--input bot) plans its route with CARLA's agents (PythonAPI\carla\agents of the package)
if "%CARLA_ROOT%"=="" set CARLA_ROOT=%CC58%\carla\Build\Package\Carla-1.0-Win64-Shipping
rem another venv (STATION_VENV), e.g. one with a patched client wheel
if "%STATION_VENV%"=="" set STATION_VENV=%CC58%\station_venv
cd /d "%~dp0.."
"%STATION_VENV%\Scripts\python.exe" -m coupledcarla.station --host 127.0.0.1 %*
