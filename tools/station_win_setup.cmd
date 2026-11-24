@echo off
rem One-time setup of a participant station on Windows (coupled-carla), everything under C:\cc58 (or %CC58%),
rem nothing system-wide: a Python 3.10 kept in %CC58%\uvpython (the CARLA 1.0 package ships only a cp310 Windows
rem wheel), a venv %CC58%\station_venv with that wheel, numpy, pygame, pyyaml, requests, pillow, and networkx and
rem shapely for CARLA's agents (the --input bot driver). Needs uv.
rem     tools\station_win_setup.cmd [path\to\carla-1.0-cp310-cp310-win_amd64.whl]
setlocal
if "%CC58%"=="" set CC58=C:\cc58
set UV_CACHE_DIR=%CC58%\uvcache
set UV_PYTHON_INSTALL_DIR=%CC58%\uvpython
set WHEEL=%~1
if "%WHEEL%"=="" set WHEEL=%CC58%\carla\Build\Package\Carla-1.0-Win64-Shipping\PythonAPI\carla\dist\carla-1.0-cp310-cp310-win_amd64.whl
if not exist "%WHEEL%" (echo no CARLA wheel at %WHEEL% & exit /b 1)
uv python install 3.10 || exit /b 1
set PY=
for /d %%P in ("%CC58%\uvpython\cpython-3.10.*-windows-x86_64-none") do set PY=%%P\python.exe
if "%PY%"=="" (echo no Python 3.10 in %CC58%\uvpython & exit /b 1)
uv venv --allow-existing --python "%PY%" "%CC58%\station_venv" || exit /b 1
uv pip install --python "%CC58%\station_venv\Scripts\python.exe" "%WHEEL%" numpy pygame pyyaml requests pillow networkx shapely || exit /b 1
"%CC58%\station_venv\Scripts\python.exe" -c "import carla, pygame, numpy; import coupledcarla.station; print('station ready')"
