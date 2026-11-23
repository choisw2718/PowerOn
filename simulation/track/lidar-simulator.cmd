@echo off
setlocal
pushd "%~dp0..\.." || exit /b 1
python -m simulation.track %*
set "launch_exit=%ERRORLEVEL%"
popd
if not "%launch_exit%"=="0" pause
exit /b %launch_exit%
