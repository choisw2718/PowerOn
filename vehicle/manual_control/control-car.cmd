@echo off
setlocal
pushd "%~dp0..\.." || exit /b 1
py -3 -m vehicle %*
set "launch_exit=%ERRORLEVEL%"
popd
if not "%launch_exit%"=="0" pause
exit /b %launch_exit%
