@echo off
setlocal

set "ROOT=%~dp0"

pushd "%ROOT%"
uv run --no-sync python -m calibrate %*
set "RESULT=%ERRORLEVEL%"
popd

exit /b %RESULT%
