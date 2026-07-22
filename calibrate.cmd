@echo off
setlocal

set "ROOT=%~dp0"
set "PYTHON=%ROOT%.venv\Scripts\python.exe"

call "%ROOT%setup.cmd"
if errorlevel 1 exit /b 1

"%PYTHON%" -m calibrate %*
