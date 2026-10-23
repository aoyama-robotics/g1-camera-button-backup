@echo off
setlocal
set PYTHONIOENCODING=utf-8
set PYTHONUTF8=1
"%~dp0..\g1_sdk_venv\Scripts\python.exe" "%~dp0START.py" %*
