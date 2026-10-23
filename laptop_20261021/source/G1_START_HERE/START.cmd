@echo off
"%~dp0..\g1_sdk_venv\Scripts\python.exe" "%~dp0START.py" %*
if "%~1"=="" pause
