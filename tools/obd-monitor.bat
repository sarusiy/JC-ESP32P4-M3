@echo off
powershell -NoExit -ExecutionPolicy Bypass -File "%~dp0obd-monitor.ps1" %*
