@echo off
powershell -NoExit -ExecutionPolicy Bypass -File "%~dp0gps-monitor.ps1" %*
