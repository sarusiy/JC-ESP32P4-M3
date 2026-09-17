@echo off
powershell -NoExit -ExecutionPolicy Bypass -File "%~dp0s3-serial-monitor.ps1" %*
