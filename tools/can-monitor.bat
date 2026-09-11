@echo off
powershell -NoExit -ExecutionPolicy Bypass -File "%~dp0can-monitor.ps1" %*
