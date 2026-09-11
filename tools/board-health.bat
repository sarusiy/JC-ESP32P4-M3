@echo off
powershell -NoExit -ExecutionPolicy Bypass -File "%~dp0board-health.ps1" %*
