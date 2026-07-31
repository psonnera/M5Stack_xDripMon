@echo off
rem Double-click launcher for build.ps1 (no execution-policy friction).
rem   build.bat             -> test build (update.inf untouched)
rem   build.bat -Release    -> release build (bumps update.inf)
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0build.ps1" %*
pause
