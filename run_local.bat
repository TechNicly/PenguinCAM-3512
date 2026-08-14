@echo off
REM ===========================================================================
REM  PenguinCAM - local launcher for Windows
REM
REM  Double-click to start PenguinCAM on this machine, no Onshape sign-in.
REM  Export a part face from Onshape as DXF, upload it in the browser.
REM
REM  Put this file in the PenguinCAM-3512 folder and make a Desktop shortcut:
REM    right-click it -> Send to -> Desktop, create shortcut
REM  Copying it straight to the Desktop also works as long as the repo is at
REM    %USERPROFILE%\PenguinCAM-3512
REM
REM  First run creates the Python virtual environment and installs
REM  dependencies, which takes a minute. Later runs start immediately.
REM ===========================================================================

setlocal
title PenguinCAM - local

REM --- Locate the repository -------------------------------------------------
REM Works whether this file lives in the repo or was copied to the Desktop.
set "REPO=%~dp0"
if not exist "%REPO%frc_cam_gui_app.py" set "REPO=%USERPROFILE%\PenguinCAM-3512\"
if not exist "%REPO%frc_cam_gui_app.py" goto :norepo
cd /d "%REPO%"

echo ===========================================================================
echo   PenguinCAM - local mode
echo   Folder: %CD%
echo ===========================================================================
echo.

REM --- First run: create the venv and install dependencies -------------------
if not exist ".venv\Scripts\python.exe" (
    echo Setting up the Python environment. First run only, takes a minute...
    echo.
    python --version >nul 2>&1
    if errorlevel 1 goto :nopython
    python -m venv .venv
    if errorlevel 1 goto :novenv
    ".venv\Scripts\python.exe" -m pip install --disable-pip-version-check -r requirements.txt
    if errorlevel 1 goto :nodeps
    echo.
    echo Setup complete.
    echo.
)

REM --- Warn if the team config is missing ------------------------------------
REM Without it, generation silently falls back to Team 6238 defaults, which are
REM the wrong feeds and speeds for another team's machine.
if not exist "PenguinCAM-config.yaml" (
    echo ***************************************************************************
    echo   WARNING: no PenguinCAM-config.yaml in this folder.
    echo   Team 6238 default feeds and speeds will be used, NOT your machine's.
    echo   Copy your team config in here, then just restart this launcher.
    echo ***************************************************************************
    echo.
)

REM --- Local mode: no Google sign-in, no Onshape OAuth gate ------------------
set "AUTH_ENABLED=false"
set "ONSHAPE_AUTH_REQUIRED=false"

REM --- Open the browser once the server has had a moment to boot -------------
start "" /min powershell -NoProfile -Command "Start-Sleep -Seconds 5; Start-Process 'http://localhost:6238'"

echo Starting server on http://localhost:6238
echo A browser window will open shortly.
echo.
echo Close this window or press Ctrl+C to stop the server.
echo.

".venv\Scripts\python.exe" frc_cam_gui_app.py

echo.
echo Server stopped.
pause
exit /b 0

REM --- Failure paths ---------------------------------------------------------
:norepo
echo.
echo   Could not find PenguinCAM.
echo   Looked in: %~dp0
echo         and: %USERPROFILE%\PenguinCAM-3512
echo.
echo   Put this file inside the PenguinCAM-3512 folder, or clone the repo to
echo   your user folder with:
echo.
echo     git clone https://github.com/TechNicly/PenguinCAM-3512.git
echo.
pause
exit /b 1

:nopython
echo.
echo   Python was not found.
echo   Install it from https://www.python.org/downloads/windows/ and be sure to
echo   check "Add python.exe to PATH" on the first installer screen.
echo.
pause
exit /b 1

:novenv
echo.
echo   Could not create the virtual environment in .venv
echo.
pause
exit /b 1

:nodeps
echo.
echo   Installing dependencies failed. Check your internet connection, then
echo   delete the .venv folder and run this again.
echo.
pause
exit /b 1
