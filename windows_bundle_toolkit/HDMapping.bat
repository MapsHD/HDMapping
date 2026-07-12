@echo off
setlocal EnableDelayedExpansion

:: ============================================================
::  HD Mapping - Windows Launcher
::  Run this script from the folder that contains the EXEs.
:: ============================================================

cd /d "%~dp0"

:menu
cls
echo =====================================================
echo   HD Mapping ^| Select a tool to launch
echo =====================================================
echo.
echo   1  Lidar Odometry Step 1
echo      (lidar_odometry_step_1.exe)
echo.
echo   2  Multi-View TLS Registration Step 2
echo      (multi_view_tls_registration_step_2.exe)
echo.
echo   3  Multi Session Registration Step 3
echo      (multi_session_registration_step_3.exe)
echo.
echo   4  Mandeye Raw Data Viewer
echo      (mandeye_raw_data_viewer.exe)
echo.
echo   5  Mandeye Single Session Viewer
echo      (mandeye_single_session_viewer.exe)
echo.
echo   6  Mandeye Mission Recorder Calibration
echo      (mandeye_mission_recorder_calibration.exe)
echo.
echo   7  Precision Forestry Tools
echo      (precision_forestry_tools.exe)
echo.
echo   8  Manual Color
echo      (mandeye_with_360_camera_manual_coloring.exe)
echo.
echo   9  Single Session Manual Coloring
echo      (single_session_manual_coloring.exe)
echo.
echo   0  Exit
echo.
echo =====================================================
set /p CHOICE="Enter number and press Enter: "

if "%CHOICE%"=="1" goto step1
if "%CHOICE%"=="2" goto step2
if "%CHOICE%"=="3" goto step3
if "%CHOICE%"=="4" goto raw_viewer
if "%CHOICE%"=="5" goto single_viewer
if "%CHOICE%"=="6" goto calibration
if "%CHOICE%"=="7" goto forestry
if "%CHOICE%"=="8" goto manual_color
if "%CHOICE%"=="9" goto single_manual_color
if "%CHOICE%"=="0" goto end
echo Invalid choice, try again.
timeout /t 2 >nul
goto menu

:step1
start "" "%~dp0lidar_odometry_step_1.exe"
goto menu

:step2
start "" "%~dp0multi_view_tls_registration_step_2.exe"
goto menu

:step3
start "" "%~dp0multi_session_registration_step_3.exe"
goto menu

:raw_viewer
start "" "%~dp0mandeye_raw_data_viewer.exe"
goto menu

:single_viewer
start "" "%~dp0mandeye_single_session_viewer.exe"
goto menu

:calibration
start "" "%~dp0mandeye_mission_recorder_calibration.exe"
goto menu

:forestry
start "" "%~dp0precision_forestry_tools.exe"
goto menu

:manual_color
start "" "%~dp0mandeye_with_360_camera_manual_coloring.exe"
goto menu

:single_manual_color
start "" "%~dp0single_session_manual_coloring.exe"
goto menu

:end
endlocal
