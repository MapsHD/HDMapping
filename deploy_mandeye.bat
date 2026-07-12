@echo off
setlocal EnableExtensions EnableDelayedExpansion

rem ============================================================
rem  deploy_mandeye.bat
rem
rem  Bundles the built mandeye executables/DLLs together with the
rem  PROJ database, a sample geoid grid and a placeholder sample
rem  GPX track into a single release folder.
rem
rem  Always also produces a .zip of the deploy folder.
rem
rem  Usage:
rem    deploy_mandeye.bat [deploy_dir]
rem
rem    deploy_dir  - optional, defaults to <repo>\deploy\mandeye
rem ============================================================

set "REPO_ROOT=%~dp0"
if "%REPO_ROOT:~-1%"=="\" set "REPO_ROOT=%REPO_ROOT:~0,-1%"

set "BUILD_DIR=%REPO_ROOT%\build\bin\RelWithDebInfo"
if not exist "%BUILD_DIR%" set "BUILD_DIR=%REPO_ROOT%\build\bin\Release"

set "PROJ_DB_SRC=%REPO_ROOT%\build\3rd_binary\PROJ\install_proj\share\proj\proj.db"

set "DEPLOY_DIR=%REPO_ROOT%\deploy\mandeye"
if not "%~1"=="" set "DEPLOY_DIR=%~1"

set "GEOID_NAME=egm96_15.gtx"
set "GEOID_URL=https://raw.githubusercontent.com/OSGeo/proj-datumgrid/master/egm96_15.gtx"

echo.
echo === Mandeye deploy ===
echo Build dir : %BUILD_DIR%
echo Deploy dir: %DEPLOY_DIR%
echo.

if not exist "%BUILD_DIR%\mandeye_raw_data_viewer.exe" (
    echo ERROR: %BUILD_DIR% does not look like a built output folder.
    echo Build the project first, e.g.:
    echo   cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
    echo   cmake --build build --config RelWithDebInfo
    exit /b 1
)

if not exist "%DEPLOY_DIR%" mkdir "%DEPLOY_DIR%"
if not exist "%DEPLOY_DIR%\proj" mkdir "%DEPLOY_DIR%\proj"
if not exist "%DEPLOY_DIR%\sample_data" mkdir "%DEPLOY_DIR%\sample_data"

echo Copying executables and DLLs...
xcopy /Y /Q "%BUILD_DIR%\*.exe" "%DEPLOY_DIR%\" >nul
xcopy /Y /Q "%BUILD_DIR%\*.dll" "%DEPLOY_DIR%\" >nul

rem Everything the last known-good build produced. If a target wasn't
rem built (e.g. you only built a subset), it just won't be in BUILD_DIR
rem and xcopy above silently skips it - so check explicitly and warn.
set "EXPECTED=concatenate_multi_livox.exe drag_folder_with_mandeye_data_and_drop_here-precision_forestry.exe laz_to_pcd.exe laz_to_ply.exe laz_to_txt.exe lidar_odometry_step_1.exe livox_mid_360_intrinsic_calibration.exe mandeye_compare_trajectories.exe mandeye_mission_recorder_calibration.exe mandeye_raw_data_viewer.exe mandeye_single_session_viewer.exe mandeye_with_360_camera_manual_coloring.exe matrix_mul.exe multiply_timestamps_session_point_cloud_laz.exe multiply_timestamps_session_trajectory_csv.exe multi_session_registration_step_3.exe multi_view_tls_registration_step_2.exe pcd_to_laz.exe precision_forestry_tools.exe single_session_manual_coloring.exe split_multi_livox.exe freeglut.dll laszip3.dll opencv_world4130.dll proj_9_3.dll tbb12.dll z.dll"

set "MISSING="
for %%F in (%EXPECTED%) do (
    if not exist "%DEPLOY_DIR%\%%F" set "MISSING=!MISSING! %%F"
)

if defined MISSING (
    echo   WARNING: the following expected files were not found in the build output
    echo   and are missing from the deploy folder ^(target not built?^):
    for %%F in (!MISSING!) do echo     - %%F
) else (
    echo   all expected executables and DLLs present.
)

echo Copying PROJ database (proj.db)...
if exist "%PROJ_DB_SRC%" (
    copy /Y "%PROJ_DB_SRC%" "%DEPLOY_DIR%\proj.db" >nul
) else (
    echo   WARNING: proj.db not found at %PROJ_DB_SRC%
    echo   Did you build the project? PROJ transforms will not work without it.
)

echo Fetching sample geoid grid (EGM96 15', NGA, public domain)...
if exist "%DEPLOY_DIR%\%GEOID_NAME%" (
    echo   already present, skipping download.
) else (
    curl -fsSL -o "%DEPLOY_DIR%\proj\%GEOID_NAME%" "%GEOID_URL%"
    if errorlevel 1 (
        echo   WARNING: failed to download %GEOID_NAME%.
        echo   Add a geoid grid manually to %DEPLOY_DIR%\proj if you need geoid correction.
    )
)

>"%DEPLOY_DIR%\NOTICE_geoid.txt" (
    echo %GEOID_NAME%
    echo Source: NGA ^(National Geospatial-Intelligence Agency^), EGM96 15-arcminute geoid undulation grid
    echo License: Public Domain
    echo Obtained from: %GEOID_URL%
)

set "GIT_TAG=notag"
set "GIT_COMMIT=nogit"
for /f "delims=" %%i in ('git -C "%REPO_ROOT%" describe --tags --exact-match 2^>nul') do set "GIT_TAG=%%i"
if "%GIT_TAG%"=="notag" (
    for /f "delims=" %%i in ('git -C "%REPO_ROOT%" describe --tags --abbrev=0 2^>nul') do set "GIT_TAG=%%i"
)
for /f "delims=" %%i in ('git -C "%REPO_ROOT%" rev-parse --short HEAD 2^>nul') do set "GIT_COMMIT=%%i"

for %%I in ("%DEPLOY_DIR%") do set "DEPLOY_PARENT=%%~dpI"
if "%DEPLOY_PARENT:~-1%"=="\" set "DEPLOY_PARENT=%DEPLOY_PARENT:~0,-1%"
for %%I in ("%DEPLOY_DIR%") do set "DEPLOY_NAME=%%~nI"

set "ZIP_PATH=%DEPLOY_PARENT%\%DEPLOY_NAME%_%GIT_TAG%_%GIT_COMMIT%.zip"

echo Creating zip archive...
powershell -NoProfile -Command "Compress-Archive -Path '%DEPLOY_DIR%\*' -DestinationPath '%ZIP_PATH%' -Force"
echo Zip created: %ZIP_PATH%

echo.
echo Done. Deployed to: %DEPLOY_DIR%
endlocal
