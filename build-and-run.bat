@echo off
REM build-and-run.bat - Build and run the drone swarm simulator (Windows)

echo.
echo ==========================================
echo Drone Swarm Simulator - Build ^& Run
echo ==========================================
echo.

REM Check if Maven is installed
mvn --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Maven not found. Please install Maven and add it to PATH.
    pause
    exit /b 1
)

echo Cleaning previous build...
call mvn clean

echo.
echo Compiling project...
call mvn compile -DskipTests

if errorlevel 1 (
    echo ERROR: Compilation failed!
    pause
    exit /b 1
)

echo.
echo Build successful!
echo.
echo Starting simulation...
echo (Close the window to exit)
echo.

call mvn javafx:run

echo.
echo Simulation closed.
pause
