@echo off
echo TripleT Flight Firmware - Web Interface Local Server
echo =====================================================
echo.
echo 🚨 IMPORTANT: This script helps avoid HTTPS redirection issues
echo Choose a server option:
echo 1. Python 3 HTTP Server (port 8000)
echo 2. Python 3 HTTP Server (port 3000) - Alternative port
echo 3. Python 3 HTTP Server (port 9000) - Another alternative
echo 4. Node.js HTTP Server (port 8080)
echo 5. Open directly in Chrome (file:// protocol)
echo 6. Python server on 127.0.0.1 (port 8000)
echo.
set /p choice="Enter your choice (1-6): "

if "%choice%"=="1" (
    echo Starting Python 3 HTTP Server on http://localhost:8000
    echo 🌐 Open: http://localhost:8000
    echo Press Ctrl+C to stop the server
    python -m http.server 8000
)
if "%choice%"=="2" (
    echo Starting Python 3 HTTP Server on http://localhost:3000
    echo 🌐 Open: http://localhost:3000
    echo Press Ctrl+C to stop the server
    python -m http.server 3000
)
if "%choice%"=="3" (
    echo Starting Python 3 HTTP Server on http://localhost:9000
    echo 🌐 Open: http://localhost:9000
    echo Press Ctrl+C to stop the server
    python -m http.server 9000
)
if "%choice%"=="4" (
    where npx >nul 2>nul
    if %errorlevel%==0 (
        echo Starting Node.js HTTP Server on http://localhost:8080
        echo 🌐 Open: http://localhost:8080
        echo Press Ctrl+C to stop the server
        npx http-server -p 8080
    ) else (
        echo Node.js/npx not found. Please install Node.js first.
    )
)
if "%choice%"=="5" (
    echo Opening web interface directly in Chrome (file:// protocol)...
    where chrome >nul 2>nul
    if %errorlevel%==0 (
        chrome index.html
    ) else (
        where chrome.exe >nul 2>nul
        if %errorlevel%==0 (
            chrome.exe index.html
        ) else (
            echo Chrome not found. Please install Chrome or open index.html manually.
        )
    )
)
if "%choice%"=="6" (
    echo Starting Python 3 HTTP Server on http://127.0.0.1:8000
    echo 🌐 Open: http://127.0.0.1:8000
    echo Press Ctrl+C to stop the server
    python -m http.server 8000 --bind 127.0.0.1
)

if not "%choice%"=="1" if not "%choice%"=="2" if not "%choice%"=="3" if not "%choice%"=="4" if not "%choice%"=="5" if not "%choice%"=="6" (
    echo Invalid choice. Please run the script again and choose 1-6.
)

pause 