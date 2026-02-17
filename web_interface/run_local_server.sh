#!/bin/bash

# TripleT Flight Firmware - Local Web Server Launcher
# This script provides multiple options to run the web interface locally
# without HTTPS redirection issues

echo "TripleT Flight Firmware - Web Interface Local Server"
echo "====================================================="
echo ""
echo "🚨 IMPORTANT: This script helps avoid HTTPS redirection issues"
echo "Choose a server option:"
echo "1. Python 3 HTTP Server (port 8000)"
echo "2. Python 3 HTTP Server (port 3000) - Alternative port"
echo "3. Python 3 HTTP Server (port 9000) - Another alternative"
echo "4. Node.js HTTP Server (port 8080)"
echo "5. PHP Built-in Server (port 8000)"
echo "6. Open directly in Chrome (file:// protocol)"
echo "7. Python server on 127.0.0.1 (port 8000)"
echo "8. Kill any existing Python servers"
echo ""
read -p "Enter your choice (1-8): " choice

case $choice in
    1)
        echo "Starting Python 3 HTTP Server on http://localhost:8000"
        echo "🌐 Open: http://localhost:8000"
        echo "Press Ctrl+C to stop the server"
        python3 -m http.server 8000
        ;;
    2)
        echo "Starting Python 3 HTTP Server on http://localhost:3000"
        echo "🌐 Open: http://localhost:3000"
        echo "Press Ctrl+C to stop the server"
        python3 -m http.server 3000
        ;;
    3)
        echo "Starting Python 3 HTTP Server on http://localhost:9000"
        echo "🌐 Open: http://localhost:9000"
        echo "Press Ctrl+C to stop the server"
        python3 -m http.server 9000
        ;;
    4)
        if command -v npx &> /dev/null; then
            echo "Starting Node.js HTTP Server on http://localhost:8080"
            echo "🌐 Open: http://localhost:8080"
            echo "Press Ctrl+C to stop the server"
            npx http-server -p 8080
        else
            echo "Node.js/npx not found. Please install Node.js first."
        fi
        ;;
    5)
        if command -v php &> /dev/null; then
            echo "Starting PHP Built-in Server on http://localhost:8000"
            echo "🌐 Open: http://localhost:8000"
            echo "Press Ctrl+C to stop the server"
            php -S localhost:8000
        else
            echo "PHP not found. Please install PHP first."
        fi
        ;;
    6)
        echo "Opening web interface directly in Chrome (file:// protocol)..."
        if command -v google-chrome &> /dev/null; then
            google-chrome index.html
        elif command -v chromium-browser &> /dev/null; then
            chromium-browser index.html
        else
            echo "Chrome/Chromium not found. Please install Chrome or Chromium."
        fi
        ;;
    7)
        echo "Starting Python 3 HTTP Server on http://127.0.0.1:8000"
        echo "🌐 Open: http://127.0.0.1:8000"
        echo "Press Ctrl+C to stop the server"
        python3 -m http.server 8000 --bind 127.0.0.1
        ;;
    8)
        echo "Killing any existing Python HTTP servers..."
        pkill -f "python3 -m http.server"
        pkill -f "python -m http.server"
        pkill -f "python -m SimpleHTTPServer"
        echo "Done. Any existing Python servers have been stopped."
        ;;
    *)
        echo "Invalid choice. Please run the script again and choose 1-8."
        ;;
esac 