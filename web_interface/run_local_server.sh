#!/bin/bash

# TripleT Flight Firmware - Local Web Server Launcher
# This script provides multiple options to run the web interface locally
# without HTTPS redirection issues

echo "TripleT Flight Firmware - Web Interface Local Server"
echo "====================================================="
echo ""
echo "Choose a server option:"
echo "1. Python 3 HTTP Server (port 8000)"
echo "2. Python 2 HTTP Server (port 8000)"
echo "3. Node.js HTTP Server (port 8080)"
echo "4. PHP Built-in Server (port 8000)"
echo "5. Open directly in Chrome (file:// protocol)"
echo ""
read -p "Enter your choice (1-5): " choice

case $choice in
    1)
        echo "Starting Python 3 HTTP Server on http://localhost:8000"
        echo "Press Ctrl+C to stop the server"
        python3 -m http.server 8000
        ;;
    2)
        echo "Starting Python 2 HTTP Server on http://localhost:8000"
        echo "Press Ctrl+C to stop the server"
        python -m SimpleHTTPServer 8000
        ;;
    3)
        if command -v npx &> /dev/null; then
            echo "Starting Node.js HTTP Server on http://localhost:8080"
            echo "Press Ctrl+C to stop the server"
            npx http-server -p 8080
        else
            echo "Node.js/npx not found. Please install Node.js first."
        fi
        ;;
    4)
        if command -v php &> /dev/null; then
            echo "Starting PHP Built-in Server on http://localhost:8000"
            echo "Press Ctrl+C to stop the server"
            php -S localhost:8000
        else
            echo "PHP not found. Please install PHP first."
        fi
        ;;
    5)
        echo "Opening web interface directly in Chrome..."
        if command -v google-chrome &> /dev/null; then
            google-chrome index.html
        elif command -v chromium-browser &> /dev/null; then
            chromium-browser index.html
        else
            echo "Chrome/Chromium not found. Please install Chrome or Chromium."
        fi
        ;;
    *)
        echo "Invalid choice. Please run the script again and choose 1-5."
        ;;
esac 