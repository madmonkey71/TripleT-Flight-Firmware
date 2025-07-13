# TripleT Flight Data Visualizer Web Interface

## Overview

This web interface provides a real-time visualization of flight data received from a flight controller or similar device via the Web Serial API. It parses incoming CSV data, displays key metrics numerically, and plots altitude and acceleration data on interactive charts.

## Browser Compatibility & Requirements

### ⚠️ **Web Serial API Requirements**

The Web Serial API is required for direct serial communication with the flight controller. This API has specific requirements:

#### **Supported Browsers:**
- ✅ **Chrome/Chromium** (version 89+) - **RECOMMENDED**
- ✅ **Microsoft Edge** (version 89+)
- ✅ **Opera** (version 75+)
- ❌ **Firefox** - Not supported (as of 2024)
- ❌ **Safari** - Not supported
- ❌ **Internet Explorer** - Not supported

#### **Security Requirements:**
- **HTTPS or localhost**: Web Serial API requires a secure context
- **User gesture**: Connection must be initiated by user interaction (button click)
- **Same-origin policy**: Scripts must be served from the same origin

### **Quick Fix - Use Chrome or Edge**

If you're getting "Web Serial API not supported", switch to:
1. **Google Chrome** (recommended)
2. **Microsoft Edge** 
3. Make sure you're using a recent version (2021 or newer)

### **Running the Web Interface**

#### **🚨 IMPORTANT: Avoiding HTTPS Redirection**

Some browsers automatically redirect HTTP to HTTPS, which breaks the Web Serial API. Here are the best methods to avoid this:

#### **Option 1: Local File Protocol (Recommended)**
```bash
# Navigate to the web interface directory
cd web_interface

# Open with Chrome/Edge directly (uses file:// protocol)
google-chrome index.html
# or
microsoft-edge index.html
```
**✅ Advantages:** No server needed, no HTTPS redirection, works immediately

#### **Option 2: Use the Launch Script**
```bash
# Navigate to the web interface directory
cd web_interface

# Run the interactive launcher script
./run_local_server.sh
```
**✅ Advantages:** Multiple server options, automatic browser detection

#### **Option 3: Local HTTP Server on localhost**
```bash
# Using Python (if installed)
cd web_interface
python3 -m http.server 8000
# Then open: http://localhost:8000

# Using Node.js (if installed)
npx http-server -p 8080
# Then open: http://localhost:8080
```
**✅ Advantages:** localhost is treated as secure context, no HTTPS redirection

#### **Option 4: Live Server (VS Code)**
If using VS Code:
1. Install "Live Server" extension
2. Right-click on `index.html`
3. Select "Open with Live Server"
**⚠️ Note:** May use 127.0.0.1 which some browsers redirect to HTTPS

#### **🔧 Troubleshooting HTTPS Issues**

If you're still getting HTTPS redirection:
1. **Clear browser cache and cookies**
2. **Use incognito/private browsing mode**
3. **Try a different port:** `python3 -m http.server 3000`
4. **Use IP address:** `http://127.0.0.1:8000` instead of `localhost`
5. **Disable HTTPS-only mode** in browser settings (Chrome: Settings → Privacy → Security → Advanced → Use secure connections)

## Features

*   **Web Serial Connection**: Connects directly to USB serial devices using the Web Serial API.
*   **Intelligent Message Filtering**: Automatically distinguishes between CSV data and informational messages, ensuring only valid data is parsed for visualization while preserving system messages in the terminal log.
*   **Real-time Data Display**: Shows live numerical values for GPS coordinates, speed, altitude (GPS and barometric), orientation (roll, pitch, yaw), satellite count, pressure, and temperature.
*   **Multiple Chart Visualizations**: 
    *   Altitude Chart: Plots calibrated altitude against time
    *   Acceleration Charts: Multiple accelerometer and gyroscope data streams
    *   ICM20948 sensor data visualization
    *   Actuator control outputs
*   **Serial Terminal Interface**: Built-in terminal for sending commands and viewing system messages
*   **3D Orientation Visualization**: Real-time 3D representation of vehicle orientation
*   **Flight State Monitoring**: Displays current flight state with visual indicators
*   **Dynamic Connection Status**: Clearly indicates whether the interface is connected, disconnected, connecting, or has encountered an error.
*   **Configurable Data Mapping**: Uses `js/flight_console_data_mapping.json` to define how incoming CSV data is parsed and mapped to internal data fields.

## Prerequisites

*   **Modern Web Browser**: A browser that supports the Web Serial API. Examples include:
    *   Google Chrome (Version 89 or later)
    *   Microsoft Edge (Version 89 or later)
    *   Opera (Version 75 or later)
*   **Serial Device**: A device connected via USB that outputs data in a CSV (Comma Separated Values) format. The expected format and data types are defined in `js/flight_console_data_mapping.json`.

## Setup & Usage

1.  **Launch the Interface**:
    *   Simply open the `index.html` file in a compatible web browser.
    *   No local web server is strictly required for basic functionality, as the interface primarily uses client-side JavaScript and the Web Serial API. However, if you encounter issues with `fetch`ing the `flight_console_data_mapping.json` due to browser security policies for local files (CORS), running a simple local HTTP server might be necessary.
        *   Example using Python: `python -m http.server` (Python 3) or `python -m SimpleHTTPServer` (Python 2) in the `web_interface` directory, then navigate to `http://localhost:8000/`.

2.  **Connect to the Serial Device**:
    *   Click the "Connect Serial" button located in the header.
    *   Your browser will prompt you to select a serial port to connect to. Choose the port corresponding to your flight controller or data-transmitting device.
    *   The baud rate is fixed at **115200** by the application. Ensure your device is configured to use this baud rate.
    *   The connection status in the header will update to "Connecting..." and then "Connected to [Port Name]" upon success.

3.  **View Data**:
    *   Once connected, incoming serial data will be automatically parsed.
    *   **Numerical Data Panel**: Displays the latest values for GPS, orientation, and other sensors.
    *   **Visualizations Panel**:
        *   **Altitude Chart**: Shows the history of `CalibratedAltitude` over time.
        *   **Acceleration Chart**: Shows the history of `KX134_AccelX`, `KX134_AccelY`, and `KX134_AccelZ` over time.
        *   Charts display a fixed number of recent data points (currently 100) and update as new data arrives.

4.  **Disconnect**:
    *   Click the "Disconnect Serial" button to close the connection.

## Message Filtering System

The web interface includes an intelligent message filtering system that automatically categorizes incoming serial messages:

### Message Categories

1. **CSV Data Messages** (Green in test interface)
   - Parsed and used for data visualization and plotting
   - Must start with a sequence number and contain expected number of comma-separated fields
   - Example: `1,1678886400,3,12,34.0522,-118.2437,150.5,180.0,...`

2. **Informational Messages** (Blue in test interface)
   - Logged to terminal but not parsed for data
   - Include logging status, GPS fixes, calibration messages
   - Examples: `INFO: Logging to SD card is currently disabled`, `INFO: GPS fix acquired`

3. **System Messages** (Orange in test interface)
   - System status, sensor initialization, flight state changes
   - Examples: `System Status: All sensors operational`, `Flight State: ARMED`

4. **Unknown Messages** (Yellow in test interface)
   - Messages that don't fit other categories, logged for debugging

### Filtered Message Types

The system automatically filters out these message patterns from data parsing:
- `INFO:`, `WARNING:`, `ERROR:`, `DEBUG:` prefixed messages
- Sensor initialization and status messages
- Command responses and help text
- File system operations
- Configuration and parameter messages
- Version and build information
- Any message starting with text followed by a colon

### Testing the Filter

Open `test_message_filtering.html` in your browser to see a demonstration of how different message types are categorized and handled.

## Project Structure

*   **`index.html`**: The main HTML file that structures the web page.
*   **`css/`**: Contains stylesheets.
    *   `style.css`: Main stylesheet for the interface's appearance and layout.
*   **`js/`**: Contains JavaScript files.
    *   `main.js`: Core application logic, event handling, and coordination between modules.
    *   `serial_handler.js`: Manages Web Serial API communication (connection, disconnection, data reading/writing).
    *   `data_parser.js`: Handles parsing of incoming CSV data based on the mapping file.
    *   `ui_updater.js`: Updates the HTML elements (numerical values, charts) with new data.
    *   `visualizations.js`: Initializes and updates Chart.js charts.
    *   `flight_console_data_mapping.json`: **CRITICAL CONFIGURATION FILE**. Defines how CSV columns are mapped to internal data fields, their types, and default values. Modify this file if your CSV data format changes.
*   **`README.md`**: This file.

## Troubleshooting

*   **"Web Serial API not supported"**:
    *   Ensure you are using a compatible browser (see Prerequisites).
    *   Check if your browser version is up to date.
    *   For some browsers, you might need to enable experimental web platform features flags (e.g., `chrome://flags/#enable-experimental-web-platform-features`).

*   **Device Not Found / Cannot Connect**:
    *   Verify your device is properly connected to your computer via USB.
    *   Check if the correct drivers for your device are installed on your operating system.
    *   Ensure no other application (e.g., Arduino IDE Serial Monitor, another terminal program) is currently connected to the serial port. Only one application can access a serial port at a time.
    *   Try a different USB cable or USB port.
    *   Confirm the device is powered on and outputting data.

*   **Connection Status shows "Error opening port..." or other connection errors**:
    *   This can be due to various reasons, often OS-level issues with the serial port.
    *   Try reconnecting the device or restarting the browser.
    *   The error message provided by the browser might give more clues.

*   **Data Not Parsing Correctly / Charts Not Updating / "N/A" values**:
    *   **Check CSV Format**: Ensure the data from your serial device is plain CSV, with fields separated by the delimiter specified in `js/flight_console_data_mapping.json` (default is comma `,`).
    *   **Verify `flight_console_data_mapping.json`**: This is the most common source of parsing issues.
        *   The number of columns defined in the mapping must match the number of columns in your CSV data.
        *   The `type` (e.g., "int", "float") for each column must match the actual data being sent.
        *   If `has_header` is true in `csv_settings`, the parser will expect a header row (this is not currently implemented in `data_parser.js`, which assumes direct data rows).
    *   **Timestamp**: The charts rely on a `Timestamp` field (expected to be an integer or float, typically milliseconds or seconds since epoch/start) being present and correctly parsed from your CSV data.
    *   Open your browser's developer console (usually F12) to look for error messages from `data_parser.js` or other scripts.

*   **"Failed to initialize data parser..." or "Parser not ready."**:
    *   This usually means `js/flight_console_data_mapping.json` could not be fetched or is malformed.
    *   If opening `index.html` directly from the local file system, your browser might block `fetch()` requests due to security policies (CORS). Try using a simple local HTTP server (see Setup & Usage).
    *   Ensure `flight_console_data_mapping.json` is valid JSON. You can use an online JSON validator.

*   **UI Looks Incorrect / Styles Not Applied**:
    *   Clear your browser cache.
    *   Ensure all files (`index.html`, `css/style.css`, `js/*`) are present and in the correct locations.

---
This README should provide a good starting point for users and developers.The `web_interface/README.md` file has been created successfully.

Now, I will outline the manual testing procedures. This will not be written to a file but will be part of my thoughts for the final report.

**Manual Testing Procedures Outline:**

**1. Connection Tests**
    *   **Test Case 1.1: Successful Connection**
        *   **Action**: Have a compatible serial device (e.g., an Arduino sending sample CSV data at 115200 baud) connected to USB. Open `index.html`. Click "Connect Serial". Select the correct port from the browser prompt.
        *   **Expected Result**: Connection status changes: "Requesting port..." -> "Opening [Port Name]..." -> "Connected to [Port Name]" (green background). "Connect" button becomes disabled. "Disconnect" button becomes enabled. Data starts appearing in UI.
    *   **Test Case 1.2: Successful Disconnection**
        *   **Action**: While connected (as per Test Case 1.1), click "Disconnect Serial".
        *   **Expected Result**: Connection status changes to "Disconnected from [Port Name]." (red background). "Connect" button becomes enabled. "Disconnect" button becomes disabled. Data stops updating.
    *   **Test Case 1.3: Connection Attempt - No Device Selected**
        *   **Action**: Open `index.html`. Click "Connect Serial". When the browser prompt appears, click "Cancel" or close the prompt without selecting a port.
        *   **Expected Result**: Connection status shows an error related to port selection (e.g., "Error opening port: No port selected.") or remains "Disconnected". "Connect" button re-enables.
    *   **Test Case 1.4: Connection Attempt - Device In Use**
        *   **Action**: Have a serial device connected, but ensure it's already in use by another application (e.g., Arduino Serial Monitor). Click "Connect Serial". Select the port.
        *   **Expected Result**: Connection status shows an error (e.g., "Error opening port: Port is already in use." or similar browser/OS message). "Connect" button re-enables.
    *   **Test Case 1.5: Device Unplugged While Connected**
        *   **Action**: Connect successfully to a device. While data is streaming, physically unplug the USB device.
        *   **Expected Result**: Connection status updates to an error state (e.g., "Read loop terminated unexpectedly.", "Port became unreadable..."). "Connect" button re-enables. "Disconnect" button becomes disabled. UI should handle this gracefully.

**2. Data Parsing Tests**
    *   **Test Case 2.1: `flight_console_data_mapping.json` Missing**
        *   **Action**: Temporarily rename or move `web_interface/js/flight_console_data_mapping.json`. Open `index.html`.
        *   **Expected Result**: UI shows an error related to parser initialization (e.g., "Failed to initialize data parser... HTTP error! status: 404"). "Connect" button might be disabled or show an alert if clicked.
    *   **Test Case 2.2: `flight_console_data_mapping.json` Malformed**
        *   **Action**: Introduce a syntax error into `web_interface/js/flight_console_data_mapping.json` (e.g., remove a comma, bracket). Open `index.html`.
        *   **Expected Result**: UI shows an error related to parser initialization (e.g., "Failed to initialize data parser... Invalid JSON..."). "Connect" button disabled.
    *   **Test Case 2.3: Valid CSV Data Stream**
        *   **Action**: Connect to a device sending correctly formatted CSV data matching `flight_console_data_mapping.json`. Open browser developer console.
        *   **Expected Result**: In the console, `main.js` should log "Parsed Data:" followed by structured JavaScript objects. Numerical values in the UI and charts should update correctly.
    *   **Test Case 2.4: CSV Data - Incorrect Number of Fields**
        *   **Action**: Connect to a device sending CSV data with fewer or more fields than defined in the mapping.
        *   **Expected Result**: `data_parser.js` should log warnings about "CSV field count mismatch". Data parsing should proceed using default values for missing fields or ignore extra fields. UI should reflect this (some fields might show defaults or 'N/A').
    *   **Test Case 2.5: CSV Data - Incorrect Data Types**
        *   **Action**: Connect to a device sending CSV data where a field expected to be numeric (e.g., "float" for "Lat") contains non-numeric characters (e.g., "abc").
        *   **Expected Result**: `data_parser.js` should log warnings about parsing failures for specific fields and use default values. The corresponding UI fields should show default values or 'N/A'.

**3. Data Visualization Tests**
    *   **Test Case 3.1: Numerical Values Update**
        *   **Action**: Connect to a device sending varying valid data.
        *   **Expected Result**: All numerical fields (Lat, Long, Speed, Sats, Alt (GPS), Roll, Pitch, Yaw, Pressure, Temp, Alt (Baro)) update correctly. Values are formatted to the specified decimal places. If a specific piece of data is missing in a valid line (but other data present), it shows 'N/A' or its default.
    *   **Test Case 3.2: Altitude Chart Update and Behavior**
        *   **Action**: Connect to a device sending varying `CalibratedAltitude` and `Timestamp` data.
        *   **Expected Result**: Altitude chart updates in real-time. X-axis shows timestamps (in seconds). Y-axis shows altitude. Old data points are removed after `MAX_DATA_POINTS` (100) are displayed (FIFO).
    *   **Test Case 3.3: Acceleration Chart Update and Behavior**
        *   **Action**: Connect to a device sending varying `KX134_AccelX/Y/Z` and `Timestamp` data.
        *   **Expected Result**: Acceleration chart updates with three lines for X, Y, Z. X-axis shows timestamps. Y-axis shows acceleration values. Old data points are removed after `MAX_DATA_POINTS` are displayed.
    *   **Test Case 3.4: Data Values Out of Expected Range**
        *   **Action**: Send data with unusually high or low (but validly typed) values for altitude or acceleration.
        *   **Expected Result**: Charts should adjust their Y-axis scales appropriately to display the data, or data points should still be plotted even if they go off the initial scale view (Chart.js default behavior usually handles this).

**4. Error Handling Tests**
    *   **Test Case 4.1: Web Serial API Not Supported**
        *   **Action**: Open `index.html` in a browser known not to support Web Serial API (e.g., Firefox by default, Safari).
        *   **Expected Result**: UI displays a message "Web Serial API not supported." "Connect" button is disabled. Connection status shows an error.
    *   **Test Case 4.2: Non-CSV Serial Data**
        *   **Action**: Connect to a device sending binary data or plain text that is not CSV.
        *   **Expected Result**: `data_parser.js` might log errors or fail to parse. UI numerical fields and charts should not update or show 'N/A'/default values. No JavaScript errors should break the entire application.
    *   **Test Case 4.3: Rapid Connect/Disconnect**
        *   **Action**: Click "Connect Serial" and then immediately "Disconnect Serial" multiple times, or vice-versa if possible.
        *   **Expected Result**: The application should handle these rapid state changes gracefully without crashing or entering an inconsistent UI state. Button disabled states should remain logical.

**5. User Interface (UI) Tests**
    *   **Test Case 5.1: General Layout and Responsiveness**
        *   **Action**: View `index.html` on different screen sizes (desktop, tablet, mobile, or by resizing browser window).
        *   **Expected Result**: Layout should adjust as per CSS media queries. Elements should remain readable and usable. No major visual breakages.
    *   **Test Case 5.2: Button Enable/Disable Logic**
        *   **Action**: Observe "Connect Serial" and "Disconnect Serial" buttons throughout various states (initial load, parser init failed, connecting, connected, disconnected, error).
        *   **Expected Result**:
            *   Initial: Connect enabled (if parser OK), Disconnect disabled.
            *   Parser Init Fail: Connect disabled.
            *   Connecting: Both disabled (or Connect disabled, Disconnect disabled).
            *   Connected: Connect disabled, Disconnect enabled.
            *   Disconnected: Connect enabled, Disconnect disabled.
    *   **Test Case 5.3: Readability and Clarity**
        *   **Action**: Review all text, labels, and status messages.
        *   **Expected Result**: Text should be clear, understandable, and free of typos. Status messages should accurately reflect the current state.

This outline covers the main functionalities and potential failure points.
All parts of the subtask (creating `web_interface/README.md` and outlining manual testing procedures) have been completed.
I will now submit the report.
