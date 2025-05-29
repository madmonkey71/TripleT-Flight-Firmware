// Main JavaScript file for the Flight Data Visualizer

document.addEventListener('DOMContentLoaded', () => {
    console.log("Flight Data Visualizer initialized.");

    const connectButton = document.getElementById('connectButton');
    const disconnectButton = document.getElementById('disconnectButton');
    const connectionStatusDiv = document.getElementById('connectionStatus');
    let parserInitialized = false;

    // --- Update Connection Status Display ---
    function updateConnectionStatus(message, statusClass) {
        if (connectionStatusDiv) {
            connectionStatusDiv.textContent = `Status: ${message}`;
            connectionStatusDiv.className = 'status-' + statusClass; // e.g., status-connected, status-disconnected
        } else {
            console.warn("connectionStatusDiv not found.");
        }
    }
    // Set initial status
    updateConnectionStatus("Disconnected", "disconnected");


    // --- Initialize Data Parser ---
    initDataParser(
        () => {
            console.log("Data parser initialized successfully in main.js.");
            parserInitialized = true;
            if (connectButton) connectButton.disabled = false; // Enable connect button once parser is ready
        },
        (error) => {
            console.error("Failed to initialize data parser in main.js:", error);
            parserInitialized = false;
            updateConnectionStatus("Parser Error: " + error.message, "error");
            if (connectButton) {
                connectButton.disabled = true;
                connectButton.title = "Data parser failed to initialize.";
            }
        }
    );

    // --- Serial Handler Callbacks ---
    function handleSerialConnect(connectionInfo) { // Updated to receive an object
        // connectionInfo can be { status: 'connecting', message: '...' } or { status: 'connected', message: '...', portInfo: {...} }
        console.log("Serial connection event:", connectionInfo.message);
        if (connectionInfo.status === 'connected') {
            updateConnectionStatus(connectionInfo.message, "connected");
            if (connectButton) connectButton.disabled = true;
            if (disconnectButton) disconnectButton.disabled = false;
        } else if (connectionInfo.status === 'connecting') {
            updateConnectionStatus(connectionInfo.message, "connecting");
            if (connectButton) connectButton.disabled = true;
            if (disconnectButton) disconnectButton.disabled = true;
        }
    }

    function handleReceivedData(data) {
        if (!parserInitialized) return;
        const parsedData = parseSerialData(data);
        if (parsedData) {
            if (typeof updateUI === 'function') {
                updateUI(parsedData);
            } else {
                console.error("updateUI function is not defined.");
            }
        }
    }

    function handleSerialDisconnect(reason, isActiveDisconnect) {
        console.log("Serial port disconnected:", reason);
        updateConnectionStatus(reason, isActiveDisconnect ? "disconnected" : "error");
        if (connectButton) connectButton.disabled = !parserInitialized; // Only enable if parser is OK
        if (disconnectButton) disconnectButton.disabled = true;
    }

    // --- Button Event Listeners ---
    if (connectButton) {
        // Disable initially until parser is ready
        connectButton.disabled = true; 
        connectButton.addEventListener('click', () => {
            if (!parserInitialized) {
                alert("Data parser is not ready. Please wait or check console for errors.");
                updateConnectionStatus("Parser not ready.", "error");
                return;
            }
            console.log("Connect button clicked.");
            // Pass all three handlers to connectSerial
            connectSerial(handleSerialConnect, handleReceivedData, handleSerialDisconnect);
        });
    } else {
        console.error("Connect button not found.");
    }

    if (disconnectButton) {
        disconnectButton.disabled = true; // Initially disabled
        disconnectButton.addEventListener('click', () => {
            console.log("Disconnect button clicked.");
            disconnectSerial();
        });
    } else {
        console.error("Disconnect button not found.");
    }

    // --- Initial Check for Web Serial API Support ---
    if (!navigator.serial) {
        const msg = "Web Serial API not supported.";
        console.warn(msg + " Please use Chrome or Edge version 89 or later.");
        updateConnectionStatus(msg, "error");
        if (connectButton) {
            connectButton.disabled = true;
            connectButton.title = "Web Serial API is not supported by this browser.";
        }
        if (disconnectButton) {
            disconnectButton.disabled = true;
        }
        const controlsDiv = document.getElementById('connectionControls'); // Changed from 'controls'
        if (controlsDiv) {
            // Check if message already exists
            if (!controlsDiv.querySelector('.api-support-warning')) {
                const warningMsg = document.createElement('p');
                warningMsg.textContent = "Web Serial API not supported. Please use Chrome or Edge.";
                warningMsg.style.color = 'red';
                warningMsg.classList.add('api-support-warning'); // Add class to prevent duplicates
                controlsDiv.appendChild(warningMsg);
            }
        }
    } else {
        console.log("Web Serial API is supported.");
        if (parserInitialized && connectButton) { // Re-enable if parser is already good to go
             connectButton.disabled = false;
        }
    }
});
