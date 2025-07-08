// Main JavaScript file for the Flight Data Visualizer

document.addEventListener('DOMContentLoaded', async () => {
    console.log("Flight Data Visualizer initialized.");

    const connectButton = document.getElementById('connectButton');
    const disconnectButton = document.getElementById('disconnectButton');
    const connectionStatusDiv = document.getElementById('connectionStatus');
    const serialTerminalOutput = document.getElementById('serialTerminalOutput');
    const serialCommandInput = document.getElementById('serialCommandInput');
    const serialSendCommandButton = document.getElementById('serialSendCommandButton');
    
    // --- Serial Terminal Logging Function ---
    function logToTerminal(message, type = 'info') {
        if (serialTerminalOutput) {
            const D = new Date();
            const timestamp = `${String(D.getHours()).padStart(2, '0')}:${String(D.getMinutes()).padStart(2, '0')}:${String(D.getSeconds()).padStart(2, '0')}`;
            const line = document.createElement('div');
            line.textContent = `[${timestamp}] ${type.toUpperCase()}: ${message}`;
            switch (type) {
                case 'sent': line.style.color = '#82E0AA'; break;
                case 'received': line.style.color = '#AED6F1'; break;
                case 'error': line.style.color = '#F1948A'; break;
                case 'info':
                default: line.style.color = '#F7DC6F'; break;
            }
            serialTerminalOutput.appendChild(line);
            serialTerminalOutput.scrollTop = serialTerminalOutput.scrollHeight;
        }
    }

    // --- Update Connection Status Display ---
    function updateConnectionStatus(message, statusClass) {
        if (connectionStatusDiv) {
            connectionStatusDiv.textContent = `Status: ${message}`;
            connectionStatusDiv.className = 'status-' + statusClass;
        }
        logToTerminal(`Connection status: ${message}`, statusClass === 'error' ? 'error' : 'info');
    }
    
    // --- Initialize Data Parser and UI ---
    try {
        if (connectButton) connectButton.disabled = true;
        updateConnectionStatus("Initializing...", "disconnected");

        await initDataParser();
        
        if (typeof initUI === 'function') {
            initUI();
        }

        updateConnectionStatus("Ready to connect", "disconnected");
        if (connectButton) connectButton.disabled = false;

    } catch (error) {
        console.error("Initialization failed:", error);
        updateConnectionStatus("Initialization Error. Check console.", "error");
        if (connectButton) {
            connectButton.disabled = true;
            connectButton.title = "Application failed to initialize.";
        }
    }

    // --- Serial Handler Callbacks ---
    function handleSerialConnect(connectionInfo) {
        updateConnectionStatus(connectionInfo.message, connectionInfo.status);
        if (connectionInfo.status === 'connected') {
            if (connectButton) connectButton.disabled = true;
            if (disconnectButton) disconnectButton.disabled = false;
        }
    }

    function handleReceivedData(data) {
        const parsedData = parseData(data);

        if (!parsedData) {
            logToTerminal(data.trim(), 'received');
            return;
        }
        
        if (parsedData.isStateUpdate) {
            if (typeof updateUI === 'function') {
                updateUI(parsedData);
            }
            return;
        }

        const parts = data.trim().split(',');
        const condensed = parts.slice(0, 3).join(',') + `... (${parts.length} fields)`;
        logToTerminal(`CSV: ${condensed}`, 'received');
        
        if (typeof updateUI === 'function') {
            updateUI(parsedData);
        }
    }

    function handleSerialDisconnect(reason, isActiveDisconnect) {
        updateConnectionStatus(reason, isActiveDisconnect ? "disconnected" : "error");
        if (connectButton) connectButton.disabled = false;
        if (disconnectButton) disconnectButton.disabled = true;
    }

    // --- Button Event Listeners ---
    // Note: Connect button listener is now handled after browser compatibility check

    if (disconnectButton) {
        disconnectButton.disabled = true;
        disconnectButton.addEventListener('click', () => {
            disconnectSerial();
        });
    }

    // --- Serial Command Sending ---
    function sendSerialCommand() {
        if (serialCommandInput && serialCommandInput.value.trim() !== "") {
            const command = serialCommandInput.value.trim();
            sendSerialData(command);
            logToTerminal(command, 'sent');
            serialCommandInput.value = "";
        }
    }

    if (serialSendCommandButton) {
        serialSendCommandButton.addEventListener('click', sendSerialCommand);
    }

    if (serialCommandInput) {
        serialCommandInput.addEventListener('keypress', (event) => {
            if (event.key === "Enter") {
                sendSerialCommand();
            }
        });
    }

    // --- Initial Check for Web Serial API Support ---
    function checkBrowserCompatibility() {
        const userAgent = navigator.userAgent.toLowerCase();
        const isChrome = userAgent.includes('chrome') && !userAgent.includes('edge');
        const isEdge = userAgent.includes('edge');
        const isOpera = userAgent.includes('opera') || userAgent.includes('opr');
        const isFirefox = userAgent.includes('firefox');
        const isSafari = userAgent.includes('safari') && !userAgent.includes('chrome');
        
        let browserName = 'Unknown';
        if (isChrome) browserName = 'Chrome';
        else if (isEdge) browserName = 'Edge';
        else if (isOpera) browserName = 'Opera';
        else if (isFirefox) browserName = 'Firefox';
        else if (isSafari) browserName = 'Safari';
        
        // More detailed debug information
        console.log("Browser compatibility check:", {
            userAgent: navigator.userAgent,
            browserName: browserName,
            hasNavigatorSerial: !!navigator.serial,
            navigatorSerial: navigator.serial,
            isSecureContext: window.isSecureContext,
            protocol: window.location.protocol,
            location: window.location.href
        });
        
        if (!navigator.serial) {
            const msg = `Web Serial API not supported in ${browserName}. Use Chrome or Edge.`;
            updateConnectionStatus(msg, "error");
            logToTerminal(`Browser: ${browserName}`, 'error');
            logToTerminal("SOLUTION: Use Chrome (recommended) or Microsoft Edge", 'info');
            logToTerminal("NOT SUPPORTED: Firefox, Safari, Internet Explorer", 'error');
            logToTerminal("DEBUG: navigator.serial = " + navigator.serial, 'error');
            
            if (connectButton) {
                connectButton.disabled = true;
                connectButton.title = `Web Serial API not supported in ${browserName}`;
                connectButton.innerHTML = `❌ Not Supported in ${browserName}`;
            }
        } else {
            logToTerminal(`Browser: ${browserName} ✅ (Web Serial API supported)`, 'info');
            logToTerminal("Protocol: " + window.location.protocol, 'info');
            logToTerminal("Secure context: " + window.isSecureContext, 'info');
            logToTerminal("DEBUG: navigator.serial = " + typeof navigator.serial, 'info');
        }
        
        return !!navigator.serial; // Return the result for use by other functions
    }
    
    const hasWebSerial = checkBrowserCompatibility();
    
    // Override the connect button to add extra checking
    if (connectButton && hasWebSerial) {
        connectButton.addEventListener('click', () => {
            // Double-check serial API availability right before connecting
            if (!navigator.serial) {
                logToTerminal("ERROR: Web Serial API disappeared between page load and button click!", 'error');
                updateConnectionStatus("Web Serial API not available", "error");
                return;
            }
            
            logToTerminal("Initiating serial connection...", 'info');
            connectSerial(handleSerialConnect, handleReceivedData, handleSerialDisconnect);
        });
    }
});
