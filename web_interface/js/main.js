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

        const fields = data.substring(4).split(',');
        const condensed = fields.slice(0, 3).join(',') + `... (${fields.length} fields)`;
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
    if (connectButton) {
        connectButton.addEventListener('click', () => {
            connectSerial(handleSerialConnect, handleReceivedData, handleSerialDisconnect);
        });
    }

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
    if (!navigator.serial) {
        const msg = "Web Serial API not supported by this browser.";
        updateConnectionStatus(msg, "error");
        if (connectButton) {
            connectButton.disabled = true;
            connectButton.title = msg;
        }
    }
});
