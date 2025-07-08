// JavaScript file for Web Serial API specific logic
const BAUD_RATE = 115200; // Common baud rate for serial communication

let port;
let portInfo = {}; // Store information about the connected port
let reader;
let keepReading = false;
let onDataReceivedCallback;
let onConnectCallback; // New callback for successful connection
let onDisconnectCallback;

/**
 * Checks for Web Serial API compatibility and prompts the user to select a serial port.
 * Opens the port, sets up a reader, and starts reading data.
 * @param {function} onConnect - Callback function when port is successfully connected. Receives portInfo.
 * @param {function} onData - Callback function for received data.
 * @param {function} onDisconnect - Callback function for port disconnection. Receives a reason string.
 */
async function connectSerial(doOnConnect, doOnData, doOnDisconnect) {
    onConnectCallback = doOnConnect;
    onDataReceivedCallback = doOnData;
    onDisconnectCallback = doOnDisconnect;

    if (!navigator.serial) {
        const errorMsg = "Web Serial API not supported by this browser.";
        const helpMsg = `
${errorMsg}

SOLUTION:
• Use Google Chrome (version 89+) - RECOMMENDED
• Use Microsoft Edge (version 89+)  
• Use Opera (version 75+)

NOT SUPPORTED:
• Firefox, Safari, Internet Explorer

Current browser: ${navigator.userAgent.split(' ')[0] || 'Unknown'}

Please switch to Chrome or Edge to use the serial interface.`;
        
        alert(helpMsg);
        console.warn(errorMsg);
        console.info("Browser compatibility info:", {
            userAgent: navigator.userAgent,
            hasSerial: !!navigator.serial,
            isSecureContext: window.isSecureContext,
            protocol: window.location.protocol
        });
        
        if (onDisconnectCallback) onDisconnectCallback(errorMsg, false); // false indicates not an active disconnect
        return;
    }

    try {
        console.log("Requesting serial port...");
        if (onConnectCallback) onConnectCallback({ status: 'connecting', message: "Requesting port..." });
        
        port = await navigator.serial.requestPort();
        const pInfo = port.getInfo();
        portInfo = { 
            usbVendorId: pInfo.usbVendorId, 
            usbProductId: pInfo.usbProductId,
            name: `Port (VID: ${pInfo.usbVendorId || 'N/A'}, PID: ${pInfo.usbProductId || 'N/A'})` // Create a user-friendly name
        };
        
        console.log(`Serial port selected: ${portInfo.name}. Opening port...`);
        if (onConnectCallback) onConnectCallback({ status: 'connecting', message: `Opening ${portInfo.name}...` });

        await port.open({ baudRate: BAUD_RATE });
        console.log(`Serial port ${portInfo.name} opened successfully with baud rate ${BAUD_RATE}.`);
        keepReading = true;
        
        if (onConnectCallback) onConnectCallback({ status: 'connected', message: `Connected to ${portInfo.name}`, portInfo: portInfo });
        
        readLoop(); // Don't await this, let it run in the background
    } catch (error) {
        console.error("Error opening serial port:", error);
        const errorMsg = `Error opening port: ${error.message}`;
        alert(errorMsg); // Keep alert for immediate user feedback on critical error
        if (onDisconnectCallback) onDisconnectCallback(errorMsg, false); // false = not an active disconnect
        port = null; 
        portInfo = {};
    }
}

async function readLoop() {
    if (!port || !port.readable) {
        console.warn("Port is not readable or not defined.");
        if (keepReading) {
            handleDisconnect("Port became unreadable or undefined.", true);
        }
        return;
    }

    reader = port.readable.getReader();
    const textDecoder = new TextDecoder('utf-8');
    let buffer = '';

    console.log("Starting read loop...");
    try {
        while (port.readable && keepReading) {
            const { value, done } = await reader.read();
            if (done) {
                console.log("Reader stream done (port closed by device or disconnectSerial).");
                break; 
            }
            if (value) {
                const decodedChunk = textDecoder.decode(value, { stream: true });
                buffer += decodedChunk;

                let newlineIndex;
                while ((newlineIndex = buffer.indexOf('\n')) !== -1) {
                    const line = buffer.substring(0, newlineIndex).trim();
                    buffer = buffer.substring(newlineIndex + 1);
                    if (line && onDataReceivedCallback) {
                        onDataReceivedCallback(line);
                    }
                }
            }
        }
    } catch (error) {
        console.error("Error during serial port reading:", error);
        if (keepReading) { 
           handleDisconnect(`Read error: ${error.message}`, true);
        }
    } finally {
        if (reader) {
            try {
                // No await here, as if reader.cancel() was called, releaseLock might be called before cancel() promise resolves
                reader.releaseLock();
            } catch (lockError) {
                if (!(lockError instanceof DOMException && lockError.name === 'InvalidStateError')) {
                     console.warn("Error releasing reader lock:", lockError);
                }
            }
            reader = null;
            console.log("Reader lock released.");
        }
    }
    if (keepReading) { // If loop exited but we still expected to read (e.g. device unplugged)
        handleDisconnect("Read loop terminated unexpectedly.", true);
    }
}

async function disconnectSerial() {
    const wasReading = keepReading; // Store if we were actively reading
    keepReading = false; 

    if (reader) {
        console.log("Cancelling reader...");
        try {
            await reader.cancel("User initiated disconnect"); 
            // reader.releaseLock() should be handled by readLoop's finally block
        } catch (error) {
            console.warn("Error cancelling reader:", error);
        }
    }

    if (port) {
        const pName = portInfo.name || "serial port";
        console.log(`Closing ${pName}...`);
        try {
            await port.close();
            console.log(`${pName} closed.`);
            handleDisconnect(`Disconnected from ${pName}.`, true); // true = active disconnect
        } catch (error) {
            console.error(`Error closing ${pName}:`, error);
            handleDisconnect(`Error closing ${pName}: ${error.message}`, true);
        } finally {
            port = null;
            portInfo = {};
        }
    } else {
        console.log("No active serial port to disconnect.");
        // If there was no port, but a disconnect callback exists, call it.
        // This can happen if connectSerial failed early or disconnect is clicked multiple times.
        // Only call if it wasn't an "active" disconnect (e.g. we weren't connected)
        if (onDisconnectCallback && !wasReading) { 
            onDisconnectCallback("No active port to disconnect.", false);
        }
    }
}

/**
 * Internal helper to manage disconnection state and callbacks.
 * @param {string} reason - The reason for disconnection.
 * @param {boolean} isActiveDisconnect - True if this was a result of an active connection being terminated.
 */
function handleDisconnect(reason, isActiveDisconnect) {
    console.log(`Disconnected: ${reason}`);
    keepReading = false; 
    
    // Attempt to ensure the reader is released if not already (failsafe)
    if (reader && port && port.readable && !reader.closed) {
        reader.cancel().catch(() => {/* ignore */});
    }

    if (onDisconnectCallback) {
        onDisconnectCallback(reason, isActiveDisconnect);
    }
    
    // Reset callbacks *only if* it was an active disconnect or to prevent stale states
    // For instance, if connect failed early, we don't want to nullify a callback that might be needed for status update
    if (isActiveDisconnect) {
        onConnectCallback = null; 
        onDataReceivedCallback = null;
        // onDisconnectCallback = null; // Careful: might be needed for multiple disconnect signals
    }
    
    if (isActiveDisconnect || !port) { // clear port info if actively disconnected or port was never established
        port = null;
        portInfo = {};
    }
}

async function sendSerialData(data) {
    if (!port || !port.writable) {
        console.error("Serial port not connected or not writable.");
        // Optionally: alert("Serial port not connected or not writable.");
        return;
    }

    const writer = port.writable.getWriter();
    const textEncoder = new TextEncoder(); 

    try {
        // console.log(`Sending data: ${data}`); // Can be verbose
        await writer.write(textEncoder.encode(data + '\n'));
        // console.log("Data sent successfully.");
    } catch (error) {
        console.error("Error writing to serial port:", error);
    } finally {
        if (writer) {
            try {
                await writer.releaseLock();
            } catch (lockError) {
                 console.warn("Error releasing writer lock:", lockError);
            }
        }
        // console.log("Writer lock released.");
    }
}
