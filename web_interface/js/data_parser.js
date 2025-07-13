// JavaScript file for parsing incoming data from the serial port

// This module will hold the configuration once loaded.
let config = {
    csvHeaders: [],
    flightStateMap: {}
};

// Fallback configuration for when JSON file cannot be fetched (file:// protocol)
const FALLBACK_CONFIG = {
    csvHeaders: [
        "SeqNum", "Timestamp", "FlightState", "FixType", "Sats", "Lat", "Long", "Alt", 
        "AltMSL", "RawAltitude", "CalibratedAltitude", "Speed", "Heading", "pDOP", "RTK", 
        "Pressure", "Temperature", "KX134_AccelX", "KX134_AccelY", "KX134_AccelZ", 
        "ICM_AccelX", "ICM_AccelY", "ICM_AccelZ", "ICM_GyroX", "ICM_GyroY", "ICM_GyroZ", 
        "ICM_MagX", "ICM_MagY", "ICM_MagZ", "ICM_Temp", "Q0", "Q1", "Q2", "Q3", 
        "EulerRoll_rad", "EulerPitch_rad", "EulerYaw_rad", "GyroBiasX_rps", "GyroBiasY_rps", 
        "GyroBiasZ_rps", "ActuatorOutRoll", "ActuatorOutPitch", "ActuatorOutYaw", 
        "TgtRoll", "TgtPitch", "TgtYaw", "PIDIntRoll", "PIDIntPitch", "PIDIntYaw"
    ],
    flightStateMap: {
        "0": "STARTUP",
        "1": "CALIBRATION",
        "2": "PAD_IDLE",
        "3": "ARMED",
        "4": "BOOST",
        "5": "COAST",
        "6": "APOGEE",
        "7": "DROGUE_DEPLOY",
        "8": "DROGUE_DESCENT",
        "9": "MAIN_DEPLOY",
        "10": "MAIN_DESCENT",
        "11": "LANDED",
        "12": "RECOVERY",
        "13": "ERROR",
        "UNKNOWN": "UNKNOWN"
    }
};

/**
 * Determines if a message should be ignored for data parsing/plotting.
 * @param {string} message - The incoming message to evaluate
 * @returns {boolean} True if the message should be ignored for data parsing
 */
function shouldIgnoreMessage(message) {
    if (!message || typeof message !== 'string') {
        return true;
    }
    
    const trimmedMessage = message.trim();
    
    // Ignore empty messages
    if (trimmedMessage.length === 0) {
        return true;
    }
    
    // Define patterns for messages that should be ignored for data parsing
    const ignorePatterns = [
        // Informational messages
        /^INFO:/i,
        /^WARNING:/i,
        /^ERROR:/i,
        /^DEBUG:/i,
        
        // Specific logging messages
        /logging to sd card is currently disabled/i,
        /sd card.*not available/i,
        /loggingEnabled=false/i,
        /log file.*created/i,
        /log file.*closed/i,
        
        // System status messages
        /^system status:/i,
        /^sensor status:/i,
        /^connection status:/i,
        /^initialization/i,
        /^calibration/i,
        
        // Command responses
        /^command received:/i,
        /^unknown command:/i,
        /^help:/i,
        /^available commands:/i,
        
        // Sensor initialization messages
        /sensor.*initialized/i,
        /sensor.*failed/i,
        /^gps.*fix/i,
        /^barometer.*calibrated/i,
        /icm.*ready/i,
        /kx134.*ready/i,
        /ms5611.*ready/i,
        
        // Flight state messages (descriptive, not data)
        /^flight state:/i,
        /^entering.*state/i,
        /^state changed/i,
        /^current state:/i,
        
        // General system messages
        /^ready/i,
        /^waiting/i,
        /^armed/i,
        /^disarmed/i,
        /^startup/i,
        /^shutdown/i,
        
        // File system messages
        /^file.*created/i,
        /^file.*closed/i,
        /^sd card.*space/i,
        /^storage.*available/i,
        /^free space:/i,
        
        // Memory and performance messages
        /^memory usage:/i,
        /^free ram:/i,
        /^loop time:/i,
        /^frequency:/i,
        
        // Configuration messages
        /^config:/i,
        /^setting:/i,
        /^parameter:/i,
        
        // Firmware version and build info
        /^version:/i,
        /^build:/i,
        /^firmware:/i,
        /^compiled:/i,
        
        // Any message that starts with text followed by colon (likely status message)
        /^[a-zA-Z][a-zA-Z\s]*:/
    ];
    
    // Check if message matches any ignore pattern
    for (const pattern of ignorePatterns) {
        if (pattern.test(trimmedMessage)) {
            return true;
        }
    }
    
    return false;
}

/**
 * Determines if a message is likely CSV data that should be parsed.
 * @param {string} message - The incoming message to evaluate
 * @returns {boolean} True if the message appears to be CSV data
 */
function isCSVDataMessage(message) {
    if (!message || typeof message !== 'string') {
        return false;
    }
    
    const trimmedMessage = message.trim();
    
    // TEMPORARY: Much more lenient CSV detection for debugging
    console.log(`CSV Detection: Checking message: "${trimmedMessage.substring(0, 50)}..."`);
    
    // Check if it starts with a number (sequence number)
    if (!/^\d+/.test(trimmedMessage)) {
        console.log('CSV Detection: Does not start with number');
        return false;
    }
    
    // Count comma-separated fields
    const fields = trimmedMessage.split(',');
    console.log(`CSV Detection: Found ${fields.length} fields`);
    
    // Much more lenient - just need a few fields
    if (fields.length < 5) {
        console.log(`CSV Detection: Message has only ${fields.length} fields, need at least 5`);
        return false;
    }
    
    // Skip the field count check temporarily for debugging
    // if (columnMappings && Math.abs(fields.length - columnMappings.length) > 10) {
    //     console.log(`CSV Detection: Field count ${fields.length} differs too much from expected ${columnMappings.length}`);
    //     return false;
    // }
    
    // Very lenient numeric check - just check if first field is a number
    const firstField = fields[0].trim();
    if (!/^\d+$/.test(firstField)) {
        console.log(`CSV Detection: First field (${firstField}) is not a pure number`);
        return false;
    }
    
    console.log(`CSV Detection: ✓ Message identified as CSV data with ${fields.length} fields`);
    return true;
}

/**
 * Categorizes a message type for appropriate handling.
 * @param {string} message - The incoming message to categorize
 * @returns {string} Message category: 'csv_data', 'info', 'system', 'unknown'
 */
function categorizeMessage(message) {
    if (!message || typeof message !== 'string') {
        return 'unknown';
    }
    
    if (isCSVDataMessage(message)) {
        return 'csv_data';
    }
    
    if (shouldIgnoreMessage(message)) {
        // Further categorize informational messages
        const trimmedMessage = message.trim().toLowerCase();
        if (trimmedMessage.includes('info:') || 
            trimmedMessage.includes('logging') || 
            trimmedMessage.includes('sd card')) {
            return 'info';
        }
        return 'system';
    }
    
    return 'unknown';
}

/**
 * Initializes the parser by fetching the configuration file.
 * This must be called before the main application starts trying to parse data.
 * @returns {Promise<void>} A promise that resolves when the config is loaded.
 */
async function initDataParser() {
    try {
        const response = await fetch('js/flight_console_data_mapping.json');
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        const mapping = await response.json();
        
        // Validate the structure of the loaded JSON
        if (!mapping.csv_headers || !mapping.flight_states) {
            throw new Error("Invalid mapping file format. Missing 'csv_headers' or 'flight_states'.");
        }
        
        config.csvHeaders = mapping.csv_headers;
        config.flightStateMap = mapping.flight_states;
        
        console.log("Data parser initialized successfully from JSON file.");

    } catch (error) {
        console.warn("Failed to load configuration from JSON file:", error);
        console.log("Using fallback configuration (file:// protocol compatibility)");
        
        // Use fallback configuration for file:// protocol
        config.csvHeaders = FALLBACK_CONFIG.csvHeaders;
        config.flightStateMap = FALLBACK_CONFIG.flightStateMap;
        
        console.log("Data parser initialized successfully with fallback configuration.");
    }
}

/**
 * Parses a single line of text from the serial port.
 * It can handle both full CSV lines and the special JSON state messages.
 * @param {string} line - The raw line of text from the serial port.
 * @returns {object|null} A structured data object or null if the line is not valid data.
 */
function parseData(line) {
    const trimmedLine = line.trim();
    console.log("🔍 PARSE DEBUG: Processing line:", trimmedLine);

    // 1. Handle JSON state messages: {"state_id":3,"state_name":"ARMED"}
    if (trimmedLine.startsWith('{') && trimmedLine.endsWith('}')) {
        try {
            const stateData = JSON.parse(trimmedLine);
            if (stateData && typeof stateData.state_name !== 'undefined') {
                console.log("🔍 PARSE DEBUG: JSON state message found:", stateData);
                return { 
                    isStateUpdate: true,
                    FlightState: stateData.state_name 
                };
            }
        } catch (e) {
            console.log("🔍 PARSE DEBUG: JSON parsing failed:", e);
            return null;
        }
    }

    // 2. Handle CSV data lines from firmware, which do not have a "CSV:" prefix.
    // We identify them by checking if the first part is a number (sequence number).
    const parts = trimmedLine.split(',');
    console.log("🔍 PARSE DEBUG: CSV parts:", parts.length, "Expected headers:", config.csvHeaders.length);
    
    if (parts.length > 1 && !isNaN(parts[0])) {
        console.log("🔍 PARSE DEBUG: Line starts with number, checking CSV format");
        
        // Ensure headers are loaded and the data has the correct number of fields
        if (config.csvHeaders.length === 0) {
            console.warn("🔍 PARSE DEBUG: No CSV headers loaded!");
            return null;
        }
        
        if (parts.length !== config.csvHeaders.length) {
            console.warn(`🔍 PARSE DEBUG: CSV mismatch: Expected ${config.csvHeaders.length}, got ${parts.length}.`);
            console.log("🔍 PARSE DEBUG: Headers:", config.csvHeaders);
            console.log("🔍 PARSE DEBUG: First few parts:", parts.slice(0, 10));
            return null;
        }

        const dataObject = {};
        config.csvHeaders.forEach((header, index) => {
            dataObject[header] = parts[index] ? parts[index].trim() : '';
        });

        if (dataObject.FlightState !== undefined) {
            dataObject.FlightState = config.flightStateMap[dataObject.FlightState] || 'UNKNOWN';
        }
        
        console.log("🔍 PARSE DEBUG: Successfully parsed CSV data:", dataObject);
        return dataObject;
    }

    console.log("🔍 PARSE DEBUG: Line doesn't match any known format");
    return null;
}

// Example of how main.js might use this:
// initDataParser(() => {
//   console.log("Parser ready.");
//   // Now safe to call parseSerialData
//   const testData = "1,1678886400,3,12,34.0522,-118.2437,150.5,180.0,155.2,152.3,10.2,90.0,1.5,1,1012.5,25.5,0.1,0.2,9.8,0.1,0.2,9.7,0.01,0.02,0.03,0.1,0.2,0.3,22.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.5,0.2,0.1,0.0,0.0,0.0,0.0,0.0,0.0";
//   const parsed = parseSerialData(testData);
//   console.log("Parsed Data:", parsed);
// });

// Function to fetch and set CSV headers from the JSON mapping file
async function loadCsvHeaders() {
    try {
        const response = await fetch('js/flight_console_data_mapping.json');
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        const mapping = await response.json();
        config.csvHeaders = mapping.csv_headers;
        console.log("CSV headers loaded successfully:", config.csvHeaders);
    } catch (error) {
        console.error("Failed to load CSV headers:", error);
    }
}

// Ensure headers are loaded before any parsing is attempted
loadCsvHeaders();
