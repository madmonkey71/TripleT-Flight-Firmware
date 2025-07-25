// JavaScript file for parsing incoming data from the serial port

// This module will hold the configuration once loaded.
let config = {
    csvHeaders: [],
    flightStateMap: {}
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
    
    // Check if it starts with a number (sequence number)
    if (!/^\d+/.test(trimmedMessage)) {
        return false;
    }
    
    // Count comma-separated fields
    const fields = trimmedMessage.split(',');
    
    // Check for reasonable field count (CSV data should have many fields)
    if (fields.length < 5) {
        return false;
    }
    
    // Very lenient numeric check - just check if first field is a number
    const firstField = fields[0].trim();
    if (!/^\d+$/.test(firstField)) {
        return false;
    }
    
    return true;
}

/**
 * Parses servo command messages from the firmware debug output.
 * @param {string} message - The message to parse
 * @returns {object|null} Parsed servo data or null if not a servo command
 */
function parseServoCommand(message) {
    // Pattern: "Servo CMDs (P,R,Y): 66, 86, 81"
    const servoPattern = /^Servo CMDs \(P,R,Y\):\s*(\d+),\s*(\d+),\s*(\d+)$/;
    const match = message.match(servoPattern);
    
    if (match) {
        return {
            isServoCommand: true,
            timestamp: Date.now(), // Use current time since servo commands don't have timestamps
            pitchServo: parseInt(match[1]),
            rollServo: parseInt(match[2]),
            yawServo: parseInt(match[3])
        };
    }
    
    return null;
}

/**
 * Categorizes a message type for appropriate handling.
 * @param {string} message - The incoming message to categorize
 * @returns {string} Message category: 'csv_data', 'servo_command', 'info', 'system', 'unknown'
 */
function categorizeMessage(message) {
    if (!message || typeof message !== 'string') {
        return 'unknown';
    }
    
    if (isCSVDataMessage(message)) {
        return 'csv_data';
    }
    
    // Check for servo command messages
    if (parseServoCommand(message)) {
        return 'servo_command';
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
        
        console.log("Data parser initialized successfully.");

    } catch (error) {
        console.error("Failed to initialize data parser:", error);
        // Re-throw the error so the main application knows initialization failed.
        throw error; 
    }
}

/**
 * Parses a single line of text from the serial port.
 * It can handle both full CSV lines, servo command messages, and special JSON state messages.
 * @param {string} line - The raw line of text from the serial port.
 * @returns {object|null} A structured data object or null if the line is not valid data.
 */
function parseData(line) {
    const trimmedLine = line.trim();

    // 1. Handle JSON state messages: {"state_id":3,"state_name":"ARMED"}
    if (trimmedLine.startsWith('{') && trimmedLine.endsWith('}')) {
        try {
            const stateData = JSON.parse(trimmedLine);
            if (stateData && typeof stateData.state_name !== 'undefined') {
                return { 
                    isStateUpdate: true,
                    FlightState: stateData.state_name 
                };
            }
        } catch (e) {
            return null;
        }
    }

    // 2. Handle servo command messages: "Servo CMDs (P,R,Y): 66, 86, 81"
    const servoData = parseServoCommand(trimmedLine);
    if (servoData) {
        return servoData;
    }

    // 3. Handle CSV data lines from firmware, which do not have a "CSV:" prefix.
    // We identify them by checking if the first part is a number (sequence number).
    const parts = trimmedLine.split(',');
    if (parts.length > 1 && !isNaN(parts[0])) {
        // Ensure headers are loaded and the data has the correct number of fields
        if (config.csvHeaders.length === 0 || parts.length !== config.csvHeaders.length) {
            console.warn(`CSV mismatch: Expected ${config.csvHeaders.length}, got ${parts.length}.`);
            return null;
        }

        const dataObject = {};
        config.csvHeaders.forEach((header, index) => {
            dataObject[header] = parts[index] ? parts[index].trim() : '';
        });

        if (dataObject.FlightState !== undefined) {
            dataObject.FlightState = config.flightStateMap[dataObject.FlightState] || 'UNKNOWN';
        }
        
        return dataObject;
    }

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
