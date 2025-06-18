// JavaScript file for updating the User Interface (UI) with new data

// Chart instances, will be initialized on DOMContentLoaded
let altitudeChartInstance = null;
let accelerationChartInstance = null;
let icmAccelChartInstance = null; // Added for ICM20948 Accel
let icmGyroChartInstance = null;  // Added for ICM20948 Gyro
let icmMagChartInstance = null;   // Added for ICM20948 Mag
let actuatorChartInstance = null; // Added for Actuator Control chart

// DOM elements for numerical data
let valLat, valLong, valSpeed, valSats, valAltGps;
let valRoll, valPitch, valYaw;
let valPressure, valTemp, valAltBaro;
let currentFlightStateValueElement; // Added for flight state

// Flight State Enum will be populated by initUI
let FlightState = { UNKNOWN: "UNKNOWN" };

/**
 * Initializes the UI components and flight state mappings.
 * This function is called by main.js after the data parser is ready.
 * @param {object} loadedFlightStates - The flight state mapping object from the JSON config.
 */
function initUI(loadedFlightStates) {
    FlightState = loadedFlightStates;
    console.log("UI Initialized with flight states:", FlightState);
    
    // The rest of the DOMContentLoaded logic can be moved here
    // Initialize chart contexts
    const altitudeChartCtx = document.getElementById('altitudeChart')?.getContext('2d');
    const accelerationChartCtx = document.getElementById('accelerationChart')?.getContext('2d');
    const icmAccelChartCtx = document.getElementById('icmAccelChart')?.getContext('2d'); // Added for ICM Accel
    const icmGyroChartCtx = document.getElementById('icmGyroChart')?.getContext('2d');   // Added for ICM Gyro
    const icmMagChartCtx = document.getElementById('icmMagChart')?.getContext('2d');     // Added for ICM Mag
    const actuatorChartCtx = document.getElementById('actuatorChart')?.getContext('2d'); // Added for Actuator chart

    if (altitudeChartCtx && typeof initAltitudeChart === 'function') {
        altitudeChartInstance = initAltitudeChart(altitudeChartCtx);
        console.log("Altitude chart initialized by ui_updater.");
    } else {
        console.error("Failed to initialize altitude chart: context or init function not found.");
    }

    if (accelerationChartCtx && typeof initAccelerationChart === 'function') {
        accelerationChartInstance = initAccelerationChart(accelerationChartCtx);
        console.log("Acceleration chart initialized by ui_updater.");
    } else {
        console.error("Failed to initialize acceleration chart: context or init function not found.");
    }

    // Initialize ICM20948 Charts
    if (icmAccelChartCtx && typeof initICMAccelChart === 'function') {
        icmAccelChartInstance = initICMAccelChart(icmAccelChartCtx);
        console.log("ICM20948 Acceleration chart initialized by ui_updater.");
    } else {
        console.error("Failed to initialize ICM20948 Acceleration chart: context or init function not found.");
    }

    if (icmGyroChartCtx && typeof initICMGyroChart === 'function') {
        icmGyroChartInstance = initICMGyroChart(icmGyroChartCtx);
        console.log("ICM20948 Gyroscope chart initialized by ui_updater.");
    } else {
        console.error("Failed to initialize ICM20948 Gyroscope chart: context or init function not found.");
    }

    if (icmMagChartCtx && typeof initICMMagChart === 'function') {
        icmMagChartInstance = initICMMagChart(icmMagChartCtx);
        console.log("ICM20948 Magnetometer chart initialized by ui_updater.");
    } else {
        console.error("Failed to initialize ICM20948 Magnetometer chart: context or init function not found.");
    }

    // Initialize Actuator Control Chart
    if (actuatorChartCtx && typeof initActuatorChart === 'function') {
        actuatorChartInstance = initActuatorChart(actuatorChartCtx);
        console.log("Actuator Control chart initialized by ui_updater.");
    } else {
        console.error("Failed to initialize Actuator Control chart: context or init function not found.");
    }

    // Get references to numerical display elements
    valLat = document.getElementById('valLat');
    valLong = document.getElementById('valLong');
    valSpeed = document.getElementById('valSpeed');
    valSats = document.getElementById('valSats');
    valAltGps = document.getElementById('valAltGps');

    valRoll = document.getElementById('valRoll');
    valPitch = document.getElementById('valPitch');
    valYaw = document.getElementById('valYaw');
    
    valPressure = document.getElementById('valPressure');
    valTemp = document.getElementById('valTemp');
    valAltBaro = document.getElementById('valAltBaro');
    currentFlightStateValueElement = document.getElementById('currentFlightStateValue'); // Added for flight state

    // Initialize 3D Visualizer
    if (typeof init3DVisualizer === 'function') {
        init3DVisualizer();
        console.log("3D Visualizer initialized by ui_updater.");
    } else {
        console.error("Failed to initialize 3D Visualizer: init3DVisualizer function not found.");
    }
}

/**
 * Converts a flight state number to its string representation.
 * @param {number} stateNumber - The flight state number from the firmware.
 * @returns {string} The string representation of the flight state.
 */
function getFlightStateString(stateNumber) {
    return FlightState[stateNumber] || FlightState.UNKNOWN;
}

document.addEventListener('DOMContentLoaded', () => {
    // This event listener is now effectively replaced by the initUI function,
    // which is called from main.js once the parser and configs are ready.
    // We can leave it empty or remove it if all initialization logic has been moved.
});

/**
 * Updates the UI with parsed data from the flight controller.
 * @param {object} parsedData - The structured JavaScript object from data_parser.js.
 */
function updateUI(parsedData) {
    // Helper to convert radians to degrees
    const toDegrees = (radians) => (radians * 180 / Math.PI);

    if (!parsedData) {
        console.warn("updateUI called with no data.");
        return;
    }

    // --- Update Flight State Display ---
    if (parsedData.FlightState !== undefined && currentFlightStateValueElement) {
        currentFlightStateValueElement.textContent = getFlightStateString(parsedData.FlightState);
    }

    // --- Update Charts ---
    const timestamp = parsedData.Timestamp; // Assuming Timestamp is available and numeric

    // Update Altitude Chart (now with Calibrated and GPS altitude)
    if (altitudeChartInstance && 
        parsedData.CalibratedAltitude !== undefined && 
        parsedData.Alt !== undefined &&  // Alt is GPS altitude
        timestamp !== undefined) {
        updateChart(altitudeChartInstance, { 
            timestamp: timestamp, 
            values: [parsedData.CalibratedAltitude, parsedData.Alt] 
        });
    }

    // Update KX134 Acceleration Chart
    if (accelerationChartInstance && 
        parsedData.KX134_AccelX !== undefined && 
        parsedData.KX134_AccelY !== undefined && 
        parsedData.KX134_AccelZ !== undefined && 
        timestamp !== undefined) {
        updateChart(accelerationChartInstance, { 
            timestamp: timestamp, 
            values: [parsedData.KX134_AccelX, parsedData.KX134_AccelY, parsedData.KX134_AccelZ] 
        });
    }

    // Update ICM20948 Acceleration Chart
    if (icmAccelChartInstance && 
        parsedData.ICM_AccelX !== undefined && 
        parsedData.ICM_AccelY !== undefined && 
        parsedData.ICM_AccelZ !== undefined && 
        timestamp !== undefined) {
        updateChart(icmAccelChartInstance, { 
            timestamp: timestamp, 
            values: [parsedData.ICM_AccelX, parsedData.ICM_AccelY, parsedData.ICM_AccelZ] 
        });
    }

    // Update ICM20948 Gyroscope Chart (convert rad/s to deg/s)
    if (icmGyroChartInstance && 
        parsedData.ICM_GyroX !== undefined && 
        parsedData.ICM_GyroY !== undefined && 
        parsedData.ICM_GyroZ !== undefined && 
        timestamp !== undefined) {
        updateChart(icmGyroChartInstance, { 
            timestamp: timestamp, 
            values: [
                toDegrees(parsedData.ICM_GyroX), 
                toDegrees(parsedData.ICM_GyroY), 
                toDegrees(parsedData.ICM_GyroZ)
            ]
        });
    }

    // Update ICM20948 Magnetometer Chart
    if (icmMagChartInstance && 
        parsedData.ICM_MagX !== undefined && 
        parsedData.ICM_MagY !== undefined && 
        parsedData.ICM_MagZ !== undefined && 
        timestamp !== undefined) {
        updateChart(icmMagChartInstance, { 
            timestamp: timestamp, 
            values: [parsedData.ICM_MagX, parsedData.ICM_MagY, parsedData.ICM_MagZ] 
        });
    }

    // Update Actuator Control Chart (placeholder - assuming Actuator1, Actuator2 fields)
    if (actuatorChartInstance && timestamp !== undefined) {
        // Use placeholder values if actual data fields aren't available yet
        const actuator1Value = parsedData.Actuator1 !== undefined ? parsedData.Actuator1 : 0; 
        const actuator2Value = parsedData.Actuator2 !== undefined ? parsedData.Actuator2 : 0;
        updateChart(actuatorChartInstance, { 
            timestamp: timestamp, 
            values: [actuator1Value, actuator2Value] 
        });
    }

    // --- Update Numerical Values ---
    // Helper to update text content and handle undefined or null values
    const updateElementText = (element, value, defaultValue = 'N/A', precision = null) => {
        if (element) {
            if (value !== undefined && value !== null) {
                element.textContent = precision !== null && typeof value === 'number' ? value.toFixed(precision) : value;
            } else {
                element.textContent = defaultValue;
            }
        }
    };

    // GPS Data
    updateElementText(valLat, parsedData.Lat, 'N/A', 6);
    updateElementText(valLong, parsedData.Long, 'N/A', 6);
    updateElementText(valSpeed, parsedData.Speed, 'N/A', 2); // Assuming speed in knots
    updateElementText(valSats, parsedData.Sats, 'N/A', 0);
    updateElementText(valAltGps, parsedData.Alt, 'N/A', 1); // GPS Altitude (Alt)

    // Orientation Data (Euler angles in radians, convert to degrees)
    updateElementText(valRoll, parsedData.EulerRoll_rad !== undefined ? toDegrees(parsedData.EulerRoll_rad) : undefined, 'N/A', 2);
    updateElementText(valPitch, parsedData.EulerPitch_rad !== undefined ? toDegrees(parsedData.EulerPitch_rad) : undefined, 'N/A', 2);
    updateElementText(valYaw, parsedData.EulerYaw_rad !== undefined ? toDegrees(parsedData.EulerYaw_rad) : undefined, 'N/A', 2);

    // --- Update 3D Visualizer ---
    if (typeof update3DVisualizer === 'function' && 
        parsedData.EulerRoll_rad !== undefined && 
        parsedData.EulerPitch_rad !== undefined && 
        parsedData.EulerYaw_rad !== undefined) {
        update3DVisualizer(
            toDegrees(parsedData.EulerRoll_rad),
            toDegrees(parsedData.EulerPitch_rad),
            toDegrees(parsedData.EulerYaw_rad)
        );
    }

    // Sensor Data
    updateElementText(valPressure, parsedData.Pressure, 'N/A', 2); // Pressure in hPa
    updateElementText(valTemp, parsedData.Temperature, 'N/A', 1); // Temperature in Celsius
    updateElementText(valAltBaro, parsedData.CalibratedAltitude, 'N/A', 1); // Barometric Altitude (CalibratedAltitude)
}

// Example of how main.js might call this after parsing data:
// const sampleData = {
//     Timestamp: Date.now(),
//     CalibratedAltitude: 100 + Math.random() * 10,
//     KX134_AccelX: Math.random() * 2 - 1,
//     KX134_AccelY: Math.random() * 2 - 1,
//     KX134_AccelZ: 9.8 + Math.random() * 0.5,
//     Lat: 34.052235 + (Math.random() - 0.5) * 0.01,
//     Long: -118.243683 + (Math.random() - 0.5) * 0.01,
//     Speed: 25 + Math.random() * 5,
//     Sats: 10 + Math.floor(Math.random() * 3),
//     Alt: 160 + Math.random() * 10, // GPS Altitude
//     EulerRoll_rad: (Math.random() - 0.5) * Math.PI / 4,
//     EulerPitch_rad: (Math.random() - 0.5) * Math.PI / 8,
//     EulerYaw_rad: Math.random() * 2 * Math.PI,
//     Pressure: 1013.25 + (Math.random() - 0.5) * 10,
//     Temperature: 20 + (Math.random() - 0.5) * 5
// };
// updateUI(sampleData); // This would be called from main.js typically
// setTimeout(() => updateUI(sampleData), 2000); // For testing
// setTimeout(() => updateUI({...sampleData, Timestamp: Date.now(), CalibratedAltitude: 100 + Math.random() * 10}), 3000); // For testing
// setTimeout(() => updateUI({...sampleData, Timestamp: Date.now(), CalibratedAltitude: 100 + Math.random() * 10}), 4000); // For testing
