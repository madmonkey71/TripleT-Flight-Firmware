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

/**
 * Initializes the UI components and flight state mappings.
 * This function is called by main.js after the data parser is ready.
 */
function initUI() {
    // FlightState is now managed by the data_parser module.
    console.log("UI Initializing...");
    
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
    }
    if (accelerationChartCtx && typeof initAccelerationChart === 'function') {
        accelerationChartInstance = initAccelerationChart(accelerationChartCtx);
    }
    if (icmAccelChartCtx && typeof initICMAccelChart === 'function') {
        icmAccelChartInstance = initICMAccelChart(icmAccelChartCtx);
    }
    if (icmGyroChartCtx && typeof initICMGyroChart === 'function') {
        icmGyroChartInstance = initICMGyroChart(icmGyroChartCtx);
    }
    if (icmMagChartCtx && typeof initICMMagChart === 'function') {
        icmMagChartInstance = initICMMagChart(icmMagChartCtx);
    }
    if (actuatorChartCtx && typeof initActuatorChart === 'function') {
        actuatorChartInstance = initActuatorChart(actuatorChartCtx);
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
    currentFlightStateValueElement = document.getElementById('currentFlightStateValue');

    // Initialize 3D Visualizer
    if (typeof init3DVisualizer === 'function') {
        init3DVisualizer();
    }
    console.log("UI Initialized.");
}

/**
 * Updates the UI with parsed data from the flight controller.
 * @param {object} parsedData - The structured JavaScript object from data_parser.js.
 */
function updateUI(parsedData) {
    if (!parsedData) return;

    // --- Update Flight State Display ---
    // The parser now consistently provides the state name in the FlightState field.
    if (parsedData.FlightState && currentFlightStateValueElement) {
        currentFlightStateValueElement.textContent = parsedData.FlightState;
    }

    // If this is a state-only update, do not process other fields.
    if (parsedData.isStateUpdate) {
        return;
    }

    // --- Update Charts ---
    const timestamp = parsedData.Timestamp;
    const toDegrees = (radians) => (radians * 180 / Math.PI);

    if (altitudeChartInstance && parsedData.CalibratedAltitude !== undefined && parsedData.Alt !== undefined) {
        updateChart(altitudeChartInstance, { timestamp, values: [parsedData.CalibratedAltitude, parsedData.Alt] });
    }
    if (accelerationChartInstance && parsedData.KX134_AccelX !== undefined) {
        updateChart(accelerationChartInstance, { timestamp, values: [parsedData.KX134_AccelX, parsedData.KX134_AccelY, parsedData.KX134_AccelZ] });
    }
    if (icmAccelChartInstance && parsedData.ICM_AccelX !== undefined) {
        updateChart(icmAccelChartInstance, { timestamp, values: [parsedData.ICM_AccelX, parsedData.ICM_AccelY, parsedData.ICM_AccelZ] });
    }
    if (icmGyroChartInstance && parsedData.ICM_GyroX !== undefined) {
        updateChart(icmGyroChartInstance, { timestamp, values: [toDegrees(parsedData.ICM_GyroX), toDegrees(parsedData.ICM_GyroY), toDegrees(parsedData.ICM_GyroZ)] });
    }
    if (icmMagChartInstance && parsedData.ICM_MagX !== undefined) {
        updateChart(icmMagChartInstance, { timestamp, values: [parsedData.ICM_MagX, parsedData.ICM_MagY, parsedData.ICM_MagZ] });
    }
    if (actuatorChartInstance && parsedData.ActuatorOutRoll !== undefined) {
        updateChart(actuatorChartInstance, { timestamp, values: [parsedData.ActuatorOutRoll, parsedData.ActuatorOutPitch, parsedData.ActuatorOutYaw] });
    }

    // --- Update Numerical Values ---
    const updateElementText = (element, value, defaultValue = 'N/A', precision = null) => {
        if (element) {
            let text = defaultValue;
            if (value !== undefined && value !== null) {
                text = precision !== null && typeof value === 'number' ? Number(value).toFixed(precision) : value;
            }
            element.textContent = text;
        }
    };

    updateElementText(valLat, parsedData.Lat, 'N/A', 6);
    updateElementText(valLong, parsedData.Long, 'N/A', 6);
    updateElementText(valSpeed, parsedData.Speed, 'N/A', 2);
    updateElementText(valSats, parsedData.Sats, 'N/A', 0);
    updateElementText(valAltGps, parsedData.Alt, 'N/A', 1);

    updateElementText(valRoll, parsedData.EulerRoll_rad !== undefined ? toDegrees(parsedData.EulerRoll_rad) : undefined, 'N/A', 2);
    updateElementText(valPitch, parsedData.EulerPitch_rad !== undefined ? toDegrees(parsedData.EulerPitch_rad) : undefined, 'N/A', 2);
    updateElementText(valYaw, parsedData.EulerYaw_rad !== undefined ? toDegrees(parsedData.EulerYaw_rad) : undefined, 'N/A', 2);

    // --- Update 3D Visualizer ---
    if (typeof update3DVisualizer === 'function' && parsedData.EulerRoll_rad !== undefined) {
        update3DVisualizer(
            toDegrees(parsedData.EulerRoll_rad),
            toDegrees(parsedData.EulerPitch_rad),
            toDegrees(parsedData.EulerYaw_rad)
        );
    }
    
    // Barometer and other data
    updateElementText(valPressure, parsedData.Pressure, 'N/A', 2);
    updateElementText(valTemp, parsedData.Temperature, 'N/A', 1);
    updateElementText(valAltBaro, parsedData.CalibratedAltitude, 'N/A', 1);
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
