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
let valPressure, valTemp, valAltBaro, valBattVoltage;
let currentFlightStateValueElement; // Added for flight state

/**
 * Initializes the UI components. This function is called by main.js
 * once the data parser has successfully loaded its configuration.
 */
function initUI() {
    console.log("UI Initializing...");
    
    // Initialize chart contexts
    const altitudeChartCtx = document.getElementById('altitudeChart')?.getContext('2d');
    const accelerationChartCtx = document.getElementById('accelerationChart')?.getContext('2d');
    const icmAccelChartCtx = document.getElementById('icmAccelChart')?.getContext('2d');
    const icmGyroChartCtx = document.getElementById('icmGyroChart')?.getContext('2d');
    const icmMagChartCtx = document.getElementById('icmMagChart')?.getContext('2d');
    const actuatorChartCtx = document.getElementById('actuatorChart')?.getContext('2d');

    if (altitudeChartCtx) altitudeChartInstance = initAltitudeChart(altitudeChartCtx);
    if (accelerationChartCtx) accelerationChartInstance = initAccelerationChart(accelerationChartCtx);
    if (icmAccelChartCtx) icmAccelChartInstance = initICMAccelChart(icmAccelChartCtx);
    if (icmGyroChartCtx) icmGyroChartInstance = initICMGyroChart(icmGyroChartCtx);
    if (icmMagChartCtx) icmMagChartInstance = initICMMagChart(icmMagChartCtx);
    if (actuatorChartCtx) actuatorChartInstance = initActuatorChart(actuatorChartCtx);

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
    valBattVoltage = document.getElementById('valBattVoltage');
    currentFlightStateValueElement = document.getElementById('currentFlightStateValue');

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
    if (parsedData.FlightState && currentFlightStateValueElement) {
        currentFlightStateValueElement.textContent = parsedData.FlightState;
    }

    if (parsedData.isStateUpdate) {
        return;
    }

    // --- Update Charts ---
    const timestamp = parsedData.Timestamp;
    const toDegrees = (radians) => (radians * 180 / Math.PI);

    if (altitudeChartInstance && parsedData.CalibratedAltitude !== undefined && parsedData.Alt !== undefined) {
        updateChart(altitudeChartInstance, { timestamp, values: [parseFloat(parsedData.CalibratedAltitude), parseFloat(parsedData.Alt)] });
    }
    if (accelerationChartInstance && parsedData.KX134_AccelX !== undefined) {
        updateChart(accelerationChartInstance, { timestamp, values: [parseFloat(parsedData.KX134_AccelX), parseFloat(parsedData.KX134_AccelY), parseFloat(parsedData.KX134_AccelZ)] });
    }
    if (icmAccelChartInstance && parsedData.ICM_AccelX !== undefined) {
        updateChart(icmAccelChartInstance, { timestamp, values: [parseFloat(parsedData.ICM_AccelX), parseFloat(parsedData.ICM_AccelY), parseFloat(parsedData.ICM_AccelZ)] });
    }
    if (icmGyroChartInstance && parsedData.ICM_GyroX !== undefined) {
        updateChart(icmGyroChartInstance, { timestamp, values: [toDegrees(parseFloat(parsedData.ICM_GyroX)), toDegrees(parseFloat(parsedData.ICM_GyroY)), toDegrees(parseFloat(parsedData.ICM_GyroZ))] });
    }
    if (icmMagChartInstance && parsedData.ICM_MagX !== undefined) {
        updateChart(icmMagChartInstance, { timestamp, values: [parseFloat(parsedData.ICM_MagX), parseFloat(parsedData.ICM_MagY), parseFloat(parsedData.ICM_MagZ)] });
    }
    if (actuatorChartInstance && parsedData.ActuatorOutRoll !== undefined) {
        updateChart(actuatorChartInstance, { timestamp, values: [parseFloat(parsedData.ActuatorOutRoll), parseFloat(parsedData.ActuatorOutPitch), parseFloat(parsedData.ActuatorOutYaw)] });
    }

    // --- Update Numerical Values ---
    const updateElementText = (element, value, defaultValue = 'N/A', precision = null) => {
        if (element) {
            let text = defaultValue;
            if (value !== undefined && value !== null && value !== '') {
                const numValue = parseFloat(value);
                text = precision !== null ? numValue.toFixed(precision) : numValue.toString();
            }
            element.textContent = text;
        }
    };

    updateElementText(valLat, parsedData.Lat, 'N/A', 6);
    updateElementText(valLong, parsedData.Long, 'N/A', 6);
    updateElementText(valSpeed, parsedData.Speed, 'N/A', 2);
    updateElementText(valSats, parsedData.Sats, 'N/A', 0);
    updateElementText(valAltGps, parsedData.Alt, 'N/A', 1);

    const rollRad = parseFloat(parsedData.EulerRoll_rad);
    const pitchRad = parseFloat(parsedData.EulerPitch_rad);
    const yawRad = parseFloat(parsedData.EulerYaw_rad);

    updateElementText(valRoll, !isNaN(rollRad) ? toDegrees(rollRad) : undefined, 'N/A', 2);
    updateElementText(valPitch, !isNaN(pitchRad) ? toDegrees(pitchRad) : undefined, 'N/A', 2);
    updateElementText(valYaw, !isNaN(yawRad) ? toDegrees(yawRad) : undefined, 'N/A', 2);

    // --- Update 3D Visualizer ---
    if (typeof update3DVisualizer === 'function' && !isNaN(rollRad)) {
        update3DVisualizer(
            toDegrees(rollRad),
            toDegrees(pitchRad),
            toDegrees(yawRad)
        );
    }
    
    updateElementText(valPressure, parsedData.Pressure, 'N/A', 2);
    updateElementText(valTemp, parsedData.Temperature, 'N/A', 1);
    updateElementText(valAltBaro, parsedData.CalibratedAltitude, 'N/A', 1);
    updateElementText(valBattVoltage, parsedData.BattVoltage, 'N/A', 2);
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
