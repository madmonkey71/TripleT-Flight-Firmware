// JavaScript file for updating the User Interface (UI) with new data

// Chart instances, will be initialized on DOMContentLoaded
let altitudeChartInstance = null;
let accelerationChartInstance = null;

// DOM elements for numerical data
let valLat, valLong, valSpeed, valSats, valAltGps;
let valRoll, valPitch, valYaw;
let valPressure, valTemp, valAltBaro;

document.addEventListener('DOMContentLoaded', () => {
    // Initialize chart contexts
    const altitudeChartCtx = document.getElementById('altitudeChart')?.getContext('2d');
    const accelerationChartCtx = document.getElementById('accelerationChart')?.getContext('2d');

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
});

/**
 * Updates the UI with parsed data from the flight controller.
 * @param {object} parsedData - The structured JavaScript object from data_parser.js.
 */
function updateUI(parsedData) {
    if (!parsedData) {
        console.warn("updateUI called with no data.");
        return;
    }

    // --- Update Charts ---
    const timestamp = parsedData.Timestamp; // Assuming Timestamp is available and numeric

    if (altitudeChartInstance && parsedData.CalibratedAltitude !== undefined && timestamp !== undefined) {
        updateChart(altitudeChartInstance, { timestamp: timestamp, value: parsedData.CalibratedAltitude });
    }

    if (accelerationChartInstance && 
        parsedData.KX134_AccelX !== undefined && 
        parsedData.KX134_AccelY !== undefined && 
        parsedData.KX134_AccelZ !== undefined && 
        timestamp !== undefined) {
        updateChart(accelerationChartInstance, { 
            timestamp: timestamp, 
            x: parsedData.KX134_AccelX, 
            y: parsedData.KX134_AccelY, 
            z: parsedData.KX134_AccelZ 
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
    const toDegrees = (radians) => (radians * 180 / Math.PI);
    updateElementText(valRoll, parsedData.EulerRoll_rad !== undefined ? toDegrees(parsedData.EulerRoll_rad) : undefined, 'N/A', 2);
    updateElementText(valPitch, parsedData.EulerPitch_rad !== undefined ? toDegrees(parsedData.EulerPitch_rad) : undefined, 'N/A', 2);
    updateElementText(valYaw, parsedData.EulerYaw_rad !== undefined ? toDegrees(parsedData.EulerYaw_rad) : undefined, 'N/A', 2);

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
