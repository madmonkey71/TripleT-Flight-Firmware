// JavaScript file for data visualizations (charts, 3D models, etc.)

const MAX_DATA_POINTS = 100; // Max data points to show on charts

/**
 * Initializes a new Chart.js line chart for altitude.
 * @param {CanvasRenderingContext2D} ctx - The canvas 2D rendering context.
 * @returns {Chart} The Chart.js instance.
 */
function initAltitudeChart(ctx) {
    console.log("Initializing Altitude Chart");
    return new Chart(ctx, {
        type: 'line',
        data: {
            labels: [], // Timestamps
            datasets: [{
                label: 'Calibrated Altitude (m)',
                data: [], // Altitude values
                borderColor: 'rgb(75, 192, 192)',
                backgroundColor: 'rgba(75, 192, 192, 0.5)',
                tension: 0.1,
                pointRadius: 1,
                borderWidth: 1
            },{
                label: 'GPS Altitude (m)',
                data: [], // GPS Altitude values
                borderColor: 'rgb(255, 159, 64)', // Orange variant
                backgroundColor: 'rgba(255, 159, 64, 0.5)',
                tension: 0.1,
                pointRadius: 1,
                borderWidth: 1
            }]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            scales: {
                x: {
                    title: {
                        display: true,
                        text: 'Timestamp (s)'
                    }
                },
                y: {
                    title: {
                        display: true,
                        text: 'Altitude (m)'
                    }
                }
            }
        }
    });
}

/**
 * Initializes a new Chart.js line chart for acceleration.
 * @param {CanvasRenderingContext2D} ctx - The canvas 2D rendering context.
 * @returns {Chart} The Chart.js instance.
 */
function initAccelerationChart(ctx) {
    console.log("Initializing Acceleration Chart");
    return new Chart(ctx, {
        type: 'line',
        data: {
            labels: [], // Timestamps
            datasets: [
                {
                    label: 'AccelX (G)',
                    data: [],
                    borderColor: 'rgb(255, 99, 132)',
                    backgroundColor: 'rgba(255, 99, 132, 0.5)',
                    tension: 0.1,
                    pointRadius: 1,
                    borderWidth: 1
                },
                {
                    label: 'AccelY (G)',
                    data: [],
                    borderColor: 'rgb(54, 162, 235)',
                    backgroundColor: 'rgba(54, 162, 235, 0.5)',
                    tension: 0.1,
                    pointRadius: 1,
                    borderWidth: 1
                },
                {
                    label: 'AccelZ (G)',
                    data: [],
                    borderColor: 'rgb(255, 206, 86)',
                    backgroundColor: 'rgba(255, 206, 86, 0.5)',
                    tension: 0.1,
                    pointRadius: 1,
                    borderWidth: 1
                }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            scales: {
                x: {
                    title: {
                        display: true,
                        text: 'Timestamp (s)'
                    }
                },
                y: {
                    title: {
                        display: true,
                        text: 'Acceleration (G)'
                    }
                }
            }
        }
    });
}

/**
 * Initializes a new Chart.js line chart for ICM20948 acceleration.
 * @param {CanvasRenderingContext2D} ctx - The canvas 2D rendering context.
 * @returns {Chart} The Chart.js instance.
 */
function initICMAccelChart(ctx) {
    console.log("Initializing ICM20948 Acceleration Chart");
    return new Chart(ctx, {
        type: 'line',
        data: {
            labels: [], // Timestamps
            datasets: [
                {
                    label: 'ICM AccelX (G)',
                    data: [],
                    borderColor: 'rgb(255, 99, 71)', // Tomato
                    backgroundColor: 'rgba(255, 99, 71, 0.5)',
                    tension: 0.1,
                    pointRadius: 1,
                    borderWidth: 1
                },
                {
                    label: 'ICM AccelY (G)',
                    data: [],
                    borderColor: 'rgb(60, 179, 113)', // MediumSeaGreen
                    backgroundColor: 'rgba(60, 179, 113, 0.5)',
                    tension: 0.1,
                    pointRadius: 1,
                    borderWidth: 1
                },
                {
                    label: 'ICM AccelZ (G)',
                    data: [],
                    borderColor: 'rgb(106, 90, 205)', // SlateBlue
                    backgroundColor: 'rgba(106, 90, 205, 0.5)',
                    tension: 0.1,
                    pointRadius: 1,
                    borderWidth: 1
                }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            scales: {
                x: {
                    title: {
                        display: true,
                        text: 'Timestamp (s)'
                    }
                },
                y: {
                    title: {
                        display: true,
                        text: 'Acceleration (G)'
                    }
                }
            }
        }
    });
}

/**
 * Initializes a new Chart.js line chart for ICM20948 gyroscope.
 * @param {CanvasRenderingContext2D} ctx - The canvas 2D rendering context.
 * @returns {Chart} The Chart.js instance.
 */
function initICMGyroChart(ctx) {
    console.log("Initializing ICM20948 Gyroscope Chart");
    return new Chart(ctx, {
        type: 'line',
        data: {
            labels: [], // Timestamps
            datasets: [
                {
                    label: 'ICM GyroX (deg/s)',
                    data: [],
                    borderColor: 'rgb(255, 165, 0)', // Orange
                    backgroundColor: 'rgba(255, 165, 0, 0.5)',
                    tension: 0.1,
                    pointRadius: 1,
                    borderWidth: 1
                },
                {
                    label: 'ICM GyroY (deg/s)',
                    data: [],
                    borderColor: 'rgb(72, 209, 204)', // MediumTurquoise
                    backgroundColor: 'rgba(72, 209, 204, 0.5)',
                    tension: 0.1,
                    pointRadius: 1,
                    borderWidth: 1
                },
                {
                    label: 'ICM GyroZ (deg/s)',
                    data: [],
                    borderColor: 'rgb(218, 112, 214)', // Orchid
                    backgroundColor: 'rgba(218, 112, 214, 0.5)',
                    tension: 0.1,
                    pointRadius: 1,
                    borderWidth: 1
                }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            scales: {
                x: {
                    title: {
                        display: true,
                        text: 'Timestamp (s)'
                    }
                },
                y: {
                    title: {
                        display: true,
                        text: 'Angular Velocity (deg/s)'
                    }
                }
            }
        }
    });
}

/**
 * Initializes a new Chart.js line chart for ICM20948 magnetometer.
 * @param {CanvasRenderingContext2D} ctx - The canvas 2D rendering context.
 * @returns {Chart} The Chart.js instance.
 */
function initICMMagChart(ctx) {
    console.log("Initializing ICM20948 Magnetometer Chart");
    return new Chart(ctx, {
        type: 'line',
        data: {
            labels: [], // Timestamps
            datasets: [
                {
                    label: 'ICM MagX (uT)',
                    data: [],
                    borderColor: 'rgb(139, 69, 19)', // SaddleBrown
                    backgroundColor: 'rgba(139, 69, 19, 0.5)',
                    tension: 0.1,
                    pointRadius: 1,
                    borderWidth: 1
                },
                {
                    label: 'ICM MagY (uT)',
                    data: [],
                    borderColor: 'rgb(128, 128, 0)', // Olive
                    backgroundColor: 'rgba(128, 128, 0, 0.5)',
                    tension: 0.1,
                    pointRadius: 1,
                    borderWidth: 1
                },
                {
                    label: 'ICM MagZ (uT)',
                    data: [],
                    borderColor: 'rgb(0, 128, 128)', // Teal
                    backgroundColor: 'rgba(0, 128, 128, 0.5)',
                    tension: 0.1,
                    pointRadius: 1,
                    borderWidth: 1
                }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            scales: {
                x: {
                    title: {
                        display: true,
                        text: 'Timestamp (s)'
                    }
                },
                y: {
                    title: {
                        display: true,
                        text: 'Magnetic Field (uT)'
                    }
                }
            }
        }
    });
}

/**
 * Initializes a new Chart.js line chart for Actuator Control.
 * @param {CanvasRenderingContext2D} ctx - The canvas 2D rendering context.
 * @returns {Chart} The Chart.js instance.
 */
function initActuatorChart(ctx) {
    console.log("Initializing Actuator Control Chart");
    return new Chart(ctx, {
        type: 'line',
        data: {
            labels: [], // Timestamps
            datasets: [
                {
                    label: 'Roll Actuator',
                    data: [], 
                    borderColor: 'rgb(255, 99, 132)', // Red
                    backgroundColor: 'rgba(255, 99, 132, 0.5)',
                    tension: 0.1,
                    pointRadius: 1,
                    borderWidth: 1
                },
                {
                    label: 'Pitch Actuator',
                    data: [], 
                    borderColor: 'rgb(54, 162, 235)', // Blue
                    backgroundColor: 'rgba(54, 162, 235, 0.5)',
                    tension: 0.1,
                    pointRadius: 1,
                    borderWidth: 1
                },
                {
                    label: 'Yaw Actuator',
                    data: [], 
                    borderColor: 'rgb(75, 192, 192)', // Green
                    backgroundColor: 'rgba(75, 192, 192, 0.5)',
                    tension: 0.1,
                    pointRadius: 1,
                    borderWidth: 1
                }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            scales: {
                x: {
                    title: {
                        display: true,
                        text: 'Timestamp (s)'
                    }
                },
                y: {
                    title: {
                        display: true,
                        text: 'Actuator Command'
                    },
                    // Allow for both normalized outputs (-1 to 1) and servo angles (0 to 180)
                    suggestedMin: -1,
                    suggestedMax: 180
                }
            },
            plugins: {
                legend: {
                    display: true
                },
                tooltip: {
                    mode: 'index',
                    intersect: false
                }
            }
        }
    });
}

/**
 * Updates a Chart.js instance with new data.
 * @param {Chart} chartInstance - The Chart.js instance to update.
 * @param {object} newDataPoint - Data point. 
 * For single-line charts: { timestamp: number, values: [value1] }
 * For multi-line (e.g., XYZ accel) charts: { timestamp: number, values: [x, y, z] }
 */
function updateChart(chartInstance, newDataPoint) {
    if (!chartInstance || !newDataPoint || !newDataPoint.values) return;

    const chartLabels = chartInstance.data.labels;
    const datasets = chartInstance.data.datasets;

    // Add new timestamp label
    // Convert timestamp to seconds for readability if it's in milliseconds
    const displayTimestamp = newDataPoint.timestamp / 1000; 
    chartLabels.push(displayTimestamp.toFixed(1));

    // Add new data points to each dataset
    if (datasets.length === newDataPoint.values.length) {
        datasets.forEach((dataset, index) => { // Iterate through each dataset
            dataset.data.push(newDataPoint.values[index]); // Push the corresponding value
        });
    } else {
        console.warn('Mismatch between number of datasets and number of new data values', datasets.length, newDataPoint.values.length);
        // As a fallback, or if only one value provided for a multi-dataset chart (e.g. old single value altitude)
        // this might be a bit naive and could be improved if specific charts need specific handling.
        if (datasets.length > 0 && newDataPoint.values.length === 1) { // If only one value given, apply to first dataset.
            datasets[0].data.push(newDataPoint.values[0]);
        } else if (datasets.length === 1 && newDataPoint.values.length > 0) { // If one dataset and multiple values, use first value.
             datasets[0].data.push(newDataPoint.values[0]);
        } // Otherwise, do nothing to prevent errors if counts don't match
    }

    // Limit the number of data points
    if (chartLabels.length > MAX_DATA_POINTS) {
        chartLabels.shift(); // Remove the oldest label
        datasets.forEach(dataset => {
            dataset.data.shift(); // Remove the oldest data point
        });
    }

    chartInstance.update('none');
}
