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
                tension: 0.1
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
                    tension: 0.1
                },
                {
                    label: 'AccelY (G)',
                    data: [],
                    borderColor: 'rgb(54, 162, 235)',
                    backgroundColor: 'rgba(54, 162, 235, 0.5)',
                    tension: 0.1
                },
                {
                    label: 'AccelZ (G)',
                    data: [],
                    borderColor: 'rgb(255, 206, 86)',
                    backgroundColor: 'rgba(255, 206, 86, 0.5)',
                    tension: 0.1
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
 * Updates a Chart.js instance with new data.
 * @param {Chart} chartInstance - The Chart.js instance to update.
 * @param {object} newDataPoint - Data point. 
 * For single-line charts: { timestamp: number, value: number }
 * For multi-line (e.g., XYZ accel) charts: { timestamp: number, x: number, y: number, z: number }
 */
function updateChart(chartInstance, newDataPoint) {
    if (!chartInstance || !newDataPoint) return;

    const chartLabels = chartInstance.data.labels;
    const datasets = chartInstance.data.datasets;

    // Add new timestamp label
    // Convert timestamp to seconds for readability if it's in milliseconds
    const displayTimestamp = newDataPoint.timestamp / 1000; 
    chartLabels.push(displayTimestamp.toFixed(1));


    if (datasets.length === 1) { // Single dataset (e.g., altitude)
        datasets[0].data.push(newDataPoint.value);
    } else if (datasets.length === 3) { // Multiple datasets (e.g., acceleration X, Y, Z)
        datasets[0].data.push(newDataPoint.x);
        datasets[1].data.push(newDataPoint.y);
        datasets[2].data.push(newDataPoint.z);
    }

    // Limit the number of data points
    if (chartLabels.length > MAX_DATA_POINTS) {
        chartLabels.shift(); // Remove the oldest label
        datasets.forEach(dataset => {
            dataset.data.shift(); // Remove the oldest data point
        });
    }

    chartInstance.update();
}
