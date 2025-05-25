# TripleT Flight Visualizer

A Python application for real-time visualization of flight data streamed from the TripleT Flight Firmware.

## Features

*   Real-time 2D plots for various data categories:
    *   IMU (Accelerometers, Gyroscopes)
    *   High-G Accelerometer & Magnetometer
    *   Environmental (Temperatures, Pressure) & Altitude (Barometric, GPS)
    *   GPS (Speed, Heading, Satellites, pDOP)
    *   Euler Angles (Actual vs. Target)
    *   Actuator Outputs & PID Controller Integrals
*   Real-time 3D orientation view of the vehicle using quaternion data.
*   Serial port connectivity with dynamic port listing and connect/disconnect controls.
*   Configurable plot history length.
*   Button to clear current plot data.
*   Tabbed interface for organized plot display.

## Prerequisites

*   Python 3.8 or newer.
*   The following Python libraries:
    *   `pyserial`
    *   `matplotlib`
    *   `pyvista`
    *   `pyvistaqt`
    *   `numpy`
    *   `scipy`

## Setup

1.  **Install Dependencies:**
    Open your terminal or command prompt and run:
    ```bash
    pip install pyserial matplotlib pyvista pyvistaqt numpy scipy
    ```

2.  **Firmware Configuration:**
    Ensure the TripleT Flight Firmware on your flight controller has the `enableSerialCSV` option activated. This will enable the streaming of CSV-formatted data over the serial (USB) connection. The expected baud rate is typically 115200.

## Running the Tool

1.  Navigate to the root directory of this project in your terminal.
2.  Run the visualizer script using:
    ```bash
    python flight_visualizer/main_visualizer.py
    ```

## Using the Visualizer

1.  **Serial Connection:**
    *   Select the correct serial port for your flight controller from the dropdown menu.
    *   Click "Refresh Ports" if your port doesn't appear initially.
    *   Click "Connect". The status label should indicate a successful connection.
2.  **Viewing Data:**
    *   Once connected, data should start streaming and plots will update automatically.
    *   The 3D orientation view will show the vehicle's attitude.
    *   Navigate through the different data categories using the tabs above the 2D plots.
3.  **Controls:**
    *   **Plot History:** Adjust the spinbox to change the number of data points displayed on the 2D plots.
    *   **Clear Plots:** Click this button to clear all data currently displayed on the 2D plots. New data will continue to be plotted.
    *   **Disconnect:** Click to close the serial connection.

## Troubleshooting

*   **No Serial Ports Found:**
    *   Ensure your flight controller is plugged into the computer.
    *   Check if the necessary USB drivers for your flight controller are installed.
    *   Try clicking the "Refresh Ports" button.
*   **Connection Fails:**
    *   Verify the correct serial port is selected.
    *   Ensure no other application is currently using the same serial port.
    *   Check that the firmware is running and outputting data.
*   **Data Not Plotting / Incorrect Values:**
    *   Confirm `enableSerialCSV` is active in the firmware.
    *   Verify the firmware is sending data in the expected CSV format and baud rate (115200).
    *   Check the console output of `main_visualizer.py` for any error messages related to data parsing.
*   **3D View Issues:**
    *   Ensure PyVista and PyVistaQT are correctly installed.
    *   Check for any error messages in the console related to PyVista.
