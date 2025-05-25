1.  **Project Setup & Plan Documentation:**
    *   Create a new directory named `flight_visualizer` in the root of the project.
    *   Create a markdown file named `visualization_plan.md` inside `flight_visualizer` and write down the detailed plan for the visualization tool. The plan will propose a Python-based desktop application using `pyserial` for data acquisition, `tkinter` for the GUI, `matplotlib` for 2D plots, and `PyVista` for 3D orientation.
2.  **Develop Basic Serial Communication & CSV Parsing:**
    *   In a Python script within `flight_visualizer` (e.g., `main_visualizer.py`):
        *   Implement functionality to list available serial ports.
        *   Allow the user to select a port and connect/disconnect.
        *   Start a thread to continuously read serial data when connected.
        *   Parse the incoming CSV data strings. The CSV headers will be based on `flight_console_data_mapping.json` or the `LOG_COLUMNS` definition from the firmware. Store a fixed number of recent data points for each metric.
        *   Initially, print parsed data to the console for verification.
3.  **Build GUI Shell & Integrate 2D Plotting:**
    *   Develop the main GUI window using `tkinter`.
    *   Add GUI elements for serial port selection, connect/disconnect button, and status indicators.
    *   Integrate `matplotlib` to create several empty plot areas.
    *   Implement functions to update these plots with the latest data received from the serial parser for key metrics like:
        *   Accelerometers (ICM & KX134)
        *   Gyroscopes (ICM)
        *   Magnetometers (ICM)
4.  **Implement 3D Orientation View:**
    *   Integrate `PyVista` into a dedicated area in the `tkinter` GUI.
    *   Create a simple 3D model (e.g., a representation of the rocket or a simple coordinate system axes).
    *   Use the quaternion data (q0, q1, q2, q3) from the parsed serial data to dynamically update the orientation of the 3D model in real-time.
5.  **Expand Plotting Capabilities & Refine GUI:**
    *   Add more 2D plots for the remaining relevant data:
        *   Temperatures (Barometer, ICM)
        *   Pressures & Altitudes (Barometer raw & calibrated)
        *   GPS data (Altitude, Speed, Heading)
        *   Euler angles (Roll, Pitch, Yaw)
        *   Guidance data (Target angles, PID integrals, Actuator outputs)
    *   Organize plots effectively (e.g., using tabs or a scrollable area if many).
    *   Implement basic error handling (e.g., serial port errors, data parsing issues).
    *   Add options to control plot history length or clear plots.
6.  **Testing and Documentation:**
    *   Thoroughly test the application with live data from the flight controller (ensure `enableSerialCSV` is active in firmware).
    *   Add comments to the Python code.
    *   Create a `README.md` for the `flight_visualizer` explaining how to set up and use the tool.
7.  **Submit the Visualization Tool:**
    *   Commit the `flight_visualizer` directory and its contents with a descriptive commit message.
