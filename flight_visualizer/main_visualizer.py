# Standard library imports
import serial
import serial.tools.list_ports
import threading
import time
import csv
import collections # For collections.deque
import pprint # For pretty printing data, useful in debugging

# GUI and plotting libraries
import tkinter as tk
from tkinter import ttk, messagebox # Themed Tkinter widgets and standard dialogs
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk # Matplotlib Tkinter embedding

# Numerical and 3D libraries
import numpy as np
import pyvista as pv
from pyvistaqt import BackgroundPlotter # PyVista integration with Qt backend (needed for Tkinter)
from scipy.spatial.transform import Rotation as R_scipy # For quaternion to rotation matrix conversion


# --- Global Variables ---
# These variables are accessed by both the serial reading thread and the main GUI thread.
# Care must be taken when modifying them to ensure thread safety, though Python's GIL
# often simplifies this for basic types and operations.

serial_connection = None  # Holds the active serial.Serial object when connected.
data_queue = collections.deque(maxlen=1000)  # A thread-safe double-ended queue to store recent data rows (dictionaries).
                                             # `maxlen` ensures it doesn't grow indefinitely.
reader_thread = None  # Holds the threading.Thread object for the serial reader.
stop_thread_flag = threading.Event()  # A threading.Event to signal the serial reader thread to stop.

COLUMN_HEADERS = []  # List to store the CSV column headers once received from the serial stream.
EXPECTED_COLUMN_COUNT = 0  # Number of columns expected based on the received headers.
PRINT_RAW_SERIAL = False  # Debug flag: Set to True to print all raw lines received from serial.
# PLOT_HISTORY_SIZE is now a tk.IntVar in the FlightVisualizerApp class.


# --- Data Type Mapping ---
# This dictionary maps CSV column headers to their expected Python data types (int, float, or str).
# It's crucial for correctly parsing and converting incoming serial data strings.
# The `_attempt_type_conversion` function uses this map. If a key is not found,
# it currently defaults to 'float', which is a common case for sensor data.
DATA_TYPE_MAPPING = {
    # General / System
    "SeqNum": "int",        # Sequence number of the log entry
    "Timestamp": "int",     # Timestamp, typically milliseconds from start

    # GPS Data
    "FixType": "int",       # GPS fix type (e.g., 0: No fix, 3: 3D fix)
    "Sats": "int",          # Number of satellites used in solution
    "Lat": "float",         # Latitude (degrees)
    "Long": "float",        # Longitude (degrees)
    "Alt": "float",         # GPS Altitude (meters above ellipsoid)
    "AltMSL": "float",      # GPS Altitude (meters above Mean Sea Level)
    "Speed": "float",       # GPS ground speed (m/s)
    "Heading": "float",     # GPS heading (degrees)
    "pDOP": "float",        # GPS Position Dilution of Precision
    "RTK": "int",           # RTK fix status (if applicable)

    # Barometer / Environment
    "RawAltitude": "float", # Altitude calculated from raw pressure
    "CalibratedAltitude": "float", # Altitude after calibration/filtering
    "Pressure": "float",    # Atmospheric pressure (e.g., hPa)
    "Temperature": "float", # Barometer temperature (°C)

    # Accelerometers
    "KX134_AccelX": "float", "KX134_AccelY": "float", "KX134_AccelZ": "float", # High-G accelerometer
    "ICM_AccelX": "float", "ICM_AccelY": "float", "ICM_AccelZ": "float",     # IMU accelerometer

    # Gyroscopes
    "ICM_GyroX": "float", "ICM_GyroY": "float", "ICM_GyroZ": "float",       # IMU gyroscope

    # Magnetometers
    "ICM_MagX": "float", "ICM_MagY": "float", "ICM_MagZ": "float",         # IMU magnetometer
    "ICM_Temp": "float",    # IMU internal temperature (°C)

    # Orientation (Quaternions & Euler Angles)
    "Q0": "float", "Q1": "float", "Q2": "float", "Q3": "float",             # Quaternion components (w, x, y, z)
    "EulerRoll_rad": "float", "EulerPitch_rad": "float", "EulerYaw_rad": "float", # Euler angles in radians

    # Gyro Bias (if estimated and logged)
    "GyroBiasX_rps": "float", "GyroBiasY_rps": "float", "GyroBiasZ_rps": "float", # Gyro bias in radians/sec

    # Actuators & Control System
    "ActuatorX": "float", "ActuatorY": "float", "ActuatorZ": "float",       # Actuator output/command (%)
    "TargetRoll_rad": "float", "TargetPitch_rad": "float", "TargetYaw_rad": "float", # Target Euler angles for guidance
    "PIDIntRoll": "float", "PIDIntPitch": "float", "PIDIntYaw": "float"    # PID controller integral terms
}

# --- Serial Communication Functions ---

def list_serial_ports_for_gui():
    """
    Lists available serial ports.
    Used to populate the serial port selection combobox in the GUI.
    Returns a list of device names (e.g., ['COM3', '/dev/ttyUSB0']).
    """
    ports = serial.tools.list_ports.comports()
    if not ports:
        return []
    return [port.device for port in ports]

def _parse_csv_line(csv_line_content):
    """
    Parses a single line of CSV text into a list of string values.
    Args:
        csv_line_content (str): The raw CSV string for a single line.
    Returns:
        list: A list of strings, where each string is a field from the CSV line.
              Returns an empty list if parsing fails or the line is empty.
    """
    if not csv_line_content:
        return []
    try:
        reader = csv.reader([csv_line_content]) # csv.reader expects an iterable of lines
        for row in reader: # Should only be one row
            return row
    except csv.Error as e:
        print(f"Warning: CSV parsing error for line '{csv_line_content}': {e}")
        return [] # Return empty list on parsing error
    return []

def _attempt_type_conversion(value_str, target_type_str, column_name):
    """
    Attempts to convert a string value to the specified target type (int, float).
    Uses the DATA_TYPE_MAPPING.
    Args:
        value_str (str): The string value to convert.
        target_type_str (str): The target type ('int' or 'float').
        column_name (str): The name of the column (for error reporting).
    Returns:
        The converted value (int or float), or the original string if conversion fails
        or target_type_str is not 'int' or 'float'. For failed numeric conversions,
        returns 0 for int/float to prevent downstream errors.
    """
    try:
        if target_type_str == "int":
            return int(value_str)
        elif target_type_str == "float":
            return float(value_str)
        else:
            # This case should ideally not be reached if DATA_TYPE_MAPPING is comprehensive
            # and _read_serial_data defaults unknown columns to 'float'.
            print(f"Warning: Unknown target type '{target_type_str}' for column '{column_name}'. Value '{value_str}' kept as string.")
            return value_str
    except ValueError:
        # If conversion fails, print a warning and return a default value for numeric types
        # to avoid crashing the plotting or data processing logic.
        print(f"Warning: Could not convert '{value_str}' to {target_type_str} for column '{column_name}'. Using default value (0 or original string).")
        if target_type_str in ["int", "float"]:
            return 0 # Default to 0 for numeric types on conversion error
        return value_str # Return original string if it was meant to be a string but failed (unlikely here)

def read_serial_data(port, baudrate):
    """
    This function runs in a separate thread and continuously reads data from the specified serial port.
    It parses incoming CSV data, converts values to their appropriate types,
    and adds the data (as a dictionary) to the global `data_queue`.
    Handles header detection and data integrity checks (column count).

    Args:
        port (str): The serial port to connect to (e.g., 'COM3' or '/dev/ttyUSB0').
        baudrate (int): The baud rate for the serial communication (e.g., 115200).
    """
    global serial_connection, data_queue, stop_thread_flag, COLUMN_HEADERS, EXPECTED_COLUMN_COUNT, DATA_TYPE_MAPPING

    try:
        # Attempt to open the serial connection
        serial_connection = serial.Serial(port, baudrate, timeout=1) # timeout=1 allows thread to check stop_thread_flag
        print(f"INFO: Serial port {port} opened successfully at {baudrate} baud.")
    except serial.SerialException as e:
        print(f"Error: Could not open serial port {port}: {e}")
        # Signal GUI or handle error appropriately if needed here
        return # Exit thread if port cannot be opened

    while serial_connection.is_open and not stop_thread_flag.is_set():
        try:
            # Read a line from the serial port, decode it, and strip whitespace
            line = serial_connection.readline().decode('utf-8', errors='replace').strip()

            if PRINT_RAW_SERIAL:
                print(f"RAW: {line}")

            if not line: # Skip empty lines (often due to timeout)
                continue

            # --- Header Detection Logic ---
            if not COLUMN_HEADERS:
                parsed_line_as_header = _parse_csv_line(line)
                # Basic heuristic: Check if "SeqNum" or "Timestamp" is in the first potential header field.
                # This helps distinguish header from data if connection starts mid-stream.
                if parsed_line_as_header and ("SeqNum" in parsed_line_as_header[0] or "Timestamp" in parsed_line_as_header[0]):
                    COLUMN_HEADERS[:] = parsed_line_as_header # Assign to global
                    EXPECTED_COLUMN_COUNT = len(COLUMN_HEADERS)
                    print(f"INFO: CSV Headers received: {COLUMN_HEADERS}")
                    print(f"INFO: Expected column count: {EXPECTED_COLUMN_COUNT}")
                    # Verify all received headers are in DATA_TYPE_MAPPING or add them with a default type (e.g., float)
                    for header in COLUMN_HEADERS:
                        if header not in DATA_TYPE_MAPPING:
                            print(f"Warning: Header '{header}' not found in DATA_TYPE_MAPPING. Defaulting to 'float'.")
                            DATA_TYPE_MAPPING[header] = "float" # Dynamically add if necessary
                    continue # Header processed, continue to next line for data
                else:
                    # Received a line that doesn't look like a header before headers were established.
                    # This could be a partial line or noise.
                    if parsed_line_as_header: # Only print if it was somewhat parsable
                        print(f"Warning: Received data before headers or unrecognized header format: '{line}'")
                    continue

            # --- Data Parsing Logic ---
            if COLUMN_HEADERS: # Process data only if headers are known
                parsed_values = _parse_csv_line(line)
                if len(parsed_values) == EXPECTED_COLUMN_COUNT:
                    data_dict = {}
                    for i in range(EXPECTED_COLUMN_COUNT):
                        col_name = COLUMN_HEADERS[i]
                        raw_val_str = parsed_values[i]
                        # Determine target type: Use mapped type, or default to 'float' if somehow still unknown
                        target_type = DATA_TYPE_MAPPING.get(col_name, "float")
                        converted_val = _attempt_type_conversion(raw_val_str, target_type, col_name)
                        data_dict[col_name] = converted_val
                    data_queue.append(data_dict) # Add the processed data dictionary to the queue
                else:
                    # Line doesn't match expected number of columns
                    if parsed_values: # Only print warning if there was some data, not for empty lines
                        print(f"Warning: Mismatched column count. Expected {EXPECTED_COLUMN_COUNT}, got {len(parsed_values)}. Line: '{line}'")

        except serial.SerialException as e:
            # Handle errors during an active connection (e.g., device unplugged)
            print(f"Serial error during read: {e}")
            break # Exit the loop on serial error
        except Exception as e:
            # Catch any other unexpected errors during line processing
            print(f"Error processing serial data: {e}. Line: '{line}'")
            # Depending on severity, might 'continue' or 'break'
            # break # For now, break on other errors to be safe

    # --- Cleanup when loop exits ---
    if serial_connection and serial_connection.is_open:
        serial_connection.close()
        print(f"INFO: Serial port {port} closed.")
    print("INFO: Serial reader thread ending.")

def start_serial_reader(port, baudrate=115200):
    """
    Starts the serial data reading thread.
    If a thread is already running, it attempts to stop it before starting a new one.
    Args:
        port (str): The serial port name.
        baudrate (int): The baud rate.
    """
    global reader_thread, stop_thread_flag, COLUMN_HEADERS, EXPECTED_COLUMN_COUNT
    
    if reader_thread and reader_thread.is_alive():
        print("INFO: Stopping existing serial reader thread first...")
        stop_serial_reader() # Ensure previous one is fully stopped and joined

    stop_thread_flag.clear() # Reset the stop flag for the new thread
    COLUMN_HEADERS[:] = [] # Clear previous headers
    EXPECTED_COLUMN_COUNT = 0 # Reset expected column count
    # data_queue.clear() # Optionally clear the data queue on new connection
    
    # Create and start the new thread. `daemon=True` allows main program to exit even if thread is running.
    reader_thread = threading.Thread(target=read_serial_data, args=(port, baudrate), daemon=True)
    reader_thread.start()
    print(f"INFO: Serial reader thread started for {port}.")

def stop_serial_reader():
    """
    Signals the serial reader thread to stop and waits for it to join.
    Also handles cleanup of the global `serial_connection` variable if the thread
    itself didn't close it (e.g., due to an error).
    """
    global reader_thread, stop_thread_flag, serial_connection
    
    if reader_thread and reader_thread.is_alive():
        print("INFO: Stopping serial reader thread...")
        stop_thread_flag.set() # Signal the thread to stop its loop
        
        # The serial_connection.close() call is best left to the thread that opened it (read_serial_data).
        # Setting the flag and joining is the primary mechanism.
        # If readline() in the thread is blocking, the timeout in serial.Serial(timeout=1) helps.
        
        reader_thread.join(timeout=2.0) # Wait for the thread to finish, with a timeout
        
        if reader_thread.is_alive():
            print("Warning: Serial reader thread did not stop in time. It might be blocked.")
            # If the thread is still alive, it might mean serial_connection.readline() is stuck.
            # A forced close here from another thread can be risky but might be a last resort
            # if serial_connection:
            #     try:
            #         serial_connection.close()
            #         print("INFO: Forcibly closed serial connection from main thread.")
            #     except Exception as e:
            #         print(f"Error forcibly closing serial connection: {e}")
        else:
            print("INFO: Serial reader thread stopped successfully.")
    else:
        print("INFO: Serial reader thread not running or already stopped.")
    
    # Ensure serial_connection reflects reality if thread closed it or failed to open
    if serial_connection and not serial_connection.is_open:
        serial_connection = None # Set to None if it's closed


# --- Main GUI Application Class ---

class FlightVisualizerApp:
    """
    The main class for the Flight Data Visualizer application.
    It sets up the Tkinter GUI, manages serial connections,
    handles data plotting (2D and 3D), and coordinates updates.
    """
    def __init__(self, master):
        """
        Initializes the FlightVisualizerApp.
        Args:
            master (tk.Tk): The root Tkinter window.
        """
        self.master = master
        master.title("TripleT Flight Visualizer")
        master.geometry("1600x900") # Set initial window size

        # --- Serial Controls Frame (Left Panel) ---
        # This frame contains widgets for selecting serial port, connecting/disconnecting,
        # controlling plot history, and clearing plots.
        serial_frame = ttk.LabelFrame(master, text="Serial Connection")
        serial_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=5) # Pack to the left, fill vertically

        # Serial port selection
        self.port_label = ttk.Label(serial_frame, text="Serial Port:")
        self.port_label.pack(side=tk.TOP, anchor=tk.W, padx=5, pady=5)
        self.port_combobox = ttk.Combobox(serial_frame, width=25, state="readonly") # Readonly prevents manual typing
        self.port_combobox.pack(side=tk.TOP, anchor=tk.W, padx=5, pady=5)

        # Buttons for port refresh, connect, disconnect
        self.refresh_button = ttk.Button(serial_frame, text="Refresh Ports", command=self.list_serial_ports_gui)
        self.refresh_button.pack(side=tk.TOP, anchor=tk.W, padx=5, pady=5)
        self.connect_button = ttk.Button(serial_frame, text="Connect", command=self.connect_serial)
        self.connect_button.pack(side=tk.TOP, anchor=tk.W, padx=5, pady=5)
        self.disconnect_button = ttk.Button(serial_frame, text="Disconnect", command=self.disconnect_serial, state=tk.DISABLED) # Starts disabled
        self.disconnect_button.pack(side=tk.TOP, anchor=tk.W, padx=5, pady=5)

        # Status label for connection messages
        self.status_label = ttk.Label(serial_frame, text="Status: Disconnected", width=30)
        self.status_label.pack(side=tk.TOP, anchor=tk.W, padx=5, pady=5)

        # Plot History Control
        self.plot_history_size_var = tk.IntVar(value=100) # Tkinter variable for spinbox
        plot_history_label = ttk.Label(serial_frame, text="Plot History (points):")
        plot_history_label.pack(side=tk.TOP, anchor=tk.W, padx=5, pady=(10,0)) # Padding: (top, bottom)
        plot_history_spinbox = ttk.Spinbox(serial_frame, from_=50, to=1000, increment=50,
                                           textvariable=self.plot_history_size_var, width=7)
        plot_history_spinbox.pack(side=tk.TOP, anchor=tk.W, padx=5, pady=2)

        # Clear Plots Button
        self.clear_plots_button = ttk.Button(serial_frame, text="Clear Plots", command=self.clear_all_plots)
        self.clear_plots_button.pack(side=tk.TOP, anchor=tk.W, padx=5, pady=5)

        self.list_serial_ports_gui() # Populate port list initially

        # --- Main Data Display Frame (Right Panel) ---
        # This frame is a container for the 3D view and the 2D plot tabs.
        data_display_frame = ttk.Frame(master)
        data_display_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=5)

        # --- 3D View Frame (Top Right) ---
        self.view_3d_frame = ttk.LabelFrame(data_display_frame, text="3D Orientation", width=400, height=350)
        self.view_3d_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.view_3d_frame.pack_propagate(False) # Prevent children from shrinking the frame

        # Initialize PyVista BackgroundPlotter for 3D visualization
        self.pv_plotter = None
        self.orientation_object_actor = None
        try:
            # BackgroundPlotter integrates PyVista with a Qt backend, which can then be embedded in Tkinter.
            # Requires a Qt binding like PySide2 or PyQt5 to be installed.
            self.pv_plotter = BackgroundPlotter(
                app=master, # Pass the Tkinter root window
                window_size=(self.view_3d_frame.winfo_reqwidth(), self.view_3d_frame.winfo_reqheight()), # Initial size
                show_toolbar=False, # PyVista's own toolbar, not the Matplotlib one
                panel=self.view_3d_frame # Embed the PyVista rendering window into this Tkinter frame
            )
            # Add a 3D arrow mesh to represent the orientation. Could be any pv.PolyData object.
            self.orientation_object_actor = self.pv_plotter.add_mesh(pv.Arrow(direction=(0,0,1), scale=1.5), name='orientation_indicator', color='cyan')
            
            # Configure camera and axes appearance for the 3D view
            self.pv_plotter.camera_position = 'iso' # Isometric view
            self.pv_plotter.camera.azimuth = 45
            self.pv_plotter.camera.elevation = 30
            self.pv_plotter.enable_zoom_scaling() # Allow zooming
            self.pv_plotter.add_axes_at_origin(labels_off=False) # Show X, Y, Z axes at origin
            self.pv_plotter.view_isometric() # Apply settings and reset view
            
        except Exception as e:
            # Fallback if PyVista initialization fails (e.g., missing Qt bindings)
            print(f"Error initializing PyVista plotter: {e}")
            self.pv_plotter = None
            self.orientation_object_actor = None
            # Display an error message in the 3D view frame
            error_label = ttk.Label(self.view_3d_frame, text=f"PyVista Error: {e}\nEnsure PySide2 or PyQt5 is installed.", wraplength=380)
            error_label.pack(padx=10, pady=10, fill='both', expand=True)


        # --- 2D Plotting Frame with Tabs (Bottom Right) ---
        # This frame uses a ttk.Notebook to manage multiple tabs, each containing Matplotlib plots.
        plot_frame = ttk.Frame(data_display_frame) 
        plot_frame.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.plot_notebook = ttk.Notebook(plot_frame) # The tab manager
        self.plot_notebook.pack(expand=True, fill='both')

        self.tabs_data = {} # Dictionary to store data related to each tab (figure, canvas, axes)
        
        # Configuration for each plot tab. Defines the plots, data keys, titles, and layout.
        # 'key' for single-line plots, 'keys' for multi-line plots on the same axes.
        # 'labels' for multi-line plots legends.
        self.tab_config = {
            "IMU": { # Inertial Measurement Unit Tab
                "plots": [
                    {'key': 'ICM_AccelX', 'title': 'ICM Accel X (m/s^2)'}, {'key': 'ICM_AccelY', 'title': 'ICM Accel Y (m/s^2)'}, {'key': 'ICM_AccelZ', 'title': 'ICM Accel Z (m/s^2)'},
                    {'key': 'ICM_GyroX', 'title': 'ICM Gyro X (rad/s)'}, {'key': 'ICM_GyroY', 'title': 'ICM Gyro Y (rad/s)'}, {'key': 'ICM_GyroZ', 'title': 'ICM Gyro Z (rad/s)'}
                ], "layout": (2,3) # 2 rows, 3 columns of subplots
            },
            "HiG_Mag": { # High-G Accelerometer and Magnetometer Tab
                "plots": [
                    {'key': 'KX134_AccelX', 'title': 'KX134 Accel X (m/s^2)'}, {'key': 'KX134_AccelY', 'title': 'KX134 Accel Y (m/s^2)'}, {'key': 'KX134_AccelZ', 'title': 'KX134 Accel Z (m/s^2)'},
                    {'key': 'ICM_MagX', 'title': 'ICM Mag X (uT)'}, {'key': 'ICM_MagY', 'title': 'ICM Mag Y (uT)'}, {'key': 'ICM_MagZ', 'title': 'ICM Mag Z (uT)'}
                ], "layout": (2,3)
            },
            "Env_Alt": { # Environment (Temp, Pressure) and Altitude Tab
                "plots": [
                    {'key': 'Temperature', 'title': 'Baro Temp (°C)'}, {'key': 'ICM_Temp', 'title': 'ICM Temp (°C)'}, {'key': 'Pressure', 'title': 'Pressure (hPa)'},
                    {'key': 'RawAltitude', 'title': 'Raw Alt (Baro, m)'}, {'key': 'CalibratedAltitude', 'title': 'Cal. Alt (Baro, m)'}, {'key': 'Alt', 'title': 'GPS Alt (MSL, m)'}
                ], "layout": (2,3)
            },
            "GPS": { # GPS Data Tab
                "plots": [
                    {'key': 'Speed', 'title': 'GPS Speed (m/s)'}, {'key': 'Heading', 'title': 'GPS Heading (°)'},
                    {'key': 'Sats', 'title': 'Satellites'}, {'key': 'pDOP', 'title': 'pDOP'}
                ], "layout": (2,2) # 2 rows, 2 columns
            },
            "Euler": { # Euler Angles (Actual vs. Target) Tab
                "plots": [
                    {'keys': ['EulerRoll_rad', 'TargetRoll_rad'], 'title': 'Euler Roll (rad)', 'labels': ['Actual Roll', 'Target Roll']},
                    {'keys': ['EulerPitch_rad', 'TargetPitch_rad'], 'title': 'Euler Pitch (rad)', 'labels': ['Actual Pitch', 'Target Pitch']},
                    {'keys': ['EulerYaw_rad', 'TargetYaw_rad'], 'title': 'Euler Yaw (rad)', 'labels': ['Actual Yaw', 'Target Yaw']}
                ], "layout": (1,3) # 1 row, 3 columns
            },
            "Actuators_PID": { # Actuator Outputs and PID Controller Integrals Tab
                "plots": [
                    {'key': 'ActuatorX', 'title': 'Actuator X (%)'}, {'key': 'ActuatorY', 'title': 'Actuator Y (%)'}, {'key': 'ActuatorZ', 'title': 'Actuator Z (%)'},
                    {'key': 'PIDIntRoll', 'title': 'PID Integral Roll'}, {'key': 'PIDIntPitch', 'title': 'PID Integral Pitch'}, {'key': 'PIDIntYaw', 'title': 'PID Integral Yaw'}
                ], "layout": (2,3)
            }
        }
 
        # Dynamically create tabs, figures, canvases, and axes based on tab_config
        for tab_name, config in self.tab_config.items():
            tab_frame = ttk.Frame(self.plot_notebook) # Create a frame for each tab
            self.plot_notebook.add(tab_frame, text=tab_name) # Add tab to notebook
            
            # Create a Matplotlib Figure and Canvas for each tab
            fig = Figure(figsize=(10, 7), dpi=100) # Adjust size as needed
            canvas = FigureCanvasTkAgg(fig, master=tab_frame)
            
            # Embed the Matplotlib navigation toolbar in each tab
            toolbar = NavigationToolbar2Tk(canvas, tab_frame)
            toolbar.update() # Necessary for the toolbar to be correctly displayed
            canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True) # Pack canvas into tab
            
            current_axes_map = {} # Maps plot key to Matplotlib Axes object for this tab
            rows, cols = config['layout']
            plot_infos_list = config['plots'] # List of plot definitions for this tab
 
            for i, plot_info_dict in enumerate(plot_infos_list):
                ax = fig.add_subplot(rows, cols, i + 1) # Add subplot to the figure
                # Store the Axes object. Use the primary data key (or first key for multi-line) as the dict key.
                plot_key_primary = plot_info_dict.get('key', plot_info_dict['keys'][0]) 
                current_axes_map[plot_key_primary] = ax
            
            # Store all parts for this tab (frame, figure, canvas, axes map)
            self.tabs_data[tab_name] = {'frame': tab_frame, 'figure': fig, 'canvas': canvas, 'axes_map': current_axes_map}
 
        self.update_plots() # Start the periodic plot update cycle
        master.protocol("WM_DELETE_WINDOW", self.on_closing) # Handle window close event
 
    def list_serial_ports_gui(self):
        """Populates the serial port combobox with available ports."""
        ports = list_serial_ports_for_gui()
        self.port_combobox['values'] = ports
        if ports:
            self.port_combobox.current(0) # Select first available port by default
        else:
            self.port_combobox.set('') # Clear selection if no ports found
            self.status_label.config(text="Status: No serial ports found.")
 
    def connect_serial(self):
        """Handles the 'Connect' button action. Starts serial reading."""
        global serial_connection, reader_thread # Allow modification of these globals
        selected_port = self.port_combobox.get()
        if not selected_port:
            messagebox.showerror("Error", "No serial port selected.")
            return
 
        baudrate = 115200 # Standard baud rate, could be made configurable
        start_serial_reader(selected_port, baudrate) # Call the global function to start the thread
 
        self.status_label.config(text=f"Status: Attempting to connect to {selected_port}...")
        self.master.update_idletasks() # Ensure label updates immediately
 
        # Basic check for connection status. A more robust check might involve
        # feedback from the serial thread or a timeout mechanism.
        time.sleep(0.5) # Give thread a moment to establish connection
 
        if reader_thread and reader_thread.is_alive() and serial_connection and serial_connection.is_open:
            self.status_label.config(text=f"Status: Connected to {selected_port}")
            # Update GUI element states: disable connect, enable disconnect, etc.
            self.connect_button.config(state=tk.DISABLED)
            self.disconnect_button.config(state=tk.NORMAL)
            self.port_combobox.config(state=tk.DISABLED) # Prevent changing port while connected
            self.refresh_button.config(state=tk.DISABLED)
        else:
            self.status_label.config(text="Status: Failed to connect. Check console.")
            stop_serial_reader() # Ensure thread is stopped if connection failed partway
 
    def disconnect_serial(self):
        """Handles the 'Disconnect' button action. Stops serial reading."""
        global COLUMN_HEADERS, EXPECTED_COLUMN_COUNT # Allow modification
        stop_serial_reader() # Call the global function to stop the thread
 
        self.status_label.config(text="Status: Disconnected")
        # Update GUI element states: enable connect, disable disconnect, etc.
        self.connect_button.config(state=tk.NORMAL)
        self.disconnect_button.config(state=tk.DISABLED)
        self.port_combobox.config(state=tk.NORMAL)
        self.refresh_button.config(state=tk.NORMAL)
        
        # Clear headers and expected count as we are no longer connected to that specific data source
        COLUMN_HEADERS[:] = []
        EXPECTED_COLUMN_COUNT = 0
        
        # Optionally, clear plots or display "disconnected" message on them.
        # Current clear_all_plots handles clearing, but could add specific message here.
        # For now, rely on update_plots not running if disconnected.
 
    def update_plots(self):
        """
        Periodically called to update all 2D Matplotlib plots with new data from the `data_queue`.
        This method is the heart of the 2D visualization update loop.
        """
        global data_queue, COLUMN_HEADERS # Access global data queue and headers
 
        # Only proceed if connected and headers have been received
        if not (serial_connection and serial_connection.is_open and COLUMN_HEADERS):
            self.master.after(100, self.update_plots) # Reschedule if not ready
            return
 
        plot_history_count = self.plot_history_size_var.get() # Get desired history from spinbox
        current_data_full = list(data_queue) # Snapshot of the entire data queue
 
        # Slice data for 2D plots based on history count
        if len(current_data_full) > plot_history_count:
            current_data_for_plots = current_data_full[-plot_history_count:]
        else:
            current_data_for_plots = current_data_full
        
        # If no data for 2D plots (e.g., history count is 0 or queue is empty),
        # still attempt to update 3D view with any available data and reschedule.
        if not current_data_for_plots:
            if current_data_full: # If there's any data at all (for 3D view)
                 self.update_3d_view(current_data_full)
            self.master.after(100, self.update_plots) # Reschedule
            return
 
        # Prepare timestamps for X-axis of 2D plots
        # Use 'Timestamp' or 'SeqNum' if available, otherwise generate simple sequence numbers.
        ts_key = 'Timestamp' if 'Timestamp' in COLUMN_HEADERS else 'SeqNum'
        if ts_key not in COLUMN_HEADERS: # Fallback if neither standard key is present
            timestamps = list(range(len(current_data_for_plots)))
        else:
            timestamps = [d.get(ts_key, i) for i, d in enumerate(current_data_for_plots)] # Get value or index
 
        # Iterate through each configured tab and update its plots
        for tab_name, tab_content in self.tabs_data.items():
            figure = tab_content['figure']
            canvas = tab_content['canvas']
            axes_map = tab_content['axes_map'] # Map of plot key to Axes object
            plot_infos_list = self.tab_config[tab_name]['plots'] # List of plot definitions
 
            for plot_info_dict in plot_infos_list:
                # Determine the primary key for this plot (first key if multi-line)
                plot_key_primary = plot_info_dict.get('key', plot_info_dict['keys'][0])
                ax = axes_map[plot_key_primary] # Get the Matplotlib Axes for this plot
                ax.clear() # Clear previous plot content
 
                if 'keys' in plot_info_dict: # Handle multi-line plots (e.g., Actual vs. Target)
                    for k_idx, key_to_plot in enumerate(plot_info_dict['keys']):
                        # Extract data series for this key, handling missing data with None
                        data_series = [d.get(key_to_plot, None) for d in current_data_for_plots]
                        # Filter out None values to prevent plotting errors and gaps
                        valid_timestamps = [ts for i, ts in enumerate(timestamps) if data_series[i] is not None]
                        valid_data_series = [val for val in data_series if val is not None]
                        line_label = plot_info_dict['labels'][k_idx] # Get label for this line
                        if valid_timestamps and valid_data_series:
                            ax.plot(valid_timestamps, valid_data_series, label=line_label)
                        else:
                            ax.plot([], [], label=line_label) # Plot empty if no valid data for this line
                else: # Handle single-line plots
                    key_to_plot = plot_info_dict['key']
                    data_series = [d.get(key_to_plot, None) for d in current_data_for_plots]
                    valid_timestamps = [ts for i, ts in enumerate(timestamps) if data_series[i] is not None]
                    valid_data_series = [val for val in data_series if val is not None]
                    if valid_timestamps and valid_data_series:
                        ax.plot(valid_timestamps, valid_data_series, label=key_to_plot)
                    else:
                        ax.plot([], [], label=key_to_plot) # Plot empty if no valid data
                
                # Set plot title, legend, and grid
                ax.set_title(plot_info_dict['title'])
                ax.legend(loc='upper left', fontsize='small')
                ax.grid(True)
            
            figure.tight_layout() # Adjust subplot parameters for a tight layout
            canvas.draw() # Redraw the canvas for this tab
        
        # After updating 2D plots, update the 3D view with the full data snapshot
        # (as it typically uses only the latest point from this snapshot).
        if current_data_full:
            self.update_3d_view(current_data_full)
 
        self.master.after(100, self.update_plots) # Schedule the next update (aim for ~10 FPS)
 
    def clear_all_plots(self):
        """Clears all data from all 2D Matplotlib plots."""
        print("INFO: Clearing all 2D plots...")
        for tab_name, tab_content in self.tabs_data.items(): # Iterate through each tab
            axes_map = tab_content['axes_map']
            canvas = tab_content['canvas']
            figure = tab_content['figure']
            
            plot_infos_for_titles = self.tab_config[tab_name]['plots']
 
            for plot_key_primary, ax in axes_map.items():
                ax.clear() # Clear the plot content
                # Re-apply title and grid as ax.clear() removes them.
                # Find the original plot_info to get the title.
                current_plot_info = None
                for pi in plot_infos_for_titles: # Iterate plot definitions for current tab
                    if pi.get('key', pi.get('keys', [None])[0]) == plot_key_primary:
                        current_plot_info = pi
                        break
                if current_plot_info:
                    ax.set_title(current_plot_info['title'])
                else: # Fallback title if config somehow not found
                    ax.set_title("Plot")
                ax.grid(True) # Re-enable grid
            
            figure.tight_layout() # Re-apply layout adjustments
            canvas.draw() # Redraw the cleared canvas
        print("INFO: All plots cleared.")
 
    def update_3d_view(self, current_data_full_snapshot):
        """
        Updates the 3D orientation view using the latest quaternion data.
        Args:
            current_data_full_snapshot (list): A list of data dictionaries (the full data queue snapshot).
                                               The last element is used for the latest orientation.
        """
        # Only proceed if PyVista plotter and actor are initialized and data is available
        if not self.pv_plotter or not self.orientation_object_actor or not current_data_full_snapshot:
            return
 
        latest_data_point = current_data_full_snapshot[-1] # Get the most recent data point
        
        # Extract quaternion components (Q0=w, Q1=x, Q2=y, Q3=z) with defaults if missing
        q0 = latest_data_point.get('Q0', 1.0) # Scalar component (w)
        q1 = latest_data_point.get('Q1', 0.0) # Vector component (x)
        q2 = latest_data_point.get('Q2', 0.0) # Vector component (y)
        q3 = latest_data_point.get('Q3', 0.0) # Vector component (z)
 
        try:
            # Convert quaternion to a 3x3 rotation matrix.
            # SciPy's Rotation.from_quat expects (x, y, z, w) order.
            rotation = R_scipy.from_quat([q1, q2, q3, q0])
            rotation_matrix_3x3 = rotation.as_matrix()
 
            # Create a 4x4 homogeneous transformation matrix for PyVista.
            # PyVista actors often use 4x4 matrices for position, rotation, and scale.
            transform_matrix_4x4 = np.eye(4) # Start with identity matrix
            transform_matrix_4x4[:3, :3] = rotation_matrix_3x3 # Set the top-left 3x3 to the rotation
 
            # Apply this transformation matrix to the PyVista actor's user_matrix.
            # This directly sets the orientation (and position if included) of the actor.
            self.orientation_object_actor.user_matrix = transform_matrix_4x4
            
            # The BackgroundPlotter should render automatically.
            # If explicit rendering is needed (rarely for BackgroundPlotter): self.pv_plotter.render()
        except Exception as e:
            print(f"Error updating 3D orientation: {e}. Quaternion data: Q0={q0},Q1={q1},Q2={q2},Q3={q3}")
 
    def on_closing(self):
        """Handles the window close event to ensure graceful shutdown."""
        print("INFO: Closing application...")
        self.disconnect_serial() # Disconnect serial port and stop reader thread
        
        # Close the PyVista plotter if it was initialized
        if self.pv_plotter:
            try:
                self.pv_plotter.close() # Proper cleanup for PyVista BackgroundPlotter
                print("INFO: PyVista plotter closed.")
            except Exception as e:
                print(f"Error closing PyVista plotter: {e}")
 
        # Ensure the main Tkinter window is destroyed
        if self.master.winfo_exists(): # Check if window still exists
             self.master.destroy()
        print("INFO: Application closed.")
 
 
 # --- Main Execution Block ---
if __name__ == "__main__":
    # This block runs when the script is executed directly.
    
    # DATA_TYPE_MAPPING is already defined globally and used by read_serial_data.
    # No specific setup for it needed here.
 
    # Create the main Tkinter window and start the application
    root = tk.Tk() # Create the top-level window
    app = FlightVisualizerApp(root) # Instantiate the application class
    root.mainloop() # Start the Tkinter event loop
 
    # After mainloop exits (window is closed), ensure everything is cleaned up.
    # This part might be redundant if on_closing is robust and always called.
    if reader_thread and reader_thread.is_alive():
        print("INFO: mainloop ended, ensuring serial reader is stopped (extra check).")
        stop_serial_reader()
