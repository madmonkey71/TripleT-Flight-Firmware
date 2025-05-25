# Standard library imports
import serial
import serial.tools.list_ports
import threading
import time
import csv
import collections # For collections.deque
import pprint # For pretty printing data, useful in debugging
import os # Added for setting environment variables
from PIL import Image, ImageTk # Added for off-screen rendering

# Force PyVista to use PyQt5, must be done before PyVista imports
os.environ['PYVISTA_QT_API'] = 'pyqt5'

# GUI and plotting libraries
import tkinter as tk
from tkinter import ttk, messagebox # Themed Tkinter widgets and standard dialogs
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk # Matplotlib Tkinter embedding
from matplotlib.ticker import FuncFormatter # Added for custom timestamp formatting

# Numerical and 3D libraries
import numpy as np
import pyvista as pv
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
    "Timestamp": "float",     # Changed from "int" to "float" for robust parsing

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
    "RTK": "float",           # RTK fix status (if applicable)

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

# Helper function for formatting milliseconds to HH:MM:SS
def format_milli_to_hhmmss(milliseconds, pos):
    # print(f"DEBUG: format_milli_to_hhmmss received: {milliseconds} (type: {type(milliseconds)})") # Uncomment for deep debug
    if milliseconds is None or not isinstance(milliseconds, (int, float)) or milliseconds < 0:
        # print("DEBUG: format_milli_to_hhmmss returning empty due to invalid input") # Uncomment for deep debug
        return "" # Return empty string for invalid inputs
    total_seconds = int(milliseconds / 1000)
    hours = total_seconds // 3600
    minutes = (total_seconds % 3600) // 60
    seconds = total_seconds % 60
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}"

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
    It now assumes COLUMN_HEADERS and EXPECTED_COLUMN_COUNT are pre-initialized based on DATA_TYPE_MAPPING.

    Args:
        port (str): The serial port to connect to (e.g., 'COM3' or '/dev/ttyUSB0').
        baudrate (int): The baud rate for the serial communication (e.g., 115200).
    """
    global serial_connection, data_queue, stop_thread_flag, COLUMN_HEADERS, EXPECTED_COLUMN_COUNT, DATA_TYPE_MAPPING

    # COLUMN_HEADERS and EXPECTED_COLUMN_COUNT are now assumed to be initialized
    # by start_serial_reader before this thread starts.

    try:
        # Attempt to open the serial connection
        serial_connection = serial.Serial(port, baudrate, timeout=1) # timeout=1 allows thread to check stop_thread_flag
        print(f"INFO: Serial port {port} opened successfully at {baudrate} baud.")
    except serial.SerialException as e:
        print(f"Error: Could not open serial port {port}: {e}")
        # Signal GUI or handle error appropriately if needed here
        return # Exit thread if port cannot be opened

    while serial_connection.is_open and not stop_thread_flag.is_set():
        line = "" # Initialize line to ensure it's defined for error messages
        try:
            line = serial_connection.readline().decode('utf-8', errors='replace').strip()
            if PRINT_RAW_SERIAL:
                print(f"RAW: {line}")
            if not line:
                continue
            if COLUMN_HEADERS:
                parsed_values = _parse_csv_line(line)
                if len(parsed_values) == EXPECTED_COLUMN_COUNT:
                    data_dict = {}
                    for i in range(EXPECTED_COLUMN_COUNT):
                        col_name = COLUMN_HEADERS[i]
                        raw_val_str = parsed_values[i]
                        target_type = DATA_TYPE_MAPPING.get(col_name, "float")
                        converted_val = _attempt_type_conversion(raw_val_str, target_type, col_name)
                        data_dict[col_name] = converted_val
                    data_queue.append(data_dict)
                else:
                    if parsed_values:
                        print(f"Warning: Mismatched column count. Expected {EXPECTED_COLUMN_COUNT}, got {len(parsed_values)}. Line: '{line}'")
        except serial.SerialException as e:
            print(f"Serial error during read: {e}")
            break # Exit the loop on major serial error
        except Exception as e:
            print(f"CRITICAL: Error processing serial data line: {e}. Line: '{line}'. Skipping line, continuing read.")
            continue # Continue to try and process further lines

    if serial_connection and serial_connection.is_open:
        serial_connection.close()
        print(f"INFO: Serial port {port} closed (read_serial_data exit).") # Added detail
    print("INFO: Serial reader thread ending (read_serial_data exit).") # Added detail

def start_serial_reader(port, baudrate=115200):
    """
    Starts the serial data reading thread.
    If a thread is already running, it attempts to stop it before starting a new one.
    Initializes COLUMN_HEADERS and EXPECTED_COLUMN_COUNT from DATA_TYPE_MAPPING.
    Args:
        port (str): The serial port name.
        baudrate (int): The baud rate.
    """
    global reader_thread, stop_thread_flag, COLUMN_HEADERS, EXPECTED_COLUMN_COUNT, DATA_TYPE_MAPPING
    
    if reader_thread and reader_thread.is_alive():
        print("INFO: Stopping existing serial reader thread first...")
        stop_serial_reader() # Ensure previous one is fully stopped and joined

    stop_thread_flag.clear() # Reset the stop flag for the new thread
    
    # Initialize COLUMN_HEADERS and EXPECTED_COLUMN_COUNT from DATA_TYPE_MAPPING
    COLUMN_HEADERS[:] = list(DATA_TYPE_MAPPING.keys())
    EXPECTED_COLUMN_COUNT = len(COLUMN_HEADERS)
    print(f"INFO: Using predefined column headers: {COLUMN_HEADERS}")
    print(f"INFO: Expected column count based on predefined headers: {EXPECTED_COLUMN_COUNT}")
    
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
        self.plot_history_size_var = tk.IntVar(value=200) # Tkinter variable for spinbox, default 200 points (20s at 10Hz)
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
        self.view_3d_frame.update_idletasks() # Ensure frame is drawn before PyVista uses it
        self.debug_rotation_angle = 0.0 # Initialize for 3D view heartbeat

        # Setup for off-screen PyVista rendering
        self.pv_plotter = None
        self.orientation_object_actor = None
        self.pv_image_label = None # Label to display the rendered image

        try:
            pv.set_plot_theme("document") # Set PyVista theme
            
            # Use an off-screen plotter
            self.pv_plotter = pv.Plotter(off_screen=True, window_size=[self.view_3d_frame.winfo_reqwidth(), self.view_3d_frame.winfo_reqheight()])
            
            self.orientation_object_actor = self.pv_plotter.add_mesh(pv.Arrow(direction=(0,0,1), scale=1.5), name='orientation_indicator', color='cyan')
            self.pv_plotter.camera_position = 'iso'
            self.pv_plotter.camera.azimuth = 45
            self.pv_plotter.camera.elevation = 30
            self.pv_plotter.add_axes_at_origin(labels_off=False)
            self.pv_plotter.view_isometric()

            # Label to display the rendered image
            self.pv_image_label = ttk.Label(self.view_3d_frame)
            self.pv_image_label.pack(fill=tk.BOTH, expand=True)
            
            # Render an initial empty or placeholder view if desired
            # self.update_3d_view_with_placeholder() # You might call a method to show an initial state
            print("INFO: PyVista off-screen plotter initialized successfully.")

        except Exception as e:
            print(f"CRITICAL: Error initializing PyVista off-screen plotter: {e}")
            self.pv_plotter = None
            self.orientation_object_actor = None
            error_message = f"PyVista Init Error: {e}\nOff-screen rendering setup failed."
            if self.pv_image_label: # If label was created before error
                self.pv_image_label.destroy()
            error_label_widget = ttk.Label(self.view_3d_frame, text=error_message, wraplength=380)
            error_label_widget.pack(padx=10, pady=10, fill='both', expand=True)
        
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
                plot_key_primary = None
                if 'key' in plot_info_dict:
                    plot_key_primary = plot_info_dict['key']
                elif 'keys' in plot_info_dict and isinstance(plot_info_dict['keys'], list) and plot_info_dict['keys']:
                    plot_key_primary = plot_info_dict['keys'][0]
                
                if plot_key_primary is not None:
                    current_axes_map[plot_key_primary] = ax
                else:
                    print(f"Warning: Could not determine plot_key_primary for plot_info_dict at index {i} in tab '{tab_name}'. Plot axis not mapped. Details: {plot_info_dict}")
            
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
        TEMPORARILY SIMPLIFIED FOR DEBUGGING PREMATURE SHUTDOWN.
        """
        global serial_connection, COLUMN_HEADERS, data_queue # Added data_queue for minimal check

        print(f"DEBUG: update_plots called. Serial connected: {serial_connection and serial_connection.is_open}")

        # Minimal check: if serial connection is active, try to call update_3d_view with latest data if any
        # This is to see if update_3d_view itself is causing a crash when called.
        if serial_connection and serial_connection.is_open and COLUMN_HEADERS:
            current_data_full = list(data_queue)
            if current_data_full:
                # print("DEBUG: update_plots trying to call update_3d_view") # Uncomment for further debug
                try:
                    self.update_3d_view(current_data_full) # Call 3D update to check for crashes here
                except Exception as e:
                    print(f"CRITICAL ERROR in update_3d_view called from simplified update_plots: {e}")
            # else:
                # print("DEBUG: update_plots - data_queue is empty or headers not ready")
        # else:
            # print("DEBUG: update_plots - serial not connected or headers not ready")

        self.master.after(200, self.update_plots) # Reschedule, increased interval slightly for debug
 
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
        if not self.pv_plotter or not self.orientation_object_actor or not self.pv_image_label or not current_data_full_snapshot:
            return

        latest_data_point = current_data_full_snapshot[-1]
        q0 = latest_data_point.get('Q0', 1.0)
        q1 = latest_data_point.get('Q1', 0.0)
        q2 = latest_data_point.get('Q2', 0.0)
        q3 = latest_data_point.get('Q3', 0.0)

        try:
            # Sensor-based rotation
            sensor_rotation = R_scipy.from_quat([q1, q2, q3, q0])
            sensor_rotation_matrix_3x3 = sensor_rotation.as_matrix()

            # Debug heartbeat rotation (around Z-axis)
            self.debug_rotation_angle += 0.03 # Increment angle (radians)
            debug_rotation_matrix_3x3 = R_scipy.from_euler('z', self.debug_rotation_angle, degrees=False).as_matrix()

            # Combine rotations: apply sensor rotation, then debug rotation
            final_rotation_matrix_3x3 = debug_rotation_matrix_3x3 @ sensor_rotation_matrix_3x3
            
            transform_matrix_4x4 = np.eye(4)
            transform_matrix_4x4[:3, :3] = final_rotation_matrix_3x3
            self.orientation_object_actor.user_matrix = transform_matrix_4x4

            img = self.pv_plotter.screenshot(transparent_background=False, return_img=True)
            if img is not None:
                pil_image = Image.fromarray(img)
                tk_image = ImageTk.PhotoImage(image=pil_image)
                self.pv_image_label.configure(image=tk_image)
                self.pv_image_label.image = tk_image
            else:
                print("Warning: PyVista screenshot returned None.")
        except Exception as e:
            print(f"Error updating 3D orientation (off-screen): {e}. Quaternion data: Q0={q0},Q1={q1},Q2={q2},Q3={q3}")

    def on_closing(self):
        """Handles the window close event to ensure graceful shutdown."""
        print("INFO: Closing application...")
        self.disconnect_serial() # Disconnect serial port and stop reader thread
        
        # PyVista Plotter (off-screen) doesn't have a .close() method like BackgroundPlotter
        # It should be garbage collected. We can clear our reference.
        if self.pv_plotter:
            self.pv_plotter = None # Clear reference
            print("INFO: PyVista off-screen plotter reference cleared.")

        if self.master.winfo_exists():
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
