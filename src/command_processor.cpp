#include "command_processor.h"
#include "debug_flags.h"     // Include debug flags structure
#include "flight_context.h"  // Include flight context structure
#include "config.h"
#include "constants.h"
// utility_functions.h might declare initSDCard and createNewLogFile.
// If not, they are externed in command_processor.h and defined in TripleT_Flight_Firmware.cpp
#include "utility_functions.h"   // For scan_i2c
#include "ms5611_functions.h"    // For MS5611 type and ms5611_calibrate_with_gps
#include "icm_20948_functions.h" // For ICM_20948_print, etc.
#include "gps_functions.h"       // For setGPSDebugging, SFE_UBLOX_GNSS type
#include "state_management.h"    // For FlightState, getStateName, saveStateToEEPROM, isSensorSuiteHealthy
#include "kalman_filter.h"       // For kalman_init
#include "data_structures.h"
#include "log_format_definition.h"
#include "flight_logic.h"

extern ErrorCode_t g_last_error_code; // For accessing last error code

// Functions defined in TripleT_Flight_Firmware.cpp that command_processor needs.
// These declarations signal that the linker should find them elsewhere.
// initSDCard, createNewLogFile, closeAllFiles, checkStorageSpace are now declared in command_processor.h
// extern void scan_i2c(); // Now included via utility_functions.h
// extern MS5611 ms5611Sensor; // Now passed as parameter where needed

// No longer needed due to SystemStatusContext or direct parameters from processCommand:
// extern Adafruit_NeoPixel pixels;
// extern char logFileName[];
// extern uint64_t availableSpace;
// extern bool loggingEnabled;
// extern SdFat SD;
// extern bool sdCardMounted;
// extern bool sdCardPresent;
// extern bool sdCardAvailable; // Now in SystemStatusContext
// extern bool flashAvailable; // Now in SystemStatusContext
// extern bool baroCalibrated; // Now in SystemStatusContext or passed as ref
// extern bool icm20948_ready; // Now in SystemStatusContext
// extern bool ms5611_initialized_ok; // Now in SystemStatusContext
// extern bool kx134_initialized_ok; // Now in SystemStatusContext
// extern SFE_UBLOX_GNSS myGNSS; // Now in SystemStatusContext
// Madgwick filter references removed - only Kalman filter is supported
// extern bool useKalmanFilter; // Now in SystemStatusContext


// extern float icm_q0, icm_q1, icm_q2, icm_q3; // Still global, not directly used by command_processor funcs
// extern float kalmanRoll, kalmanPitch, kalmanYaw; // Still global


void printSDCardStatus(const SystemStatusContext& statusCtx, SdFat& sd_obj_ref, uint64_t& availableSpace_refresh_ref) {
  Serial.println(F("\n--- SD Card Status ---"));
  // Call utility function to refresh available space; this utility is declared in command_processor.h
  // and defined in TripleT_Flight_Firmware.cpp
  checkStorageSpace(sd_obj_ref, availableSpace_refresh_ref);

  Serial.print(F("Physically Present: "));
  Serial.println(statusCtx.sdCardPresent ? F("Yes") : F("No (SDIO initialization failed)"));

  Serial.print(F("Mounted: "));
  Serial.println(statusCtx.sdCardMounted ? F("Yes") : F("No"));

  Serial.print(F("Logging Enabled: "));
  Serial.println(statusCtx.loggingEnabled ? F("Yes") : F("No"));

  if (statusCtx.sdCardMounted) {
    Serial.print(F("Available Space: "));
    if (availableSpace_refresh_ref >= 1024 * 1024) {
      Serial.print(availableSpace_refresh_ref / (1024L * 1024L));
      Serial.println(F(" MB"));
    } else {
      Serial.print(availableSpace_refresh_ref / 1024L);
      Serial.println(F(" KB"));
    }
    Serial.print(F("Current Log File: "));
    if (statusCtx.logFileName && strlen(statusCtx.logFileName) > 0 && statusCtx.loggingEnabled) {
      Serial.println(statusCtx.logFileName);
    } else {
      Serial.println(F("None or logging not active"));
    }
  } else {
    Serial.println(F("Available Space: N/A (Card not mounted)"));
    Serial.println(F("Current Log File: N/A (Card not mounted)"));
  }
  Serial.println(F("----------------------"));
}

void attemptToStartLogging(SdFat& sd_obj,
                           SFE_UBLOX_GNSS& gnss_obj,
                           FsFile& logFile_obj,
                           char* logFileName_global_buf, // Buffer for log file name
                           size_t logFileName_global_buf_size, // Size of buffer
                           bool& sdCardAvailable_global_ref,
                           bool& loggingEnabled_global_ref,
                           bool& sdCardMounted_global_ref,
                           bool& sdCardPresent_global_ref) {
  Serial.println(F("\n--- Attempting to Start Logging ---"));

  Serial.println(F("Step 1: Initializing SD card..."));
  sdCardAvailable_global_ref = initSDCard(sd_obj, sdCardMounted_global_ref, sdCardPresent_global_ref);

  if (sdCardAvailable_global_ref) {
    Serial.println(F("SUCCESS: SD card initialized."));
    Serial.println(F("Step 2: Attempting to create a new log file..."));
    if (createNewLogFile(sd_obj, gnss_obj, logFile_obj, logFileName_global_buf, logFileName_global_buf_size)) {
      loggingEnabled_global_ref = true;
      Serial.print(F("SUCCESS: Logging started. Current log file: "));
      Serial.println(logFileName_global_buf);
    } else {
      loggingEnabled_global_ref = false;
      Serial.println(F("ERROR: Failed to create a new log file. Logging remains disabled."));
    }
  } else {
    loggingEnabled_global_ref = false;
    Serial.println(F("ERROR: Failed to initialize SD card. Logging cannot be started."));
  }
  Serial.println(F("--- Logging Attempt Complete ---"));
}

void printStorageStatistics(const SystemStatusContext& statusCtx, SdFat& sd_obj_ref) {
  Serial.println(F("\n----- STORAGE STATISTICS -----"));

  Serial.print(F("SD Card: "));
  if (statusCtx.sdCardAvailable) {
    uint32_t fs_used_space = 0;
    FsFile fs_root = sd_obj_ref.open("/"); // Use passed SdFat object
    if (fs_root) {
        while (true) {
            FsFile fs_entry = fs_root.openNextFile();
            if (!fs_entry) break;
            fs_used_space += fs_entry.size();
            fs_entry.close();
        }
        fs_root.close();
    } else {
        Serial.println(F("Error opening root directory for stats."));
    }

    Serial.print(fs_used_space / 1024L);
    Serial.print(F(" KB used"));

    if (statusCtx.logFileName && strlen(statusCtx.logFileName) > 0 && statusCtx.loggingEnabled) {
      Serial.print(F(" | Log: "));
      Serial.println(statusCtx.logFileName);
    } else {
      Serial.println();
    }
  } else {
    Serial.println(F("Not available or error reading."));
  }

  Serial.println(F("-----------------------------"));
}

void toggleDebugFlag(volatile bool& flag_to_toggle, const __FlashStringHelper* name, Stream& output, int specificState) {
  if (specificState == 0) {
    flag_to_toggle = false;
  } else if (specificState == 1) {
    flag_to_toggle = true;
  } else {
    flag_to_toggle = !flag_to_toggle;
  }
  output.print(name);
  output.print(F(": "));
  output.println(flag_to_toggle ? F("ON") : F("OFF"));

  if (name == F("GPS debug")) {
      setGPSDebugging(flag_to_toggle);
  }
}

void performCalibration(bool& baroCalibrated_ref_out, Adafruit_NeoPixel& pixels_ref, MS5611& baro_ref, SFE_UBLOX_GNSS& gps_ref) {
    if (!baroCalibrated_ref_out) {
        pixels_ref.setPixelColor(0, pixels_ref.Color(50, 0, 50));
        pixels_ref.show();
        Serial.println(F("Starting barometric calibration with GPS..."));
        Serial.println(F("Waiting for good GPS fix (pDOP < 3.0)..."));

        // ms5611_calibrate_with_gps needs to be updated to take baro_ref and gps_ref if it uses them directly
        // Or, it continues to use globals. For now, assume it uses globals or references passed to it.
        if (ms5611_calibrate_with_gps(30000)) { // This function needs baro and GPS objects.
            Serial.println(F("Barometric calibration successful!"));
            baroCalibrated_ref_out = true;
            pixels_ref.setPixelColor(0, pixels_ref.Color(0, 50, 0));
            pixels_ref.show();
            delay(1000);
        } else {
            Serial.println(F("Barometric calibration timed out or failed."));
            pixels_ref.setPixelColor(0, pixels_ref.Color(50, 0, 0));
            pixels_ref.show();
            delay(1000);
        }
    } else {
        Serial.println(F("Barometric calibration has already been performed."));
    }
}

void printSystemStatus(const SystemStatusContext& statusCtx) {
    Serial.println("System Status:");
    Serial.print("SD Card: "); Serial.println(statusCtx.sdCardAvailable ? "Available" : "Not Available");
    Serial.print("External Flash: "); Serial.println(statusCtx.flashAvailable ? "Available" : "Not Available");
    Serial.print("GPS: ");
    if (!g_gps_initialized_ok) {
        Serial.println("Not Initialized");
    } else if (GPS_fixType > 0) {
        Serial.println("Fix Type " + String(GPS_fixType) + " (" + String(SIV) + " sats)");
    } else {
        Serial.println("Initialized, No Fix");
    }
    Serial.print("Barometer (MS5611): "); Serial.println(statusCtx.ms5611_initialized_ok ? (statusCtx.ms5611Sensor_ref.isConnected() ? "Connected" : "Init OK, Not Connected") : "Not Initialized");
    Serial.print("Accelerometer (KX134): "); Serial.println(statusCtx.kx134_initialized_ok ? "Initialized" : "Not Initialized");
    Serial.print("IMU (ICM20948): "); Serial.println(statusCtx.icm20948_ready ? "Ready" : "Not Ready");
    // Display last error code if one is set
    if (g_last_error_code != NO_ERROR) {
        Serial.print("Last Error Code: ");
        Serial.print(static_cast<int>(g_last_error_code));
        Serial.print(" (");
        Serial.print(getErrorCodeName(g_last_error_code)); // Assumes getErrorCodeName is available globally or via include
        Serial.println(")");
    }
}

void printHelpMessage(const DebugFlags& debugFlags) { // Signature already updated
  Serial.println(F("=== TRIPLET FLIGHT COMMAND MENU ==="));
  Serial.println(F("Current Debug Flag States:"));
  Serial.print(F("  System Debug        (1, debug_system [on|off]): ")); Serial.println(debugFlags.enableSystemDebug ? F("ON") : F("OFF"));
  Serial.print(F("  IMU Debug           (2, debug_imu [on|off]): ")); Serial.println(debugFlags.enableIMUDebug ? F("ON") : F("OFF"));
  Serial.print(F("  GPS Debug           (3, debug_gps [on|off]): ")); Serial.println(debugFlags.enableGPSDebug ? F("ON") : F("OFF"));
  Serial.print(F("  Barometer Debug     (4, debug_baro [on|off]): ")); Serial.println(debugFlags.enableBaroDebug ? F("ON") : F("OFF"));
  Serial.print(F("  Storage Debug       (5, debug_storage [on|off]): ")); Serial.println(debugFlags.enableStorageDebug ? F("ON") : F("OFF"));
  Serial.print(F("  ICM Raw Debug       (6, debug_icm_raw [on|off]): ")); Serial.println(debugFlags.enableICMRawDebug ? F("ON") : F("OFF"));
  Serial.print(F("  Serial CSV Output   (0, debug_serial_csv [on|off]): ")); Serial.println(debugFlags.enableSerialCSV ? F("ON") : F("OFF"));
  Serial.print(F("  Sensor Detail Debug (sd, debug_sensor_detail [on|off]): ")); Serial.println(debugFlags.enableSensorDebug ? F("ON") : F("OFF"));
  Serial.print(F("  Status Summary      (j, summary, debug_status_summary [on|off]): ")); Serial.println(debugFlags.enableStatusSummary ? F("ON") : F("OFF"));
  Serial.print(F("  Detailed Display    (g, debug_detailed_display [on|off]): ")); Serial.println(debugFlags.displayMode ? F("ON") : F("OFF"));
  Serial.print(F("  Battery Debug       (debug_battery [on|off]): ")); Serial.println(debugFlags.enableBatteryDebug ? F("ON") : F("OFF"));

  Serial.println(F("\nSingle-Key Commands:"));
  Serial.println(F("  0-6: Toggle specific debug flags"));
  Serial.println(F("  7  : Start/Restart logging ('start_log')"));
  Serial.println(F("  8  : SD card status ('sd_status')"));
  Serial.println(F("  9  : Initiate Shutdown"));
  Serial.println(F("  a  : Show this help message ('help')"));
  Serial.println(F("  b  : System component status ('status')"));
  Serial.println(F("  c  : Dump data (Flash - NI)"));
  Serial.println(F("  d  : Erase flash (Flash - NI)"));
  Serial.println(F("  e  : List logs (Flash - NI)"));
  Serial.println(F("  f  : SD card statistics"));
  Serial.println(F("  g  : Toggle detailed display mode"));
  Serial.println(F("  h  : Calibrate barometer ('calibrate')"));
  Serial.println(F("  i  : Display IMU data (ICM20948)"));
  Serial.println(F("  j  : Toggle status summary display"));

  Serial.println(F("\nMulti-Character Commands:"));
  Serial.println(F("  start_log"));
  Serial.println(F("  sd_status"));
  Serial.println(F("  debug_<flag> [on|off] (system, imu, gps, baro, storage, icm_raw, serial_csv, sensor_detail, status_summary, detailed_display, battery)"));
  Serial.println(F("  debug_all_off"));
  Serial.println(F("  help"));
  Serial.println(F("  calibrate"));
  Serial.println(F("  calibrate_mag"));
  Serial.println(F("  calibrate_gyro"));
  Serial.println(F("  save_mag_cal"));
  Serial.println(F("  status"));
  Serial.println(F("  summary"));
  Serial.println(F("  set_orientation_filter [madgwick|kalman]"));
  Serial.println(F("  get_orientation_filter"));
  Serial.println(F("  arm"));
  Serial.println(F("  clear_errors"));
  Serial.println(F("  clear_to_calibration"));
  Serial.println(F("  sensor_requirements"));
  Serial.println(F("  scan_i2c"));

  Serial.println(F("\nLegacy Commands:"));
  Serial.println(F("  sd  (toggle sensor_detail_debug)"));
  Serial.println(F("  rd  (toggle icm_raw_debug)"));
  Serial.println(F("================================="));
}

void prepareForShutdown(Adafruit_NeoPixel& pixels_ref, FsFile& logFile_to_close_ref) {
  Serial.println(F("Preparing for shutdown..."));
  // Call utility function to close files; this utility is declared in command_processor.h
  // and defined in TripleT_Flight_Firmware.cpp
  closeAllFiles(logFile_to_close_ref);
  pixels_ref.setPixelColor(0, pixels_ref.Color(0, 0, 50));
  pixels_ref.show();
  tone(BUZZER_PIN, 1000); delay(100); noTone(BUZZER_PIN);
  Serial.println(F("System shutdown complete. Looping indefinitely."));
  while (true) {
    delay(5000);
  }
}

void setOrientationFilter(const char* filterType, SystemStatusContext& statusCtx) {
    if (strcasecmp(filterType, "madgwick") == 0) {
        Serial.println(F("WARNING: Madgwick filter no longer available. Kalman filter is the only option."));
        *statusCtx.useKalmanFilter_ptr = true;
    } else if (strcasecmp(filterType, "kalman") == 0) {
        *statusCtx.useKalmanFilter_ptr = true;
        kalman_init(0.0f, 0.0f, 0.0f); // Re-initialize Kalman filter with zero initial angles
        Serial.println(F("Kalman filter enabled (default and only option)."));
    } else if (strlen(filterType) > 0){ // Only print if a filter type was actually given
        Serial.print(F("Unknown filter type: "));
        Serial.println(filterType);
        Serial.println(F("Only 'kalman' is supported."));
    }
}

void getOrientationFilterStatus(const SystemStatusContext& statusCtx) {
    Serial.print(F("Current orientation filter: "));
    if (statusCtx.useKalmanFilter_ptr && *statusCtx.useKalmanFilter_ptr) {
        Serial.println(F("Kalman (only option)"));
    } else {
        Serial.println(F("Disabled (not recommended - Kalman is the only filter available)"));
    }
    Serial.print(F("  useKalmanFilter flag: "));
    Serial.println((statusCtx.useKalmanFilter_ptr && *statusCtx.useKalmanFilter_ptr) ? "true" : "false");
}

void processCommand(const char* command,
                    FlightState& currentFlightState_ref,
                    FlightState& previousFlightState_ref,
                    unsigned long& stateEntryTime_ref,
                    SystemStatusContext& statusCtx,  // Remove const qualifier
                    DebugFlags& debugFlags,
                    // Objects/refs for functions called by processCommand:
                    SdFat& sd_obj_ref_param,
                    SFE_UBLOX_GNSS& gnss_obj_ref_param,
                    FsFile& logfile_obj_ref_param,
                    char* logfilename_buf_global_param,
                    size_t logfilename_buf_size_param,
                    bool& sd_avail_global_ref_param,
                    bool& logging_en_global_ref_param,
                    bool& sd_mounted_global_ref_param,
                    bool& sd_present_global_ref_param,
                    uint64_t& available_space_global_ref_param,
                    Adafruit_NeoPixel& pixels_ref_param,
                    bool& baroCalibrated_ref,
                    MS5611& baro_ref
                    ) {
    // Skip empty commands
    if (command == NULL || strlen(command) == 0) return;

    // Skip leading whitespace manually if needed, though usually handled by caller or basic trimming
    while(*command == ' ') command++;
    if (strlen(command) == 0) return;

    // Note: 'strcasecmp' is case-insensitive comparison available in standard C library for GCC/Arduino

    if (strlen(command) == 1) {
        char cmd = command[0]; // Already lowercased? No, caller might send raw.
        // Actually, let's just inspect the char.
        if (cmd >= 'A' && cmd <= 'Z') cmd += 32; // manual toLowerCase

        if (cmd >= '0' && cmd <= '9') {
            switch (cmd) {
                case '0': toggleDebugFlag(debugFlags.enableSerialCSV, F("Serial CSV output"), Serial); break;
                case '1': toggleDebugFlag(debugFlags.enableSystemDebug, F("System debug"), Serial); break;
                case '2': toggleDebugFlag(debugFlags.enableIMUDebug, F("IMU debug"), Serial); break;
                case '3': toggleDebugFlag(debugFlags.enableGPSDebug, F("GPS debug"), Serial); break;
                case '4': toggleDebugFlag(debugFlags.enableBaroDebug, F("Barometer debug"), Serial); break;
                case '5': toggleDebugFlag(debugFlags.enableStorageDebug, F("Storage debug"), Serial); break;
                case '6': toggleDebugFlag(debugFlags.enableICMRawDebug, F("ICM raw debug"), Serial); break;
                case '7':
                    attemptToStartLogging(sd_obj_ref_param, gnss_obj_ref_param, logfile_obj_ref_param,
                                          logfilename_buf_global_param, logfilename_buf_size_param,
                                          sd_avail_global_ref_param, logging_en_global_ref_param,
                                          sd_mounted_global_ref_param, sd_present_global_ref_param);
                    break;
                case '8': printSDCardStatus(statusCtx, sd_obj_ref_param, available_space_global_ref_param); break;
                case '9': prepareForShutdown(pixels_ref_param, logfile_obj_ref_param); break;
                default: Serial.println(F("Unknown numeric command.")); break;
            }
            return;
        }

        if (cmd >= 'a' && cmd <= 'k') {
            switch (cmd) {
                case 'a': printHelpMessage(debugFlags); break;
                case 'b': printSystemStatus(statusCtx); break;
                case 'c': Serial.println(F("Dump command (Flash - Not Implemented)")); break;
                case 'd': Serial.println(F("Erase flash command (Flash - Not Implemented)")); break;
                case 'e': Serial.println(F("List logs command (Flash - Not Implemented)")); break;
                case 'f': printStorageStatistics(statusCtx, sd_obj_ref_param); break;
                case 'g': toggleDebugFlag(debugFlags.displayMode, F("Detailed display mode"), Serial); break;
                case 'h': 
                    performCalibration(baroCalibrated_ref, pixels_ref_param, baro_ref, gnss_obj_ref_param);
                    break;
                case 'i': if(statusCtx.icm20948_ready) ICM_20948_print(); else Serial.println(F("ICM20948 not ready.")); break;
                case 'j': toggleDebugFlag(debugFlags.enableStatusSummary, F("Status summary"), Serial); break;
                default: Serial.println(F("Unknown alphabetic command.")); break;
            }
            return;
        }
    }

    if (strcasecmp(command, "help") == 0) printHelpMessage(debugFlags);
    else if (strcasecmp(command, "calibrate") == 0) {
        if (currentFlightState_ref == PAD_IDLE || currentFlightState_ref == CALIBRATION || currentFlightState_ref == ERROR) {
             performCalibration(baroCalibrated_ref, pixels_ref_param, baro_ref, gnss_obj_ref_param);
        } else {
             Serial.println(F("ERROR: Calibration only allowed in PAD_IDLE, CALIBRATION or ERROR state."));
        }
    }
    else if (strcasecmp(command, "calibrate_mag") == 0) { if(statusCtx.icm20948_ready) ICM_20948_calibrate_mag_interactive(); else Serial.println(F("ICM20948 not ready for mag cal.")); }
    else if (strcasecmp(command, "calibrate_gyro") == 0) { if(statusCtx.icm20948_ready) ICM_20948_calibrate_gyro_bias(2000, 1); else Serial.println(F("ICM20948 not ready for gyro cal."));}
    else if (strcasecmp(command, "save_mag_cal") == 0) { if(statusCtx.icm20948_ready) icm_20948_save_calibration(); else Serial.println(F("ICM20948 not ready to save mag cal."));}
    else if (strcasecmp(command, "arm") == 0) {
        if (currentFlightState_ref == PAD_IDLE) {
            Serial.println(F("Attempting to arm system. Checking health for ARMED state..."));
            // Pass currentFlightState_ref (which is PAD_IDLE) to isSensorSuiteHealthy, but check for ARMED requirements.
            // The isSensorSuiteHealthy function uses its first argument to determine context for some checks.
            // However, for critical failures like barometer not initialized, it should fail if ARMED is the target.
            // Let's explicitly check requirements for ARMED state.
            if (isSensorSuiteHealthy(ARMED, true)) { // Check health requirements for ARMED state
                previousFlightState_ref = currentFlightState_ref;
                currentFlightState_ref = ARMED;
                stateEntryTime_ref = millis();
                saveStateToEEPROM(); // Assumes saveStateToEEPROM uses the global currentFlightState or is passed the ref
                Serial.println(F("System ARMED. Ready for launch."));
                if (!debugFlags.enableSerialCSV) {
                    Serial.println(F("Note: Serial CSV output is off. Web UI will not update until it is enabled ('0')."));
                }
            } else {
                Serial.println(F("Cannot arm. System health check failed for ARMED state. See details above."));
                Serial.println(F("System remains in PAD_IDLE state."));
            }
        } else {
            Serial.print(F("Cannot arm. System is not in PAD_IDLE state. Current state: "));
            Serial.println(getStateName(currentFlightState_ref));
        }
    }
    else if (strcasecmp(command, "clear_errors") == 0) {
        if (currentFlightState_ref == ERROR) {
            Serial.println(F("Attempting to clear error state..."));
            
            // First, let's check what specifically is failing
            Serial.println(F("Checking system health for PAD_IDLE state:"));
            bool healthy = isSensorSuiteHealthy(PAD_IDLE, true); // Call with verbose=true to see details
            
            if (healthy) {
                previousFlightState_ref = currentFlightState_ref;
                currentFlightState_ref = PAD_IDLE;
                stateEntryTime_ref = millis();
                saveStateToEEPROM(); // Assumes saveStateToEEPROM uses the global currentFlightState or is passed the ref
                Serial.println(F("Error state cleared. System reset to PAD_IDLE. Check sensors."));
            } else {
                Serial.println(F("Cannot clear error: System health check for PAD_IDLE failed."));
                Serial.println(F(""));
                Serial.println(F("Troubleshooting steps:"));
                Serial.println(F("1. Check if barometer needs calibration: use 'calibrate' or 'h' command"));
                Serial.println(F("2. Check sensor status: use 'status_sensors' or 'b' command"));
                Serial.println(F("3. Verify IMU initialization: at least one of ICM20948 or KX134 must be ready"));
                Serial.println(F("4. If barometer is the issue, try transitioning to CALIBRATION state first"));
                Serial.println(F(""));
                
                // Offer alternative: clear to CALIBRATION state if barometer is the main issue
                if (!statusCtx.baroCalibrated && statusCtx.ms5611_initialized_ok) {
                    Serial.println(F("Alternative: Barometer is initialized but not calibrated."));
                    Serial.println(F("Would you like to clear to CALIBRATION state instead? (Type 'clear_to_calibration')"));
                }
            }
        } else {
            Serial.print(F("System is not in ERROR state. Current state: "));
            Serial.println(getStateName(currentFlightState_ref));
        }
    }
    else if (strcasecmp(command, "clear_to_calibration") == 0) {
        if (currentFlightState_ref == ERROR) {
            Serial.println(F("Attempting to clear error state to CALIBRATION..."));
            
            // Check minimal requirements for CALIBRATION state (just barometer initialized)
            if (statusCtx.ms5611_initialized_ok) {
                previousFlightState_ref = currentFlightState_ref;
                currentFlightState_ref = CALIBRATION;
                stateEntryTime_ref = millis();
                saveStateToEEPROM();
                Serial.println(F("Error state cleared. System reset to CALIBRATION state."));
                Serial.println(F("Use 'calibrate' or 'h' command to calibrate barometer with GPS."));
            } else {
                Serial.println(F("Cannot clear to CALIBRATION: Barometer (MS5611) not initialized."));
                Serial.println(F("Check hardware connections and restart system."));
            }
        } else {
            Serial.print(F("System is not in ERROR state. Current state: "));
            Serial.println(getStateName(currentFlightState_ref));
        }
    }
    else if (strcasecmp(command, "summary") == 0) toggleDebugFlag(debugFlags.enableStatusSummary, F("Status summary"), Serial);
    else if (strcasecmp(command, "status") == 0) printSystemStatus(statusCtx);
    else if (strcasecmp(command, "sd_status") == 0) printSDCardStatus(statusCtx, sd_obj_ref_param, available_space_global_ref_param);
    else if (strcasecmp(command, "start_log") == 0) {
        attemptToStartLogging(sd_obj_ref_param, gnss_obj_ref_param, logfile_obj_ref_param,
                              logfilename_buf_global_param, logfilename_buf_size_param,
                              sd_avail_global_ref_param, logging_en_global_ref_param,
                              sd_mounted_global_ref_param, sd_present_global_ref_param);
    }
    else if (strcasecmp(command, "sd") == 0) toggleDebugFlag(debugFlags.enableSensorDebug, F("Sensor detail debug"), Serial);
    else if (strcasecmp(command, "rd") == 0) toggleDebugFlag(debugFlags.enableICMRawDebug, F("ICM raw debug"), Serial);
    else if (strncasecmp(command, "debug_", 6) == 0) {
        // Handle debug_ flags manually
        const char* flagPart = command + 6; // Skip "debug_"
        int specificState = -1;
        
        // Check for suffix " on" or " off"
        size_t len = strlen(flagPart);
        char flagIdentifier[32]; 
        
        if (len < sizeof(flagIdentifier)) {
            strcpy(flagIdentifier, flagPart);
            
            // Check for " on"
            if (len > 3 && strcasecmp(flagIdentifier + len - 3, " on") == 0) {
                specificState = 1;
                flagIdentifier[len-3] = '\0';
            }
            // Check for " off"
            else if (len > 4 && strcasecmp(flagIdentifier + len - 4, " off") == 0) {
                specificState = 0;
                flagIdentifier[len-4] = '\0';
            }
            
            // Now match flagIdentifier
            if (strcasecmp(flagIdentifier, "system") == 0) toggleDebugFlag(debugFlags.enableSystemDebug, F("System debug"), Serial, specificState);
            else if (strcasecmp(flagIdentifier, "imu") == 0) toggleDebugFlag(debugFlags.enableIMUDebug, F("IMU debug"), Serial, specificState);
            else if (strcasecmp(flagIdentifier, "gps") == 0) toggleDebugFlag(debugFlags.enableGPSDebug, F("GPS debug"), Serial, specificState);
            else if (strcasecmp(flagIdentifier, "baro") == 0) toggleDebugFlag(debugFlags.enableBaroDebug, F("Barometer debug"), Serial, specificState);
            else if (strcasecmp(flagIdentifier, "storage") == 0) toggleDebugFlag(debugFlags.enableStorageDebug, F("Storage debug"), Serial, specificState);
            else if (strcasecmp(flagIdentifier, "icm_raw") == 0) toggleDebugFlag(debugFlags.enableICMRawDebug, F("ICM raw debug"), Serial, specificState);
            else if (strcasecmp(flagIdentifier, "serial_csv") == 0) toggleDebugFlag(debugFlags.enableSerialCSV, F("Serial CSV output"), Serial, specificState);
            else if (strcasecmp(flagIdentifier, "sensor_detail") == 0) toggleDebugFlag(debugFlags.enableSensorDebug, F("Sensor detail debug"), Serial, specificState);
            else if (strcasecmp(flagIdentifier, "status_summary") == 0) toggleDebugFlag(debugFlags.enableStatusSummary, F("Status summary"), Serial, specificState);
            else if (strcasecmp(flagIdentifier, "detailed_display") == 0) toggleDebugFlag(debugFlags.displayMode, F("Detailed display mode"), Serial, specificState);
            else if (strcasecmp(flagIdentifier, "battery") == 0) toggleDebugFlag(debugFlags.enableBatteryDebug, F("Battery debug"), Serial, specificState);
            else if (strcasecmp(flagIdentifier, "all_off") == 0) {
                if (specificState == 0 || specificState == -1) {
                    Serial.println(F("Disabling all common debug flags:"));
                    toggleDebugFlag(debugFlags.enableSerialCSV, F("Serial CSV output"), Serial, 0);
                    toggleDebugFlag(debugFlags.enableSystemDebug, F("System debug"), Serial, 0);
                    toggleDebugFlag(debugFlags.enableIMUDebug, F("IMU debug"), Serial, 0);
                    toggleDebugFlag(debugFlags.enableGPSDebug, F("GPS debug"), Serial, 0);
                    toggleDebugFlag(debugFlags.enableBaroDebug, F("Barometer debug"), Serial, 0);
                    toggleDebugFlag(debugFlags.enableStorageDebug, F("Storage debug"), Serial, 0);
                    toggleDebugFlag(debugFlags.enableICMRawDebug, F("ICM raw debug"), Serial, 0);
                    toggleDebugFlag(debugFlags.enableStatusSummary, F("Status summary"), Serial, 0);
                    toggleDebugFlag(debugFlags.displayMode, F("Detailed display mode"), Serial, 0);
                    toggleDebugFlag(debugFlags.enableSensorDebug, F("Sensor detail debug"), Serial, 0);
                    toggleDebugFlag(debugFlags.enableBatteryDebug, F("Battery debug"), Serial, 0);
                    debugFlags.enableDetailedOutput = false;
                    Serial.println(F("Legacy Detailed output (global): OFF"));
                } else {
                    Serial.println(F("debug_all_off only supports 'off' or toggle to off."));
                }
            } else { Serial.print(F("Unknown debug flag: ")); Serial.println(flagIdentifier); }
        }
    }
    else if (strncasecmp(command, "set_orientation_filter ", 23) == 0) {
        const char* filterType = command + 23; // pointer arithmetic
        // Trimming is harder with raw pointers, but let's assume valid spacing
        while(*filterType == ' ') filterType++;
        setOrientationFilter(filterType, statusCtx);
    }
    else if (strcasecmp(command, "get_orientation_filter") == 0) {
        getOrientationFilterStatus(statusCtx);
    }
    else if (strcasecmp(command, "scan_i2c") == 0) {
        scan_i2c();
    }
    else if (strcasecmp(command, "sensor_requirements") == 0) {
        Serial.println(F("=== Sensor Requirements by Flight State ==="));
        Serial.println(F(""));
        Serial.println(F("STARTUP:"));
        Serial.println(F("  • No specific sensor requirements"));
        Serial.println(F(""));
        Serial.println(F("CALIBRATION:"));
        Serial.println(F("  • MS5611 Barometer: Must be initialized"));
        Serial.println(F("  • GPS: Recommended for barometer calibration"));
        Serial.println(F(""));
        Serial.println(F("PAD_IDLE:"));
        Serial.println(F("  • MS5611 Barometer: Must be initialized AND calibrated"));
        Serial.println(F("  • IMU: Not required (can be armed without IMU)"));
        Serial.println(F(""));
        Serial.println(F("ARMED through LANDED:"));
        Serial.println(F("  • MS5611 Barometer: Must be initialized AND calibrated"));
        Serial.println(F("  • IMU: At least one of ICM20948 OR KX134 must be ready"));
        Serial.println(F("  • GPS: Recommended but not required"));
        Serial.println(F(""));
        Serial.println(F("RECOVERY:"));
        Serial.println(F("  • All sensors recommended for data logging"));
        Serial.println(F(""));
        Serial.println(F("ERROR:"));
        Serial.println(F("  • System enters ERROR when critical sensors fail"));
        Serial.println(F("  • Use 'clear_errors' to attempt recovery to PAD_IDLE"));
        Serial.println(F("  • Use 'clear_to_calibration' if barometer needs calibration"));
        Serial.println(F("==========================================="));
    }
    else if (strcasecmp(command, "TEST_FREEZE") == 0) {
        Serial.println(F("Freezing system for 6 seconds (Watchdog should trigger)..."));
        delay(6000); // Exceeds 5s watchdog timeout
    }
    else {
        Serial.print(F("Unknown command: "));
        Serial.println(command);
    }
}
