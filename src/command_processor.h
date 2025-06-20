#ifndef COMMAND_PROCESSOR_H
#define COMMAND_PROCESSOR_H

#include <Arduino.h>
#include "data_structures.h" // For FlightState and SFE_UBLOX_GNSS if it's in there
#include "gps_functions.h" // For SFE_UBLOX_GNSS if not in data_structures.h
#include "debug_flags.h"   // Include debug flags structure
#include "flight_context.h" // Include flight context structure

// Forward declarations for global variables used directly by processCommand or other functions.
extern FlightState currentFlightState; // Passed directly to processCommand
extern FlightState previousFlightState; // Passed directly to processCommand
extern unsigned long stateEntryTime;    // Passed directly to processCommand

// extern bool sdCardAvailable; // Moved to SystemStatusContext
// extern bool flashAvailable; // Moved to SystemStatusContext
// extern SFE_UBLOX_GNSS myGNSS; // Referenced via SystemStatusContext
// extern bool baroCalibrated; // Moved to SystemStatusContext

// extern bool icm20948_ready; // Moved to SystemStatusContext
// extern bool ms5611_initialized_ok; // Moved to SystemStatusContext
// extern bool kx134_initialized_ok; // Moved to SystemStatusContext

// extern bool useMadgwickFilter; // Moved to SystemStatusContext (pointer)
// extern bool useKalmanFilter; // Moved to SystemStatusContext (pointer)

// Externs for objects/globals still directly used by some command_processor functions
// if not passed via context or directly.
// These should be progressively eliminated.
// extern Adafruit_NeoPixel pixels; // Now passed to processCommand
// extern SdFat SD;                 // Now passed to processCommand
// extern char logFileName[];       // Moved to SystemStatusContext
// extern uint64_t availableSpace;  // Moved to SystemStatusContext
// extern bool loggingEnabled;      // Moved to SystemStatusContext
// extern bool sdCardMounted;       // Moved to SystemStatusContext
// extern bool sdCardPresent;       // Moved to SystemStatusContext


// Function prototypes
void processCommand(String command,
                    FlightState& currentFlightState_ref,
                    FlightState& previousFlightState_ref,
                    unsigned long& stateEntryTime_ref,
                    const SystemStatusContext& statusCtx,
                    DebugFlags& debugFlags,
                    // Objects/refs for functions called by processCommand:
                    SdFat& sd_obj_ref_param,
                    SFE_UBLOX_GNSS& gnss_obj_ref_param,
                    FsFile& logfile_obj_ref_param,         // For attemptToStartLogging & prepareForShutdown
                    char* logfilename_buf_global_param,
                    size_t logfilename_buf_size_param,
                    bool& sd_avail_global_ref_param,         // For attemptToStartLogging
                    bool& logging_en_global_ref_param,       // For attemptToStartLogging
                    bool& sd_mounted_global_ref_param,     // For attemptToStartLogging
                    bool& sd_present_global_ref_param,     // For attemptToStartLogging
                    uint64_t& available_space_global_ref_param, // For printSDCardStatus to refresh
                    Adafruit_NeoPixel& pixels_ref_param // Added
                    );

void printHelpMessage(const DebugFlags& debugFlags); // Stays same

void printSDCardStatus(const SystemStatusContext& statusCtx, SdFat& sd_obj_ref, uint64_t& availableSpace_refresh_ref);

void attemptToStartLogging(SdFat& sd_obj,
                           SFE_UBLOX_GNSS& gnss_obj,
                           FsFile& logFile_obj,
                           char* logFileName_global_buf, // Buffer for log file name
                           size_t logFileName_global_buf_size, // Size of buffer
                           bool& sdCardAvailable_global_ref,
                           bool& loggingEnabled_global_ref,
                           bool& sdCardMounted_global_ref,
                           bool& sdCardPresent_global_ref);


void printStorageStatistics(const SystemStatusContext& statusCtx, SdFat& sd_obj_ref);

void toggleDebugFlag(volatile bool& flag_to_toggle, const __FlashStringHelper* name, Stream& output, int specificState = -1); // Stays same

// For performCalibration, MS5611 and SFE_UBLOX_GNSS types are needed from their respective headers included in flight_context.h
void performCalibration(bool& baroCalibrated_ref_out, Adafruit_NeoPixel& pixels_ref, MS5611& baro_ref, SFE_UBLOX_GNSS& gps_ref);

void printSystemStatus(const SystemStatusContext& statusCtx);

void prepareForShutdown(Adafruit_NeoPixel& pixels_ref, FsFile& logFile_to_close_ref);

void setOrientationFilter(String filterType, SystemStatusContext& statusCtx);
void getOrientationFilterStatus(const SystemStatusContext& statusCtx);

// Functions that are called by command_processor.cpp but defined in TripleT_Flight_Firmware.cpp (or elsewhere)
// These should ideally be passed as dependencies or accessed via a proper interface.

// Declarations for utility functions defined in TripleT_Flight_Firmware.cpp (or other .cpp files)
// that are called by functions in command_processor.cpp
extern bool initSDCard(SdFat& sd_obj, bool& sdCardMounted_out, bool& sdCardPresent_out);
extern bool createNewLogFile(SdFat& sd_obj, SFE_UBLOX_GNSS& gnss, FsFile& logFile_obj_ref, char* logFileName_out_buf, size_t logFileName_out_buf_size);
extern void closeAllFiles(FsFile& logFile_obj_to_close);
extern void checkStorageSpace(SdFat& sd_obj_ref, uint64_t& availableSpace_out_ref);

// ICM_20948 functions are declared in "icm_20948_functions.h", which should be included by command_processor.cpp
// extern void ICM_20948_print();
// extern void ICM_20948_calibrate_mag_interactive();
// extern void ICM_20948_calibrate_gyro_bias(uint16_t num_samples, uint16_t delay_ms); // Signature differs slightly (int vs uint16_t)
// extern bool icm_20948_save_calibration();


#endif // COMMAND_PROCESSOR_H
