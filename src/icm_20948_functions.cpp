#include "icm_20948_functions.h"

// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

// Create an ICM_20948_I2C object
ICM_20948_I2C myICM;

// Initialize the ICM-20948 sensor
void ICM_20948_init() {
    Serial.println(F("ICM-20948 Example"));
    myICM.begin(Wire, AD0_VAL);
    myICM.enableDebugging();

    bool ICM_20948_initialized = false;
    while (!ICM_20948_initialized) {
        Serial.print(F("Initialization of the sensor returned: "));
        Serial.println(myICM.statusString());
        if (myICM.status != ICM_20948_Stat_Ok) {
            Serial.println(F("Trying again..."));
            delay(250);
        } else {
            ICM_20948_initialized = true;
        }
    }
    Serial.println(F("Device connected!"));

    // Here we are doing a SW reset to make sure the device starts in a known state
    myICM.swReset();
    if (myICM.status != ICM_20948_Stat_Ok) {
        Serial.print(F("Software Reset returned: "));
        Serial.println(myICM.statusString());
    }
    delay(250);

    // Now wake the sensor up
    myICM.sleep(false);
    myICM.lowPower(false);

    // Set Gyro and Accelerometer to continuous sample mode
    myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
    if (myICM.status != ICM_20948_Stat_Ok) {
        Serial.print(F("setSampleMode returned: "));
        Serial.println(myICM.statusString());
    }
    
    // Set full scale ranges for both acc and gyr
    ICM_20948_fss_t myFSS;
    myFSS.a = gpm16;  // Accelerometer full scale range
    myFSS.g = dps2000;  // Gyroscope full scale range

    myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
    if (myICM.status != ICM_20948_Stat_Ok) {
        Serial.print(F("setFullScale returned: "));
        Serial.println(myICM.statusString());
    }

    // Set up Digital Low-Pass Filter configuration
    ICM_20948_dlpcfg_t myDLPcfg;
    myDLPcfg.a = acc_d473bw_n499bw;
    myDLPcfg.g = gyr_d361bw4_n376bw5;

    myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
    if (myICM.status != ICM_20948_Stat_Ok) {
        Serial.print(F("setDLPcfg returned: "));
        Serial.println(myICM.statusString());
    }

    // Choose whether or not to use DLPF
    ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Acc, false);
    ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Gyr, false);
    Serial.print(F("Enable DLPF for Accelerometer returned: "));
    Serial.println(myICM.statusString(accDLPEnableStat));
    Serial.print(F("Enable DLPF for Gyroscope returned: "));
    Serial.println(myICM.statusString(gyrDLPEnableStat));

    // Start the magnetometer
    myICM.startupMagnetometer();
    if (myICM.status != ICM_20948_Stat_Ok) {
        Serial.print(F("startupMagnetometer returned: "));
        Serial.println(myICM.statusString());
    }

    Serial.println();
    Serial.println(F("Configuration complete!"));

    // After successful ICM initialization
    delay(500); // Wait for sensor to stabilize
}

// Read data from the ICM-20948 sensor
void ICM_20948_read() {
    if (myICM.dataReady()) {
        myICM.getAGMT();  // The values are only updated when you call 'getAGMT'
        
        // Convert from mg to g and store in arrays
        icm_accel[0] = myICM.accX() / 1000.0;  // Convert from mg to g
        icm_accel[1] = myICM.accY() / 1000.0;  // Convert from mg to g
        icm_accel[2] = myICM.accZ() / 1000.0;  // Convert from mg to g

        // Store gyroscope data (already in degrees per second)
        icm_gyro[0] = myICM.gyrX();
        icm_gyro[1] = myICM.gyrY();
        icm_gyro[2] = myICM.gyrZ();

        // Store magnetometer data (already in microteslas)
        icm_mag[0] = myICM.magX();
        icm_mag[1] = myICM.magY();
        icm_mag[2] = myICM.magZ();

        // Store temperature data (already in degrees Celsius)
        icm_temp = myICM.temp();

        icm_data_available = true;
        delay(30);
    } else {
        Serial.println("Waiting for data");
        icm_data_available = false;
        delay(500);
    }
}

// Print ICM-20948 data to serial
void ICM_20948_print() {
    if (myICM.dataReady()) {
        Serial.print("Acc ");
        Serial.print((icm_accel[0]),3);
        Serial.print(", ");
        Serial.print((icm_accel[1]),3);
        Serial.print(", ");
        Serial.print((icm_accel[2]),3);
        Serial.print(" Gyr (DPS) ");
        Serial.print((icm_gyro[0]),3);
        Serial.print(", ");
        Serial.print((icm_gyro[1]),3);
        Serial.print(", ");
        Serial.print((icm_gyro[2]),3);
        Serial.print(" Mag (uT) ");
        Serial.print(icm_mag[2], 3);
        Serial.print(", ");
        Serial.print(icm_mag[2], 3);
        Serial.print(", ");
        Serial.print(icm_mag[2], 3);
        Serial.print(" Tmp (C) ");
        Serial.println(icm_temp, 2);
        delay(30);
    } else {
        Serial.println("Waiting for data");
        delay(500);
    }

} 