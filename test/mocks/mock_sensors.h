#ifndef MOCK_SENSORS_H
#define MOCK_SENSORS_H

#include "../../src/sensors/imu_interface.h"
#include <math.h>
#include <string.h>

// ============================================================================
// MOCK IMU SENSOR - Synthetic data generation for testing
//
// Provides controllable sensor data for unit tests without hardware:
// - Synthetic perfect data
// - Realistic sensor noise simulation
// - Recorded flight data playback
// - Failure injection
// ============================================================================

class MockIMUSensor : public IMUInterface {
public:
  MockIMUSensor() {
    reset();
  }

  bool begin() override {
    is_initialized = true;
    return true;
  }

  bool isHealthy() override {
    if (inject_failure) {
      return false;
    }
    return is_initialized;
  }

  const char* getErrorMessage() override {
    if (inject_failure) {
      return "Injected sensor failure";
    }
    return "Mock sensor OK";
  }

  bool read() override {
    if (!is_initialized || inject_failure) {
      return false;
    }

    if (playback_mode && playback_data) {
      // Playback recorded flight data
      if (playback_index < playback_size) {
        memcpy(&current_accel, &playback_data[playback_index].accel, sizeof(float[3]));
        memcpy(&current_gyro, &playback_data[playback_index].gyro, sizeof(float[3]));
        memcpy(&current_mag, &playback_data[playback_index].mag, sizeof(float[3]));
        current_temp = playback_data[playback_index].temp;
        memcpy(&current_quat, &playback_data[playback_index].quat, sizeof(float[4]));
        playback_index++;
        return true;
      }
      return false;  // End of playback
    }

    // Synthetic data mode - add noise if configured
    if (gaussian_noise_sigma > 0) {
      addGaussianNoise();
    }

    return true;
  }

  // ========================================================================
  // SYNTHETIC DATA CONTROL
  // ========================================================================

  void setAcceleration(float ax, float ay, float az) {
    current_accel[0] = ax;
    current_accel[1] = ay;
    current_accel[2] = az;
  }

  void setRotation(float gx, float gy, float gz) {
    current_gyro[0] = gx;
    current_gyro[1] = gy;
    current_gyro[2] = gz;
  }

  void setMagnetic(float mx, float my, float mz) {
    current_mag[0] = mx;
    current_mag[1] = my;
    current_mag[2] = mz;
  }

  void setQuaternion(float qw, float qx, float qy, float qz) {
    current_quat[0] = qw;
    current_quat[1] = qx;
    current_quat[2] = qy;
    current_quat[3] = qz;
    normalizeQuaternion();
  }

  void setTemperature(float temp) {
    current_temp = temp;
  }

  // ========================================================================
  // NOISE SIMULATION
  // ========================================================================

  void addGaussianNoise(float sigma = 0.0f) {
    if (sigma <= 0) sigma = gaussian_noise_sigma;

    // Simple Gaussian approximation (sum of 12 uniform random = ~normal)
    auto gaussian_rand = [&]() {
      float sum = 0;
      for (int i = 0; i < 12; i++) {
        sum += (float)rand() / RAND_MAX;
      }
      return (sum - 6.0f) * sigma;
    };

    current_accel[0] += gaussian_rand();
    current_accel[1] += gaussian_rand();
    current_accel[2] += gaussian_rand();
    current_gyro[0] += gaussian_rand();
    current_gyro[1] += gaussian_rand();
    current_gyro[2] += gaussian_rand();
  }

  void setGaussianNoise(float sigma) {
    gaussian_noise_sigma = sigma;
  }

  // ========================================================================
  // FLIGHT DATA PLAYBACK
  // ========================================================================

  struct FlightDataPoint {
    float accel[3];   // m/s²
    float gyro[3];    // rad/s
    float mag[3];     // μT
    float temp;       // °C
    float quat[4];    // w, x, y, z
  };

  void loadFlightData(FlightDataPoint* data, size_t size) {
    playback_data = data;
    playback_size = size;
    playback_index = 0;
    playback_mode = true;
  }

  bool playNextFrame() {
    if (!playback_mode || !playback_data || playback_index >= playback_size) {
      return false;
    }
    return read();
  }

  void resetPlayback() {
    playback_index = 0;
  }

  size_t getPlaybackPosition() const {
    return playback_index;
  }

  // ========================================================================
  // FAILURE INJECTION
  // ========================================================================

  void injectFailure(bool fail = true) {
    inject_failure = fail;
  }

  void injectDataCorruption(uint8_t percentage) {
    // Randomly corrupt percentage% of readings
    if ((rand() % 100) < percentage) {
      current_accel[0] = NAN;
      current_accel[1] = NAN;
      current_accel[2] = NAN;
    }
  }

  // ========================================================================
  // SENSOR DATA ACCESS
  // ========================================================================

  float getAccelX() override { return current_accel[0]; }
  float getAccelY() override { return current_accel[1]; }
  float getAccelZ() override { return current_accel[2]; }

  float getAccelMagnitude() override {
    return sqrt(current_accel[0]*current_accel[0] +
                current_accel[1]*current_accel[1] +
                current_accel[2]*current_accel[2]);
  }

  float getGyroX() override { return current_gyro[0]; }
  float getGyroY() override { return current_gyro[1]; }
  float getGyroZ() override { return current_gyro[2]; }

  float getMagX() override { return current_mag[0]; }
  float getMagY() override { return current_mag[1]; }
  float getMagZ() override { return current_mag[2]; }

  void getQuaternion(float& qw, float& qx, float& qy, float& qz) override {
    qw = current_quat[0];
    qx = current_quat[1];
    qy = current_quat[2];
    qz = current_quat[3];
  }

  float getTemperature() override { return current_temp; }

  void setAccelScale(uint16_t g_range) override {
    // Mock doesn't change range, but track it
    accel_range = g_range;
  }

  void setGyroScale(uint16_t dps_range) override {
    gyro_range = dps_range;
  }

  bool calibrate() override {
    return true;
  }

  // ========================================================================
  // TEST UTILITIES
  // ========================================================================

  void reset() {
    memset(current_accel, 0, sizeof(current_accel));
    memset(current_gyro, 0, sizeof(current_gyro));
    memset(current_mag, 0, sizeof(current_mag));
    current_temp = 25.0f;
    current_quat[0] = 1.0f;  // Identity quaternion
    current_quat[1] = 0.0f;
    current_quat[2] = 0.0f;
    current_quat[3] = 0.0f;
    is_initialized = false;
    playback_mode = false;
    playback_data = nullptr;
    playback_size = 0;
    playback_index = 0;
    inject_failure = false;
    gaussian_noise_sigma = 0.0f;
    accel_range = 16;
    gyro_range = 2000;
  }

private:
  float current_accel[3] = {0, 0, 0};
  float current_gyro[3] = {0, 0, 0};
  float current_mag[3] = {0, 0, 0};
  float current_temp = 25.0f;
  float current_quat[4] = {1, 0, 0, 0};

  bool is_initialized = false;
  bool playback_mode = false;
  FlightDataPoint* playback_data = nullptr;
  size_t playback_size = 0;
  size_t playback_index = 0;
  bool inject_failure = false;
  float gaussian_noise_sigma = 0.0f;
  uint16_t accel_range = 16;
  uint16_t gyro_range = 2000;

  void normalizeQuaternion() {
    float norm = sqrt(current_quat[0]*current_quat[0] +
                      current_quat[1]*current_quat[1] +
                      current_quat[2]*current_quat[2] +
                      current_quat[3]*current_quat[3]);
    if (norm > 0) {
      for (int i = 0; i < 4; i++) {
        current_quat[i] /= norm;
      }
    }
  }
};

#endif // MOCK_SENSORS_H
