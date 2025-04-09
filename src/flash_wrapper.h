#ifndef FLASH_WRAPPER_H
#define FLASH_WRAPPER_H

#include <Arduino.h>
#include <LittleFS.h>

// Simple wrapper functions for flash file operations
// This avoids the macro conflicts with SdFat

// Open a file on the flash filesystem
inline File openFlashFile(const char* path, int mode) {
  return LittleFS.open(path, mode);
}

// Remove a file from the flash filesystem
inline bool removeFlashFile(const char* path) {
  return LittleFS.remove(path);
}

// Check if a file exists on the flash filesystem
inline bool flashFileExists(const char* path) {
  return LittleFS.exists(path);
}

// Flash file read and write mode constants
#define FLASH_READ_MODE 2
#define FLASH_WRITE_MODE 1

#endif // FLASH_WRAPPER_H 