#pragma once
// ============================================================
// CAR POPPER — carpopper.h
// RKE (Remote Keyless Entry) signal replay + targeted scanning
// Uses CC1101 sub-GHz radio already on the ESP32-DIV board
//
// AUTHORIZATION GATE: VIN must be entered before any operation.
// All attempts are logged to SD card with VIN + timestamp.
//
// Supports: Toyota/Lexus, Honda/Acura, Nissan, VW/Audi, BMW
// Common frequencies: 315MHz (JP/US), 433.92MHz (EU), 868MHz (EU new)
// ============================================================

namespace CarPopper {
  void setup();
  void loop();
  void exit();
}
