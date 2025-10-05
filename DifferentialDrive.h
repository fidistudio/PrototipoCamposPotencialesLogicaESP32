#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

#include <Arduino.h>
#include "Wheel.h"

// ============================================================
// DifferentialDrive — Orquestador de 2 ruedas (R/L)
// + Coordinated Alignment/Calibration (spin-in-place)
// ============================================================
class DifferentialDrive {
public:
  struct Config {
    // --- Geometría ---
    float wheelRadius = 0.05f;   // r  [m]
    float trackWidth  = 0.20f;   // L  [m]

    // --- Límites de twist (robot) ---
    float vMax  = 0.8f;          // [m/s]
    float wMax  = 6.0f;          // [rad/s]

    // --- Rampas ---
    float vAccMax = 1.5f;        // [m/s^2]  (0 => sin rampa)
    float wAccMax = 10.0f;       // [rad/s^2]
    bool  clampTwist = true;

    // --- Límite de omega de rueda ---
    float omegaWheelMax = 120.0f;     // [rad/s]  (<=0 => desactivado)
    bool  rescaleTwistToWheelLimit = true;

    // --- Coordinated ALIGN on boot (preferido) ---
    bool    autoCoordinatedAlignOnBoot = true;
    uint8_t alignLapsBoot              = 3;     // N vueltas
    float   alignAssistW               = 2.0f;  // [rad/s] giro en sitio durante alineación

    // --- (Opcional) Coordinated CALIB si la pides explícitamente ---
    float   calibAssistW               = 2.0f;  // [rad/s]
  };

  DifferentialDrive(const Config& cfg, Wheel& right, Wheel& left);

  // Inicializa (propaga begin() a ruedas) y lanza alineación coordinada si procede
  void begin();

  // --- Comandos de alto nivel (cuando NO hay coordinación en curso) ---
  void setTwist(float v_mps, float w_radps);
  void stop();
  void neutral();

  // Actualización periódica (100 Hz típico). Llama update() de ambas ruedas.
  void update(float dt_s);

  // --- Lecturas / estado ---
  float vRef() const { return _vRef; }
  float wRef() const { return _wRef; }
  float vCmd() const { return _vCmd; }
  float wCmd() const { return _wCmd; }
  float omegaR() const { return _omegaR_cmd; }
  float omegaL() const { return _omegaL_cmd; }

  // --- Rutinas coordinadas ---
  bool startCoordinatedAlignment(uint8_t lapsN, float w_assist_radps = 0.0f);
  bool startCoordinatedCalibration(uint8_t lapsN, float w_assist_radps = 0.0f);
  void abortCoordinatedRoutine();
  bool isCoordinatedRoutineRunning() const { return _coordState != CoordIdle; }

  // (Proxies convenientes por si quieres las individuales)
  bool startCalibrationR(uint8_t N) { return _right.startCalibration(N); }
  bool startCalibrationL(uint8_t N) { return _left.startCalibration(N); }
  bool startAlignmentR (uint8_t N)  { return _right.startAlignment(N); }
  bool startAlignmentL (uint8_t N)  { return _left.startAlignment(N); }

  // Logging
  void setLog(Stream* s) { _log = s; }
  void printDebugEvery(uint32_t periodMs = 200);

  // Acceso directo
  Wheel& wheelR() { return _right; }
  Wheel& wheelL() { return _left; }

private:
  // ---------- helpers “normales” del drive ----------
  void  _applyLimitsAndRamps_(float dt);
  void  _computeWheelOmegasFromTwist_(float v, float w, float& wR, float& wL) const;
  void  _maybeRescaleToWheelLimit_(float& v, float& w, float& wR, float& wL) const;
  static inline float _clamp(float x, float a, float b) {
    return (x < a) ? a : (x > b) ? b : x;
  }

  // ---------- coordinación ----------
  enum CoordState { CoordIdle, CoordAlignR, CoordAlignL, CoordCalibR, CoordCalibL };
  void _coordUpdate_(float dt);
  void _coordEnter_(CoordState st, uint8_t laps, float w_assist);
  void _coordExit_();

private:
  Config _cfg;
  Wheel& _right;
  Wheel& _left;

  // Referencias “externas” (cuando no hay coordinación)
  float _vRef = 0.0f, _wRef = 0.0f;

  // Comandos tras rampas / escalado
  float _vCmd = 0.0f, _wCmd = 0.0f;

  // Omegas objetivo para ruedas
  float _omegaR_cmd = 0.0f, _omegaL_cmd = 0.0f;

  // Coordinación
  CoordState _coordState = CoordIdle;
  uint8_t    _coordLaps  = 0;
  float      _coordW     = 0.0f;   // [rad/s] giro en sitio durante rutina

  // Logging
  Stream*   _log = nullptr;
  uint32_t  _dbgLastMs = 0;
};

#endif // DIFFERENTIAL_DRIVE_H
