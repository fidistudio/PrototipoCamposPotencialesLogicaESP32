#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include <Arduino.h>

// ============================================================
// MotorPWM (ESP32 LEDC) - IBT-4 / BTS7960 (IN1, IN2)
// - Control por comando firmado u ∈ [-1..+1]
// - Modo Sign-Magnitude (defecto) u>0 -> IN1 PWM, IN2=0; u<0 -> IN2 PWM, IN1=0
// - Modo Locked-Anti-Phase (opcional) neutro = 50% / 50%
// - Slew-rate, deadband, duty mínimo, enable/disable, coast/brake, invert
// - Sin dependencia de pot/serial: tú decides de dónde viene u
// ============================================================
class MotorPWM {
public:
  enum class NeutralMode : uint8_t { Coast = 0, Brake = 1 };
  enum class DriveMode   : uint8_t { SignMagnitude = 0, LockedAntiPhase = 1 };

  struct Config {
    // Pines + canales LEDC
    int      pinIn1         = 32;
    int      pinIn2         = 33;
    uint8_t  chanIn1        = 0;     // 0..15
    uint8_t  chanIn2        = 1;     // 0..15 (distinto de chanIn1)

    // PWM
    uint32_t freqHz         = 20000; // 20 kHz (inaudible)
    uint8_t  resolutionBits = 8;     // 8..16 típicamente

    // Comando
    bool     invert          = false; // invierte el signo lógico de u
    float    deadband        = 0.02f; // porcentaje de neutral (|u|<db -> 0)  (0..0.2 sug.)
    float    minOutput       = 0.08f; // piso de duty para vencer fricción    (0..0.3 sug.)
    float    slewRatePerSec  = 0.0f;  // máx Δ|u| por segundo (rampa)         (0 desactiva)

    // Comportamiento para IBT-4
    NeutralMode neutralMode  = NeutralMode::Coast;           // al u=0
    DriveMode   driveMode    = DriveMode::SignMagnitude;     // modo de entrega
  };

  explicit MotorPWM(const Config& cfg);

  // Inicializa LEDC y deja el motor parado (coast o brake según neutralMode)
  void begin();

  // Llamar en loop con dt (segundos) para aplicar slew y actualizar salidas
  void update(float dt_s);

  // Comando
  void  setCommand(float uSigned); // u ∈ [-1..+1]
  float commandTarget() const { return _uTarget; }
  float commandApplied() const { return _uApplied; }

  // Estado y utilidades
  void setEnabled(bool en);
  bool enabled() const { return _enabled; }
  void stop(); // equivalente a setCommand(0) con aplicación inmediata (sin slew)
  void setInvert(bool inv) { _cfg.invert = inv; }
  void setNeutralMode(NeutralMode m) { _cfg.neutralMode = m; }
  void setDriveMode(DriveMode m) { _cfg.driveMode = m; }
  void setSlewRate(float perSec) { _cfg.slewRatePerSec = perSec; }
  void setDeadband(float db) { _cfg.deadband = constrain(db, 0.0f, 0.5f); }
  void setMinOutput(float m) { _cfg.minOutput = constrain(m, 0.0f, 0.95f); }

  // Info PWM
  uint32_t maxDuty() const { return _maxDuty; }
  uint32_t dutyIn1() const { return _lastDutyIn1; }
  uint32_t dutyIn2() const { return _lastDutyIn2; }

  // Logging opcional
  void setLog(Stream* s) { _log = s; }

private:
  // Helpers
  void _setupLEDC();
  void _applyOutputs_(float u);     // aplica mapeo -> PWM según modo
  void _writeIn1_(uint32_t d);
  void _writeIn2_(uint32_t d);
  void _neutral_();                 // aplica neutral (coast o brake)
  static float _applyDeadbandMin_(float x, float deadband, float minOut);
  static float _clamp1_(float x) { return (x < -1.0f ? -1.0f : (x > 1.0f ? 1.0f : x)); }

private:
  Config   _cfg;

  // PWM
  uint32_t _maxDuty      = 255;
  uint32_t _lastDutyIn1  = 0;
  uint32_t _lastDutyIn2  = 0;

  // Comando
  float    _uTarget      = 0.0f;
  float    _uApplied     = 0.0f;

  // Estado
  bool     _enabled      = true;

  // Logging
  Stream*  _log          = nullptr;
};

#endif // MOTOR_PWM_H
