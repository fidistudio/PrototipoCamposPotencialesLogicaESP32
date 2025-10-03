#include "MotorPWM.h"

#define MPWM_LOGF(fmt, ...) do { if (_log) _log->printf(fmt, ##__VA_ARGS__); } while(0)

MotorPWM::MotorPWM(const Config& cfg) : _cfg(cfg) {}

void MotorPWM::begin() {
  // LEDC setup
  _setupLEDC();

  // Cálculo de maxDuty según resolución
  _maxDuty = ((uint32_t)1 << _cfg.resolutionBits) - 1;

  // Estado inicial
  _uTarget  = 0.0f;
  _uApplied = 0.0f;
  _enabled  = true;

  // Asegura neutral
  _neutral_();

  MPWM_LOGF("[MotorPWM] init OK | f=%lu Hz, res=%u bits, maxDuty=%lu\n",
            (unsigned long)_cfg.freqHz, (unsigned)_cfg.resolutionBits, (unsigned long)_maxDuty);
}

void MotorPWM::_setupLEDC() {
  ledcSetup(_cfg.chanIn1, _cfg.freqHz, _cfg.resolutionBits);
  ledcSetup(_cfg.chanIn2, _cfg.freqHz, _cfg.resolutionBits);
  ledcAttachPin(_cfg.pinIn1, _cfg.chanIn1);
  ledcAttachPin(_cfg.pinIn2, _cfg.chanIn2);
}

void MotorPWM::setEnabled(bool en) {
  if (_enabled == en) return;
  _enabled = en;
  if (!_enabled) {
    // Apaga salidas al deshabilitar
    _writeIn1_(0);
    _writeIn2_(0);
    _uApplied = 0.0f;
    MPWM_LOGF("[MotorPWM] disabled\n");
  } else {
    MPWM_LOGF("[MotorPWM] enabled\n");
  }
}

void MotorPWM::setCommand(float uSigned) {
  if (_cfg.invert) uSigned = -uSigned;
  _uTarget = _clamp1_(uSigned);
}

void MotorPWM::stop() {
  _uTarget = 0.0f;
  _uApplied = 0.0f;
  _neutral_(); // salida inmediata a neutral
}

void MotorPWM::update(float dt_s) {
  if (!_enabled) return;

  // Slew-rate hacia el objetivo
  float uT = _uTarget;
  if (_cfg.slewRatePerSec > 0.0f && dt_s > 0.0f) {
    const float maxStep = _cfg.slewRatePerSec * dt_s;
    const float err = uT - _uApplied;
    if (err >  maxStep) _uApplied += maxStep;
    else if (err < -maxStep) _uApplied -= maxStep;
    else _uApplied = uT;
  } else {
    _uApplied = uT;
  }

  // Deadband + piso mínimo
  float uOut = _applyDeadbandMin_(_uApplied, _cfg.deadband, _cfg.minOutput);

  // Aplicación a salidas
  _applyOutputs_(uOut);
}

float MotorPWM::_applyDeadbandMin_(float x, float deadband, float minOut) {
  // Deadband simétrico
  if (fabsf(x) < deadband) return 0.0f;

  // Re-escalado fuera del deadband al rango [minOut..1]
  float s = (fabsf(x) - deadband) / (1.0f - deadband); // 0..1
  if (s < 0.0f) s = 0.0f; if (s > 1.0f) s = 1.0f;

  float y = minOut + (1.0f - minOut) * s; // [minOut..1]
  return (x >= 0.0f) ? y : -y;
}

void MotorPWM::_applyOutputs_(float u) {
  // Neutral (coast/brake) si u = 0
  if (u == 0.0f) {
    _neutral_();
    return;
  }

  // Magnitud en [0..1] -> duty [0..maxDuty]
  const float mag = fabsf(u);
  uint32_t duty = (uint32_t) lroundf(mag * (float)_maxDuty);
  if (duty > _maxDuty) duty = _maxDuty;

  if (_cfg.driveMode == DriveMode::SignMagnitude) {
    // Un pin PWM activo, el otro en 0 (seguro para IBT-4/BTS7960)
    if (u > 0.0f) {
      _writeIn1_(duty);
      _writeIn2_(0);
    } else {
      _writeIn1_(0);
      _writeIn2_(duty);
    }
  } else {
    // Locked Anti-Phase (neutro=50%) — usar solo si tu etapa lo soporta
    // duty1 = 0.5 + 0.5*u ; duty2 = 0.5 - 0.5*u
    // Nota: en algunas placas este modo no aplica. Úsalo bajo tu criterio.
    const float d1f = 0.5f + 0.5f * u;
    const float d2f = 0.5f - 0.5f * u;
    uint32_t d1 = (uint32_t) lroundf(d1f * (float)_maxDuty);
    uint32_t d2 = (uint32_t) lroundf(d2f * (float)_maxDuty);
    if (d1 > _maxDuty) d1 = _maxDuty; if (d2 > _maxDuty) d2 = _maxDuty;
    _writeIn1_(d1);
    _writeIn2_(d2);
  }
}

void MotorPWM::_neutral_() {
  if (_cfg.neutralMode == NeutralMode::Coast) {
    // Deja ambas entradas a 0
    _writeIn1_(0);
    _writeIn2_(0);
  } else {
    // Brake: muchas BTS7960 frenan con ambas entradas 'ALTAS'.
    // Aquí aproximamos con ambos pines a 100% en LockedAntiPhase
    // o con ambos en 100% en SignMagnitude. ¡Verifica en tu hardware!
    if (_cfg.driveMode == DriveMode::LockedAntiPhase) {
      // 50%/50% ya es neutro; para frenar marcamos extremos:
      _writeIn1_(_maxDuty);
      _writeIn2_(_maxDuty);
    } else {
      _writeIn1_(_maxDuty);
      _writeIn2_(_maxDuty);
    }
  }
}

void MotorPWM::_writeIn1_(uint32_t d) {
  _lastDutyIn1 = d;
  ledcWrite(_cfg.chanIn1, d);
}

void MotorPWM::_writeIn2_(uint32_t d) {
  _lastDutyIn2 = d;
  ledcWrite(_cfg.chanIn2, d);
}
