#include "Wheel.h"

#define WHEEL_LOGF(fmt, ...) do { if (_log) _log->printf(fmt, ##__VA_ARGS__); } while(0)

Wheel::Wheel(const Config& cfg)
: _cfg(cfg),
  _motor(_cfg.motor),
  _enc(_cfg.encoder),
  _cal(_cfg.cal),
  _pid(_cfg.pid)
{}

void Wheel::begin() {
  // Calibrador: cargar LUT/patrón
  _cal.load();

  // Encoder
  _enc.begin();
  _enc.attachCalibrator(&_cal);

  // Motor
  _motor.begin();

  // PID (ya inicializado por ctor). Logs opcionales:
  //_enc.setLog(_log);
  //_motor.setLog(_log);
  //_pid.setLog(_log);

  // Mensaje inicial
  WHEEL_LOGF("[Wheel] begin  PPR=%u  useLUT=%d  pattern=%d  assist=%.2f  Ts=%.4f\n",
             (unsigned)_cfg.encoder.pulsesPerRev,
             _cal.useLUT()?1:0, _cal.patternReady()?1:0,
             (double)_cfg.assistU, (double)_cfg.pid.Ts);

  // Alineación automática (si procede)
  _maybeAutoAlignOnBoot_();
}

void Wheel::setOmegaRef(float omega_ref_signed) {
  _omegaRef = omega_ref_signed;

  // Signo de la referencia (con pequeña zona muerta si quieres)
  _refSign = (_omegaRef >= 0.0f) ? +1 : -1;

  // Si cambió el signo: bumpless reset del PID (magnitud)
  if (_refSign != _lastRefSign) {
    _pid.reset(0.0f);
    _lastRefSign = _refSign;
    WHEEL_LOGF("[Wheel] ref sign change -> PID.reset()\n");
  }
}

void Wheel::update(float dt_s) {
  // 1) Encoder y Motor (estado interno)
  _enc.update(dt_s);
  _motor.update(dt_s);

  // 2) Si estamos en cal/align -> fuerza dirección positiva para indexado k++
  if (_cal.isCalibrating() || _cal.isAligning()) {
    _enc.setStepDirection(+1);
  } else {
    // 3) Lógica de dirección en operación normal
    _applyDirectionLogic_();
  }

  // 4) Control de velocidad (PID por magnitud)
  const float w_ref_mag = fabsf(_omegaRef);
  const float w_meas_mag = _enc.omega();       // magnitud ≥ 0
  const float u_mag = _pid.update(w_ref_mag, w_meas_mag); // ∈ [0,1]

  // 5) Aplica signo de la referencia
  const float u_signed = (_refSign >= 0 ? +u_mag : -u_mag);
  _motor.setCommand(u_signed);

  // 6) Asistente: detectar fin de cal/align y restaurar u si aplica
  _assistTrackEnd_();
}

bool Wheel::startCalibration(uint8_t lapsN) {
  if (lapsN == 0 || lapsN > _cfg.cal.maxLaps) return false;
  const bool ok = _cal.startCalibration(lapsN);
  if (ok) {
    WHEEL_LOGF("[Wheel] CAL start: %u laps\n", (unsigned)lapsN);
    _enc.setStepDirection(+1);      // indexado hacia adelante
    if (_cfg.assistOnBoot) _assistBegin_(/*isCal=*/true);
  }
  return ok;
}

bool Wheel::startAlignment(uint8_t lapsN) {
  if (!_cal.patternReady()) return false;
  if (lapsN == 0 || lapsN > _cfg.cal.maxLaps) return false;
  const bool ok = _cal.startAlignment(lapsN);
  if (ok) {
    WHEEL_LOGF("[Wheel] ALIGN start: %u laps\n", (unsigned)lapsN);
    _enc.setStepDirection(+1);      // indexado hacia adelante
    if (_cfg.assistOnBoot) _assistBegin_(/*isCal=*/false);
  }
  return ok;
}

// -------------------- Helpers privados --------------------

void Wheel::_applyDirectionLogic_() {
  // Deriva la dirección del signo del comando APLICADO por el motor,
  // con pequeña histéresis temporal y de amplitud.
  const float uA = _motor.commandApplied();
  const uint32_t nowMs = millis();

  if (fabsf(uA) > _cfg.dirEpsU) {
    int8_t s = (uA >= 0.0f) ? +1 : -1;
    if (s != _dir) {
      _dir = s;
      _enc.setStepDirection(_dir);   // informa sentido al encoder
      WHEEL_LOGF("[Wheel] stepDir = %d\n", (int)_dir);
    }
    _lastStrongCmdMs = nowMs;
  } else {
    // Si el mando es pequeño, conserva el último signo un rato para evitar flaps.
    if (nowMs - _lastStrongCmdMs > _cfg.dirHoldMs) {
      // aquí podrías forzar +1, pero preferimos conservarlo
    }
  }
}

void Wheel::_assistBegin_(bool isCal) {
  _assistPrevU = _motor.commandTarget();
  _assistMode = isCal ? AssistCal : AssistAlign;
  _motor.setCommand(_cfg.assistU);  // sentido +
  WHEEL_LOGF("[Wheel] ASSIST %s: hold u=%.2f\n", isCal?"CAL":"ALIGN", (double)_cfg.assistU);
}

void Wheel::_assistTrackEnd_() {
  static bool wasCal=false, wasAlign=false;
  const bool isCal   = _cal.isCalibrating();
  const bool isAlign = _cal.isAligning();

  if (_assistMode == AssistCal && wasCal && !isCal) {
    _motor.setCommand(_assistPrevU);
    _assistMode = AssistNone;
    WHEEL_LOGF("[Wheel] ASSIST: CAL done -> restore u\n");
  }
  if (_assistMode == AssistAlign && wasAlign && !isAlign) {
    _motor.setCommand(_assistPrevU);
    _assistMode = AssistNone;
    WHEEL_LOGF("[Wheel] ASSIST: ALIGN done -> restore u\n");
  }
  wasCal = isCal; wasAlign = isAlign;
}

void Wheel::_maybeAutoAlignOnBoot_() {
  if (!_cfg.autoAlignOnBoot) return;
  if (_cal.useLUT() && _cal.patternReady()) {
    const uint8_t N = _cfg.alignLapsBoot;
    if (_cal.startAlignment(N)) {
      WHEEL_LOGF("[Wheel] ALIGN auto: %u laps\n", (unsigned)N);
      _enc.setStepDirection(+1);
      if (_cfg.assistOnBoot) _assistBegin_(/*isCal=*/false);
    }
  }
}

// -------------------- Logging --------------------

void Wheel::setLog(Stream* s) {
  _log = s;
  // Propaga si quieres logs también en componentes:
  //_enc.setLog(s);
  //_motor.setLog(s);
  //_pid.setLog(s);
}

void Wheel::printDebugEvery(uint32_t periodMs) {
  const uint32_t now = millis();
  if (now - _dbgLastMs < periodMs) return;
  _dbgLastMs = now;

  const float uT = _motor.commandTarget();
  const float uA = _motor.commandApplied();
  const int   dir = (_enc.stepDirection() >= 0)? +1 : -1;

  if (_log) {
    _log->printf("[Wheel] wRef:%7.3f rad/s | w:%7.3f | uT:% .3f uA:% .3f | dir:%+d | sector:%2u | LUT:%d %s%s\n",
      (double)_omegaRef, (double)_enc.omega(),
      (double)uT, (double)uA,
      dir,
      (unsigned)_enc.sectorIdx(),
      _cal.useLUT()?1:0,
      _cal.isCalibrating() ? "[CAL] " : "",
      _cal.isAligning()    ? "[ALIGN]" : "");
  } else {
    Serial.printf("[Wheel] wRef:%7.3f rad/s | w:%7.3f | uT:% .3f uA:% .3f | dir:%+d | sector:%2u | LUT:%d %s%s\n",
      (double)_omegaRef, (double)_enc.omega(),
      (double)uT, (double)uA,
      dir,
      (unsigned)_enc.sectorIdx(),
      _cal.useLUT()?1:0,
      _cal.isCalibrating() ? "[CAL] " : "",
      _cal.isAligning()    ? "[ALIGN]" : "");
  }
}
