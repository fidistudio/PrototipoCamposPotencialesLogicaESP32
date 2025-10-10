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

  // Mensaje inicial
  WHEEL_LOGF("[Wheel] begin  PPR=%u  useFWD=%d useREV=%d  pattFWD=%d pattREV=%d  assist=%.2f  Ts=%.4f\n",
             (unsigned)_cfg.encoder.pulsesPerRev,
             _cal.useLUTFwd()?1:0, _cal.useLUTRev()?1:0,
             _cal.patternFwdReady()?1:0, _cal.patternRevReady()?1:0,
             (double)_cfg.assistU, (double)_cfg.pid.Ts);

  // Alineación automática (si procede) en el sentido actual (_dir)
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

  // 2) Si estamos en cal/align -> mantener el sentido fijado en el inicio de la rutina
  if (_cal.isCalibrating() || _cal.isAligning()) {
    _enc.setStepDirection(_routineDir);
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

  // Tomamos el sentido “operativo” actual inferido por la lógica de dirección
  int dir = _dir;               // +1 FWD, -1 REV
  _routineDir = dir;

  const bool ok = _cal.startCalibrationDir(lapsN, dir);
  if (ok) {
    WHEEL_LOGF("[Wheel] CAL start: %u laps (%s)\n",
               (unsigned)lapsN, (dir>=0)?"FWD":"REV");
    _enc.setStepDirection(dir);      // indexado en el sentido deseado
    if (_cfg.assistOnBoot) _assistBegin_(/*isCal=*/true, dir);
  }
  return ok;
}

bool Wheel::startAlignment(uint8_t lapsN) {
  if (lapsN == 0 || lapsN > _cfg.cal.maxLaps) return false;

  int dir = _dir;               // +1 FWD, -1 REV
  const bool pattReady = (dir >= 0) ? _cal.patternFwdReady()
                                    : _cal.patternRevReady();
  if (!pattReady) return false;

  _routineDir = dir;
  const bool ok = _cal.startAlignmentDir(lapsN, dir);
  if (ok) {
    WHEEL_LOGF("[Wheel] ALIGN start: %u laps (%s)\n",
               (unsigned)lapsN, (dir>=0)?"FWD":"REV");
    _enc.setStepDirection(dir);      // indexado en el sentido deseado
    if (_cfg.assistOnBoot) _assistBegin_(/*isCal=*/false, dir);
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
      // opcional: podrías forzar +1, preferimos conservarlo
    }
  }
}

void Wheel::_assistBegin_(bool isCal, int dir) {
  _assistPrevU = _motor.commandTarget();
  _assistMode = isCal ? AssistCal : AssistAlign;

  const float uSigned = (dir >= 0) ? +_cfg.assistU : -_cfg.assistU;
  _motor.setCommand(uSigned);   // sostener en el sentido seleccionado
  _enc.setStepDirection(dir);

  WHEEL_LOGF("[Wheel] ASSIST %s: hold u=% .2f (%s)\n",
             isCal?"CAL":"ALIGN", (double)uSigned, (dir>=0)?"FWD":"REV");
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

  // Intentar auto-align en el sentido operativo actual (_dir)
  const int dir = _dir; // por defecto +1
  const bool use = (dir >= 0) ? _cal.useLUTFwd() : _cal.useLUTRev();
  const bool patt = (dir >= 0) ? _cal.patternFwdReady() : _cal.patternRevReady();

  if (use && patt) {
    const uint8_t N = _cfg.alignLapsBoot;
    if (_cal.startAlignmentDir(N, dir)) {
      WHEEL_LOGF("[Wheel] ALIGN auto: %u laps (%s)\n", (unsigned)N, (dir>=0)?"FWD":"REV");
      _routineDir = dir;
      _enc.setStepDirection(dir);
      if (_cfg.assistOnBoot) _assistBegin_(/*isCal=*/false, dir);
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
    _log->printf("[Wheel] wRef:%7.3f rad/s | w:%7.3f | uT:% .3f uA:% .3f | dir:%+d | sector:%2u | use(F,R)=(%d,%d) %s%s\n",
      (double)_omegaRef, (double)_enc.omega(),
      (double)uT, (double)uA,
      dir,
      (unsigned)_enc.sectorIdx(),
      _cal.useLUTFwd()?1:0, _cal.useLUTRev()?1:0,
      _cal.isCalibrating() ? "[CAL] " : "",
      _cal.isAligning()    ? "[ALIGN]" : "");
  } else {
    Serial.printf("[Wheel] wRef:%7.3f rad/s | w:%7.3f | uT:% .3f uA:% .3f | dir:%+d | sector:%2u | use(F,R)=(%d,%d) %s%s\n",
      (double)_omegaRef, (double)_enc.omega(),
      (double)uT, (double)uA,
      dir,
      (unsigned)_enc.sectorIdx(),
      _cal.useLUTFwd()?1:0, _cal.useLUTRev()?1:0,
      _cal.isCalibrating() ? "[CAL] " : "",
      _cal.isAligning()    ? "[ALIGN]" : "");
  }
}
