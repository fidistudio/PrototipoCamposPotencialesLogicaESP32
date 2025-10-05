#include "DifferentialDrive.h"

#define DD_LOGF(fmt, ...) do { if (_log) _log->printf(fmt, ##__VA_ARGS__); } while(0)

DifferentialDrive::DifferentialDrive(const Config& cfg, Wheel& right, Wheel& left)
: _cfg(cfg), _right(right), _left(left) {}

void DifferentialDrive::begin() {
  _right.begin();
  _left.begin();

  DD_LOGF("[DD] begin  r=%.4f L=%.4f  vMax=%.2f wMax=%.2f omgMax=%.2f\n",
          (double)_cfg.wheelRadius, (double)_cfg.trackWidth,
          (double)_cfg.vMax, (double)_cfg.wMax, (double)_cfg.omegaWheelMax);

  // Default preferido: ALINEACIÓN coordinada si hay LUT/patrón en ambas
  if (_cfg.autoCoordinatedAlignOnBoot) {
    const bool okR = _right.useLUT() && _right.patternReady();
    const bool okL = _left.useLUT()  && _left.patternReady();
    if (okR && okL) {
      startCoordinatedAlignment(_cfg.alignLapsBoot, _cfg.alignAssistW);
    } else if (okR || okL) {
      // Si solo una tiene patrón, al menos alinea esa (también sirve).
      startCoordinatedAlignment(_cfg.alignLapsBoot, _cfg.alignAssistW);
    }
  }
}

void DifferentialDrive::setTwist(float v_mps, float w_radps) {
  if (isCoordinatedRoutineRunning()) return; // ignorar fuentes externas durante coordinación
  _vRef = v_mps;
  _wRef = w_radps;
  if (_cfg.clampTwist) {
    _vRef = _clamp(_vRef, -_cfg.vMax, +_cfg.vMax);
    _wRef = _clamp(_wRef, -_cfg.wMax, +_cfg.wMax);
  }
}

void DifferentialDrive::stop() {
  setTwist(0.0f, 0.0f);
}

void DifferentialDrive::neutral() {
  _right.neutral();
  _left.neutral();
}

void DifferentialDrive::update(float dt_s) {
  if (isCoordinatedRoutineRunning()) {
    _coordUpdate_(dt_s);
    return;
  }

  // Camino normal (sin coordinación)
  _applyLimitsAndRamps_(dt_s);
  _computeWheelOmegasFromTwist_(_vCmd, _wCmd, _omegaR_cmd, _omegaL_cmd);

  if (_cfg.omegaWheelMax > 0.0f && _cfg.rescaleTwistToWheelLimit) {
    _maybeRescaleToWheelLimit_(_vCmd, _wCmd, _omegaR_cmd, _omegaL_cmd);
  }

  _right.setOmegaRef(_omegaR_cmd);
  _left.setOmegaRef (_omegaL_cmd);

  _right.update(dt_s);
  _left.update (dt_s);
}

// ----------------- Helpers “normales” -----------------

void DifferentialDrive::_applyLimitsAndRamps_(float dt) {
  if (_cfg.vAccMax > 0.0f) {
    const float dvMax = _cfg.vAccMax * dt;
    const float dv = _vRef - _vCmd;
    if (dv >  dvMax) _vCmd += dvMax;
    else if (dv < -dvMax) _vCmd -= dvMax;
    else _vCmd = _vRef;
  } else {
    _vCmd = _vRef;
  }

  if (_cfg.wAccMax > 0.0f) {
    const float dwMax = _cfg.wAccMax * dt;
    const float dw = _wRef - _wCmd;
    if (dw >  dwMax) _wCmd += dwMax;
    else if (dw < -dwMax) _wCmd -= dwMax;
    else _wCmd = _wRef;
  } else {
    _wCmd = _wRef;
  }

  if (_cfg.clampTwist) {
    _vCmd = _clamp(_vCmd, -_cfg.vMax, +_cfg.vMax);
    _wCmd = _clamp(_wCmd, -_cfg.wMax, +_cfg.wMax);
  }
}

void DifferentialDrive::_computeWheelOmegasFromTwist_(float v, float w, float& wR, float& wL) const {
  const float r = (_cfg.wheelRadius > 1e-9f) ? _cfg.wheelRadius : 1e-3f;
  const float halfL = 0.5f * _cfg.trackWidth;
  wR = (v + halfL*w) / r;
  wL = (v - halfL*w) / r;
}

void DifferentialDrive::_maybeRescaleToWheelLimit_(float& v, float& w, float& wR, float& wL) const {
  const float aR = fabsf(wR), aL = fabsf(wL);
  const float aMax = (aR > aL) ? aR : aL;
  const float omgLim = _cfg.omegaWheelMax;
  if (aMax <= omgLim || omgLim <= 0.0f) return;

  const float k = omgLim / aMax;
  v *= k; w *= k;
  _computeWheelOmegasFromTwist_(v, w, wR, wL);
  DD_LOGF("[DD] rescale v,w by %.3f to keep |omega|<=%.2f\n", (double)k, (double)omgLim);
}

// ----------------- Coordinación -----------------

bool DifferentialDrive::startCoordinatedAlignment(uint8_t lapsN, float w_assist) {
  if (isCoordinatedRoutineRunning()) return false;
  if (lapsN == 0) return false;

  if (w_assist <= 0.0f) w_assist = _cfg.alignAssistW;
  // Alinea derecha primero si tiene patrón, si no, intenta izquierda
  if (_right.patternReady()) {
    _coordEnter_(CoordAlignR, lapsN, w_assist);
    return true;
  } else if (_left.patternReady()) {
    _coordEnter_(CoordAlignL, lapsN, w_assist);
    return true;
  }
  return false;
}

bool DifferentialDrive::startCoordinatedCalibration(uint8_t lapsN, float w_assist) {
  if (isCoordinatedRoutineRunning()) return false;
  if (lapsN == 0) return false;

  if (w_assist <= 0.0f) w_assist = _cfg.calibAssistW;
  // Calibra derecha primero por consistencia
  _coordEnter_(CoordCalibR, lapsN, w_assist);
  return true;
}

void DifferentialDrive::abortCoordinatedRoutine() {
  if (!isCoordinatedRoutineRunning()) return;
  _coordExit_();
  DD_LOGF("[DD] coord ABORT\n");
}

void DifferentialDrive::_coordEnter_(CoordState st, uint8_t laps, float w_assist) {
  _coordState = st;
  _coordLaps  = laps;
  _coordW     = fabsf(w_assist); // magnitud

  // Inicia la fase correspondiente
  switch (_coordState) {
    case CoordAlignR:
      if (!_right.patternReady()) { _coordState = CoordIdle; return; }
      _right.startAlignment(_coordLaps);
      DD_LOGF("[DD] ALIGN R start (%u laps) w=+%.3f\n", (unsigned)_coordLaps, (double)_coordW);
      break;
    case CoordAlignL:
      if (!_left.patternReady())  { _coordState = CoordIdle; return; }
      _left.startAlignment(_coordLaps);
      DD_LOGF("[DD] ALIGN L start (%u laps) w=-%.3f\n", (unsigned)_coordLaps, (double)_coordW);
      break;
    case CoordCalibR:
      _right.startCalibration(_coordLaps);
      DD_LOGF("[DD] CALIB R start (%u laps) w=+%.3f\n", (unsigned)_coordLaps, (double)_coordW);
      break;
    case CoordCalibL:
      _left.startCalibration(_coordLaps);
      DD_LOGF("[DD] CALIB L start (%u laps) w=-%.3f\n", (unsigned)_coordLaps, (double)_coordW);
      break;
    default: break;
  }
}

void DifferentialDrive::_coordExit_() {
  _coordState = CoordIdle;
  _coordLaps  = 0;
  _coordW     = 0.0f;
  // detén giro
  _right.setOmegaRef(0.0f);
  _left.setOmegaRef (0.0f);
  _vCmd = _wCmd = _vRef = _wRef = 0.0f;
}

void DifferentialDrive::_coordUpdate_(float dt) {
  // 1) impone giro en sitio (v=0, w=±_coordW) según quién esté en fase
  float wSpin = 0.0f;
  switch (_coordState) {
    case CoordAlignR:
    case CoordCalibR: wSpin = +_coordW; break; // derecha debe ir positiva (k++)
    case CoordAlignL:
    case CoordCalibL: wSpin = -_coordW; break; // izquierda positiva
    default: break;
  }

  // usa rampas del drive para suavidad
  _vRef = 0.0f; _wRef = wSpin;
  _applyLimitsAndRamps_(dt);
  _computeWheelOmegasFromTwist_(_vCmd, _wCmd, _omegaR_cmd, _omegaL_cmd);

  // 2) entrega referencias a ruedas
  _right.setOmegaRef(_omegaR_cmd);
  _left.setOmegaRef (_omegaL_cmd);

  // 3) actualizar ruedas (cada una internamente alimenta cal/align con sus pulsos)
  _right.update(dt);
  _left.update (dt);

  // 4) detectar fin de la fase y pasar a la siguiente
  switch (_coordState) {
    case CoordAlignR:
      if (!_right.isAligning()) {
        // si L tiene patrón: alinear L; si no, terminar
        if (_left.patternReady()) _coordEnter_(CoordAlignL, _coordLaps, _coordW);
        else _coordExit_();
      }
      break;
    case CoordAlignL:
      if (!_left.isAligning()) {
        _coordExit_();
      }
      break;
    case CoordCalibR:
      if (!_right.isCalibrating()) {
        _coordEnter_(CoordCalibL, _coordLaps, _coordW);
      }
      break;
    case CoordCalibL:
      if (!_left.isCalibrating()) {
        _coordExit_();
      }
      break;
    default:
      _coordExit_();
      break;
  }
}

// ----------------- Logging -----------------

void DifferentialDrive::printDebugEvery(uint32_t periodMs) {
  const uint32_t now = millis();
  if (now - _dbgLastMs < periodMs) return;
  _dbgLastMs = now;

  const char* st =
    (_coordState==CoordIdle   ) ? "IDLE"  :
    (_coordState==CoordAlignR ) ? "A_R"   :
    (_coordState==CoordAlignL ) ? "A_L"   :
    (_coordState==CoordCalibR ) ? "C_R"   :
    (_coordState==CoordCalibL ) ? "C_L"   : "?";

  if (_log) {
    _log->printf("[DD] state:%s  vRef:% .3f wRef:% .3f | vCmd:% .3f wCmd:% .3f | wR:% .3f wL:% .3f\n",
                 st, (double)_vRef, (double)_wRef, (double)_vCmd, (double)_wCmd,
                 (double)_omegaR_cmd, (double)_omegaL_cmd);
  } else {
    Serial.printf("[DD] state:%s  vRef:% .3f wRef:% .3f | vCmd:% .3f wCmd:% .3f | wR:% .3f wL:% .3f\n",
                  st, (double)_vRef, (double)_wRef, (double)_vCmd, (double)_wCmd,
                  (double)_omegaR_cmd, (double)_omegaL_cmd);
  }
}
