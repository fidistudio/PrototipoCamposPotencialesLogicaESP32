#include "PIDVel.h"

#define PID_LOGF(fmt, ...) do { if (_log) _log->printf(fmt, ##__VA_ARGS__); } while(0)

PIDVel::PIDVel(const Config& cfg) : _cfg(cfg) {
  _recomputeCoeffs();
  reset(0.0f);
  PID_LOGF("[PID] init  Kp=%.4f Ki=%.4f Kd=%.4f Ts=%.4f  limits=[%.3f..%.3f] clamp=%d\n",
           (double)_cfg.Kp, (double)_cfg.Ki, (double)_cfg.Kd, (double)_cfg.Ts,
           (double)_cfg.uMin, (double)_cfg.uMax, _cfg.clampOutput?1:0);
}

void PIDVel::_recomputeCoeffs() {
  const float Kp = _cfg.Kp;
  const float Ki = _cfg.Ki;
  const float Kd = _cfg.Kd;
  const float Ts = (_cfg.Ts > 1e-9f) ? _cfg.Ts : 1e-3f; // evita /0

  // Coeficientes EXACTOS (5.23)
  _c0 =  Kp + (Kd / Ts);
  _c1 = -Kp + (Ki * Ts) - (2.0f * Kd / Ts);
  _c2 =  (Kd / Ts);

  PID_LOGF("[PID] coeffs  c0=%.6f c1=%.6f c2=%.6f (Kp=%.4f Ki=%.4f Kd=%.4f Ts=%.4f)\n",
           (double)_c0, (double)_c1, (double)_c2,
           (double)Kp, (double)Ki, (double)Kd, (double)Ts);
}

float PIDVel::update(float refMag, float measMag) {
  _refMag  = refMag;
  _measMag = measMag;

  // Error (magnitud)
  _e = _refMag - _measMag;

  // u[n] = u[n-1] + c0*e[n] + c1*e[n-1] + c2*e[n-2]
  float u_n = _uPrev + _c0 * _e + _c1 * _e1 + _c2 * _e2;

  // Saturación opcional (por defecto [0,1])
  if (_cfg.clampOutput) {
    u_n = _clamp(u_n, _cfg.uMin, _cfg.uMax);
  }

  // Shift de estados
  _e2    = _e1;
  _e1    = _e;
  _uPrev = u_n;
  _u     = u_n;

  return _u;
}

void PIDVel::setTunings(float Kp, float Ki, float Kd) {
  _cfg.Kp = Kp; _cfg.Ki = Ki; _cfg.Kd = Kd;
  _recomputeCoeffs();
}

void PIDVel::setTs(float Ts) {
  _cfg.Ts = Ts;
  _recomputeCoeffs();
}

void PIDVel::setOutputLimits(float uMin, float uMax, bool clamp) {
  _cfg.uMin = min(uMin, uMax);
  _cfg.uMax = max(uMin, uMax);
  _cfg.clampOutput = clamp;
  PID_LOGF("[PID] limits  [%.3f..%.3f] clamp=%d\n",
           (double)_cfg.uMin, (double)_cfg.uMax, _cfg.clampOutput?1:0);
}

void PIDVel::reset(float u0) {
  _e = 0.0f; _e1 = 0.0f; _e2 = 0.0f;
  _uPrev = u0;
  _u     = u0;
  PID_LOGF("[PID] reset  u0=%.4f\n", (double)u0);
}

void PIDVel::setInitialErrors(float e1, float e2) {
  _e1 = e1; _e2 = e2;
  PID_LOGF("[PID] setInitialErrors  e1=%.5f e2=%.5f\n", (double)e1, (double)e2);
}

void PIDVel::setInitialOutput(float u_prev) {
  _uPrev = u_prev;
  _u     = u_prev;
  PID_LOGF("[PID] setInitialOutput  uPrev=%.4f\n", (double)u_prev);
}

void PIDVel::printDebugEvery(uint32_t periodMs) {
  const uint32_t now = millis();
  if (now - _dbgLastMs < periodMs) return;
  _dbgLastMs = now;

  if (_log) {
    _log->printf("[PID] ref:%.4f meas:%.4f  e:%.4f  u:%.4f  (c0:%.5f c1:%.5f c2:%.5f)\n",
                 (double)_refMag, (double)_measMag, (double)_e, (double)_u,
                 (double)_c0, (double)_c1, (double)_c2);
  } else {
    // Fallback a Serial si no se configuró _log (por comodidad)
    Serial.printf("[PID] ref:%.4f meas:%.4f  e:%.4f  u:%.4f  (c0:%.5f c1:%.5f c2:%.5f)\n",
                  (double)_refMag, (double)_measMag, (double)_e, (double)_u,
                  (double)_c0, (double)_c1, (double)_c2);
  }
}

void PIDVel::printTunings(Stream& s) const {
  s.printf("[PID] Kp=%.6f  Ki=%.6f  Kd=%.6f  Ts=%.6f  limits=[%.3f..%.3f] clamp=%d\n",
           (double)_cfg.Kp, (double)_cfg.Ki, (double)_cfg.Kd, (double)_cfg.Ts,
           (double)_cfg.uMin, (double)_cfg.uMax, _cfg.clampOutput?1:0);
}

void PIDVel::printCoeffs(Stream& s) const {
  s.printf("[PID] c0=%.8f  c1=%.8f  c2=%.8f\n",
           (double)_c0, (double)_c1, (double)_c2);
}
