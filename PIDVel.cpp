#include "PIDVel.h"

// ======================== Internos =========================
void PIDVel::_computePI_TustinCoeffs() {
  const float Kp = _cfg.Kp;
  const float Ki = _cfg.Ki;
  const float Ts = (_cfg.Ts > 1e-9f) ? _cfg.Ts : 1e-3f;

  // Incremental PI (Tustin):
  // u[n] = u[n-1] + Kp*(e[n]-e[n-1]) + (Ki*Ts/2)*(e[n]+e[n-1])
  _c0 =  Kp + (Ki*Ts*0.5f);
  _c1 = -Kp + (Ki*Ts*0.5f);
  _c2 =  0.0f;

  PID_LOGF("[PID] PI_Tustin coeffs: c0=%.6f c1=%.6f\n",
           (double)_c0, (double)_c1);
}

void PIDVel::_recomputeInternals() {
  // Derivative filter alpha
  if (_cfg.Tf > 0.0f) {
    const float Ts = (_cfg.Ts > 1e-9f) ? _cfg.Ts : 1e-3f;
    _alpha = Ts / (_cfg.Tf + Ts);
    if (_alpha < 0.0f) _alpha = 0.0f;
    if (_alpha > 1.0f) _alpha = 1.0f;
  } else {
    _alpha = 1.0f; // si Tf=0, la derivada se vuelve diferencia simple
  }

  if (_mode == PI_Tustin) {
    _computePI_TustinCoeffs();
  }

  PID_LOGF("[PID] Recompute: Kp=%.6f Ki=%.6f Kd=%.6f Tf=%.6f Ts=%.6f alpha=%.6f mode=%d\n",
           (double)_cfg.Kp, (double)_cfg.Ki, (double)_cfg.Kd,
           (double)_cfg.Tf, (double)_cfg.Ts, (double)_alpha, (int)_mode);
}

// ======================== Constructores =========================
// (Corregido) Sin argumento por defecto en la declaración del .h
PIDVel::PIDVel(const Config& cfg) : _cfg(cfg) {
  _recomputeInternals();
}

// Ctor por defecto delegado
PIDVel::PIDVel() : PIDVel(Config{}) {}

// ======================== Setters =========================
void PIDVel::setGains(float Kp, float Ki, float Kd) {
  _cfg.Kp = Kp;
  _cfg.Ki = Ki;
  _cfg.Kd = Kd;
  _recomputeInternals();
}

void PIDVel::setTf(float Tf) {
  _cfg.Tf = (Tf >= 0.0f) ? Tf : 0.0f;
  _recomputeInternals();
}

void PIDVel::setTs(float Ts) {
  _cfg.Ts = (Ts > 1e-9f) ? Ts : 1e-3f;
  _recomputeInternals();
}

void PIDVel::setDiscretization(Discretization m) {
  _mode = m;
  _recomputeInternals();
}

void PIDVel::setAntiWindup(bool on) {
  _antiWindup = on;
}

// ======================== Operación =========================
void PIDVel::reset(float u0) {
  _e = _e1 = _e2 = 0.0f;
  _y = _y1 = 0.0f;
  _dY = _dY1 = 0.0f;
  _uPid = _uSat = _clamp(u0, _cfg.uMin, _cfg.uMax);
}

// r, y son magnitudes (no negativas); el signo lo maneja el caller (tu .ino)
float PIDVel::update(float r, float y) {
  // Estado actual
  _y = y;
  _e = r - y;

  if (_mode == PI_Tustin) {
    // Incremental PI Tustin
    float u_new = _uPid + _c0*_e + _c1*_e1; // _c2=0

    // Anti-windup (clamping simple en cálculo incremental)
    if (_antiWindup) {
      // si saturaría y el error empuja en la misma dirección, "congela" en el borde
      if (u_new > _cfg.uMax && _e > 0.0f) u_new = _cfg.uMax;
      if (u_new < _cfg.uMin && _e < 0.0f) u_new = _cfg.uMin;
    }

    // Actualiza estado interno del calculador y salida saturada
    _uPid = u_new;
    _uSat = _clamp(u_new, _cfg.uMin, _cfg.uMax);

  } else { // PIDF_Tustin (paralelo con derivada de la medida filtrada)
    // Derivada filtrada de la medida
    const float dy = _y - _y1;
    const float dY = (1.0f - _alpha)*_dY1 + _alpha*dy;

    // Integral por trapecios y proporcional/derivativo en paralelo
    const float Ts = (_cfg.Ts > 1e-9f) ? _cfg.Ts : 1e-3f;
    static float I = 0.0f;

    // Integración con anti-windup básico (stop-integrator al saturar y empujar)
    float P = _cfg.Kp * _e;
    float D = (_cfg.Tf > 0.0f || _alpha < 1.0f) ? (-_cfg.Kd * dY / Ts) : 0.0f; // derivada sobre la medida

    float u_pre = P + I + D;

    // Probar saturación hipotética después de integrar
    float I_candidate = I + _cfg.Ki * (Ts*0.5f) * (_e + _e1);
    float u_candidate = P + I_candidate + D;

    if (_antiWindup) {
      // Evita integrar si empuja hacia fuera estando en el borde
      bool satur_high = (u_pre >= _cfg.uMax - 1e-6f) && (_e > 0.0f);
      bool satur_low  = (u_pre <= _cfg.uMin + 1e-6f) && (_e < 0.0f);
      if (!(satur_high || satur_low)) {
        I = I_candidate;
        u_pre = u_candidate;
      }
    } else {
      I = I_candidate;
      u_pre = u_candidate;
    }

    _uPid = u_pre;
    _uSat = _clamp(_uPid, _cfg.uMin, _cfg.uMax);

    // Shifts derivada
    _dY1 = dY;
  }

  // Shifts comunes
  _e2 = _e1;
  _e1 = _e;
  _y1 = _y;

  return _uSat;
}
