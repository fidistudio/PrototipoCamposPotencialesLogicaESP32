#ifndef PID_VEL_H
#define PID_VEL_H

#include <Arduino.h>

// ============================================================
// PIDVel — PID incremental (forma de velocidad)
// Implementa EXACTAMENTE:
//
//   u[n] = u[n-1]
//        + (Kp + Kd/Ts)            * e[n]
//        + (-Kp + Ki*Ts - 2*Kd/Ts) * e[n-1]
//        + (Kd/Ts)                 * e[n-2]
//
// Pensado para entregar |u| (magnitud). Por defecto satura en [0, 1].
// El signo del actuador lo maneja otra capa (p. ej. MotorPWM).
// ============================================================
class PIDVel {
public:
  struct Config {
    float Kp = 0.0f;
    float Ki = 0.0f;
    float Kd = 0.0f;
    float Ts = 0.01f;        // periodo de muestreo [s]

    // Límites de salida (magnitud):
    float uMin = 0.0f;
    float uMax = 1.0f;
    bool  clampOutput = true;
  };

  explicit PIDVel(const Config& cfg);

  // Actualiza con referencia y medición en magnitud (rad/s)
  // Devuelve u[n] ∈ [uMin, uMax]
  float update(float refMag, float measMag);

  // Azucarado: referencia con signo + medición en magnitud
  float updateMagnitude(float refSigned, float measMag) {
    return update(fabsf(refSigned), measMag);
  }

  // ---- Tunings / configuración en caliente ----
  void  setTunings(float Kp, float Ki, float Kd);
  void  setTs(float Ts);
  void  setOutputLimits(float uMin, float uMax, bool clamp = true);

  // ---- Estado ----
  void  reset(float u0 = 0.0f);                 // u[n-1]=u0, e[n-1]=e[n-2]=0
  void  setInitialErrors(float e1, float e2);   // fija e[n-1], e[n-2]
  void  setInitialOutput(float u_prev);         // fija u[n-1]

  // ---- Logging y debug ----
  void  setLog(Stream* s) { _log = s; }         // activa logs
  void  printDebugEvery(uint32_t periodMs = 200);
  void  printTunings(Stream& s = Serial) const;
  void  printCoeffs(Stream& s = Serial)  const;

  // ---- Getters ----
  float u()     const { return _u; }        // u[n]
  float uPrev() const { return _uPrev; }    // u[n-1]
  float e()     const { return _e; }        // e[n]
  float e1()    const { return _e1; }       // e[n-1]
  float e2()    const { return _e2; }       // e[n-2]
  float Ts()    const { return _cfg.Ts; }
  float ref()   const { return _refMag; }   // última ref usada (mag)
  float meas()  const { return _measMag; }  // última medición usada (mag)

private:
  void  _recomputeCoeffs();
  static inline float _clamp(float x, float a, float b) {
    return (x < a) ? a : (x > b) ? b : x;
  }

private:
  Config _cfg;

  // Estados
  float _e   = 0.0f;     // e[n]
  float _e1  = 0.0f;     // e[n-1]
  float _e2  = 0.0f;     // e[n-2]
  float _u   = 0.0f;     // u[n]
  float _uPrev = 0.0f;   // u[n-1]

  // Últimos ref/medición (para logs)
  float _refMag  = 0.0f;
  float _measMag = 0.0f;

  // Coeficientes (c0, c1, c2)
  float _c0 = 0.0f;
  float _c1 = 0.0f;
  float _c2 = 0.0f;

  // Logging
  Stream*  _log        = nullptr;
  uint32_t _dbgLastMs  = 0;
};

#endif // PID_VEL_H
