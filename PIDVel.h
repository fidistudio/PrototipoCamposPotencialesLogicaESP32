#pragma once
#include <cmath>
#include <cfloat>

#ifndef PID_LOGF
  #define PID_LOGF(fmt, ...) ((void)0)
#endif

class PIDVel {
public:
  struct Config {
    float Kp   = 0.0f;
    float Ki   = 0.0f;     // [1/s]
    float Kd   = 0.0f;     // [s]·K(s) en paralelo clásico
    float Tf   = 0.0f;     // filtro derivativo (s)  -> si usas N: Tf = Kd/N
    float Ts   = 0.10f;    // periodo de muestreo [s]
    float uMin = 0.0f;     // salida mínima (0..1)
    float uMax = 1.0f;     // salida máxima (0..1)
  };

  enum Discretization {
    PI_Tustin = 0,
    PIDF_Tustin
  };

  // === Constructores (corregido: sin argumento por defecto en .h) ===
  explicit PIDVel(const Config& cfg);
  PIDVel(); // ctor por defecto delegado

  // === Setup / configuración dinámica ===
  void setGains(float Kp, float Ki, float Kd);
  void setTf(float Tf);
  void setTs(float Ts);
  void setDiscretization(Discretization m);
  void setAntiWindup(bool on);

  // === Operación ===
  void  reset(float u0 = 0.0f);
  float update(float r, float y);

  // === Getters ===
  float u()    const { return _uSat; }
  float getKp() const { return _cfg.Kp; }
  float getKi() const { return _cfg.Ki; }
  float getKd() const { return _cfg.Kd; }
  float getTf() const { return _cfg.Tf; }
  float getTs() const { return _cfg.Ts; }

private:
  // Config activa
  Config _cfg;

  // Modo de discretización
  Discretization _mode = PI_Tustin;

  // Flags
  bool _antiWindup = false;

  // Estados de error/medición
  float _e=0, _e1=0, _e2=0;      // error(k), error(k-1), error(k-2)
  float _y=0, _y1=0;             // salida medida
  float _uPid=0;                  // salida "pura" del calculador (antes de sat.)
  float _uSat=0;                  // salida saturada

  // Derivada filtrada (PIDF_Tustin): dY[k] = (1-alpha)*dY[k-1] + alpha*(y[k]-y[k-1])
  float _dY=0, _dY1=0;
  float _alpha=0;                // Ts/(Tf+Ts)

  // Coeficientes incrementales (modo PI_Tustin)
  float _c0=0, _c1=0, _c2=0;     // con PI_Tustin: c2=0

  // Helpers
  void  _recomputeInternals();
  void  _computePI_TustinCoeffs();
  float _clamp(float v, float a, float b) const { return (v<b)?((v>a)?v:a):b; }
};
