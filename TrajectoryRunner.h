#ifndef TRAJECTORY_RUNNER_H
#define TRAJECTORY_RUNNER_H

#include <Arduino.h>
#include "DifferentialDrive.h"

// ============================================================
// TrajectoryRunner — Ejecuta trayectorias "giro + avance"
// sobre un DifferentialDrive con perfiles trapezoidales
// simétricos en w(t) (giro) y V(t) (avance).
//
// Flujo:
//  - planRotateAdvance(dtheta, d, wMax, vMax)  ó planFromPointInRobotFrame(x,y,...)
//  - run() / update(dt): genera (v,w) por tramos y se los pasa al drive.
//  - isFinished() indica final de la maniobra.
// ============================================================
class TrajectoryRunner {
public:
  struct Config {
    // Por defecto toma límites del drive al planear si pasas picos=0
    float vMaxDefault = 0.5f;   // [m/s]
    float wMaxDefault = 3.0f;   // [rad/s]
    // Optativo: factor de “suavidad” por si quieres bajar un poco los picos del drive
    float vPeakScale = 1.0f;    // 0<scale<=1
    float wPeakScale = 1.0f;    // 0<scale<=1
  };

  TrajectoryRunner(const Config& cfg, DifferentialDrive& drive);

  // Planificación directa (giro + avance)
  // - dtheta: rad (con signo). Giro en sitio con V=0.
  // - dist:   m   (>=0).      Avance recto con w=0.
  // - wPeak:  rad/s (si <=0, usa wMaxDefault or drive.cfg)
  // - vPeak:  m/s   (si <=0, usa vMaxDefault or drive.cfg)
  void planRotateAdvance(float dtheta, float dist, float wPeak = 0.0f, float vPeak = 0.0f);

  // Planificación desde punto en marco del robot {R}: primero orienta, luego avanza.
  void planFromPointInRobotFrame(float x_R, float y_R, float wPeak = 0.0f, float vPeak = 0.0f);

  void cancel();             // aborta y pone v=w=0
  void restart();            // reinicia la ejecución del plan actual (t=0 en tramo actual)

  // Llamar periódicamente (p.ej., 100 Hz). Internamente llama drive.setTwist() y drive.update(dt).
  void update(float dt_s);

  bool isFinished()    const { return _state == Done; }
  bool isRotating()    const { return _state == Rotating; }
  bool isAdvancing()   const { return _state == Advancing; }

  // Info útil
  float dthetaPlan()   const { return _planRot.dq; }
  float distPlan()     const { return _planLin.dq; }
  float vCmd()         const { return _v; }      // último v(t)
  float wCmd()         const { return _w; }      // último w(t)
  float tInPhase()     const { return _t; }      // tiempo en el tramo actual
  float tfPhase()      const { return _planRot.tf; } // si rotando; si avanzando usa _planLin.tf

  void setLog(Stream* s) { _log = s; }

private:
  // Perfil trapezoidal simétrico: dq = integral de qdot(t) con pico qdotMax
  // Si dq pequeño => triángulo (pico reducido). Devuelve pico real resultante y tf.
  static void computeSymmetricTrapezoid(float dq_abs, float qdotPeakReq,
                                        float& qdotPeakReal, float& t1, float& t2, float& tf);

  // Evalúa qdot(t) (velocidad) del perfil (0..tf):
  //   sube lineal 0→qdotPeak en [0,t1],
  //   const qdotPeak en [t1,t2],
  //   baja lineal qdotPeak→0 en [t2,tf].
  static float evalSymmetricTrapezoid(float t, float t1, float t2, float tf, float qdotPeak);

  void _planPhase_(float dq, float peakReq, bool isRotation);
  void _beginRotation_();
  void _beginAdvance_();
  void _advanceTime_(float dt);
  void _applyDrive_();

private:
  enum State { Idle, Rotating, Advancing, Done };

  struct PhasePlan {
    float dq        = 0.0f;  // magnitud total: |dtheta| o dist
    float peakReq   = 0.0f;  // pico solicitado (rad/s o m/s)
    float peakReal  = 0.0f;  // pico efectivo (puede bajar si dq es pequeño)
    float t1=0, t2=0, tf=0;  // tiempos del trapezoide
    bool  negSign   = false; // signo de dtheta o dist (para w/v)
  };

  Config              _cfg;
  DifferentialDrive&  _drive;

  // Plan vigente (dos fases)
  PhasePlan _planRot;   // giro
  PhasePlan _planLin;   // avance
  State     _state = Idle;

  // Integración de la ejecución
  float _t = 0.0f;      // tiempo transcurrido en la fase
  float _v = 0.0f;      // v(t) actual
  float _w = 0.0f;      // w(t) actual

  Stream* _log = nullptr;
};

#endif // TRAJECTORY_RUNNER_H
