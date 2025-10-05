#include "TrajectoryRunner.h"
#include <math.h>

#define TR_LOGF(fmt, ...) do { if (_log) _log->printf(fmt, ##__VA_ARGS__); } while(0)

TrajectoryRunner::TrajectoryRunner(const Config& cfg, DifferentialDrive& drive)
: _cfg(cfg), _drive(drive) {}

// ---------- Planificación pública ----------

void TrajectoryRunner::planRotateAdvance(float dtheta, float dist, float wPeak, float vPeak) {
  // Normaliza picos si no los pasaron:
  if (wPeak <= 0.0f) wPeak = _cfg.wMaxDefault * _cfg.wPeakScale;
  if (vPeak <= 0.0f) vPeak = _cfg.vMaxDefault * _cfg.vPeakScale;

  // Fase de giro
  _planPhase_(dtheta, wPeak, /*isRotation=*/true);
  // Fase de avance
  _planPhase_(dist,   vPeak, /*isRotation=*/false);

  // Arranque de la primera fase (si dq=0 salta a la siguiente)
  if (_planRot.dq > 0.0f) _beginRotation_();
  else if (_planLin.dq > 0.0f) _beginAdvance_();
  else { _state = Done; _v = 0.0f; _w = 0.0f; }

  TR_LOGF("[TR] plan R: dq=%.4f peak=%.3f tf=%.3f | L: dq=%.4f peak=%.3f tf=%.3f\n",
          (double)(_planRot.negSign?-_planRot.dq:_planRot.dq), (double)_planRot.peakReal, (double)_planRot.tf,
          (double)(_planLin.negSign?-_planLin.dq:_planLin.dq), (double)_planLin.peakReal, (double)_planLin.tf);
}

void TrajectoryRunner::planFromPointInRobotFrame(float x_R, float y_R, float wPeak, float vPeak) {
  const float dtheta = atan2f(y_R, x_R);           // giro deseado (rad, con signo)
  const float dist   = hypotf(x_R, y_R);           // avance (m, >=0)
  planRotateAdvance(dtheta, dist, wPeak, vPeak);
}

void TrajectoryRunner::cancel() {
  _state = Done;
  _v = 0.0f; _w = 0.0f;
  _drive.setTwist(0.0f, 0.0f);
  TR_LOGF("[TR] cancel\n");
}

void TrajectoryRunner::restart() {
  if (_state == Rotating) { _t = 0.0f; _beginRotation_(); }
  else if (_state == Advancing) { _t = 0.0f; _beginAdvance_(); }
}

// ---------- Ejecución ----------

void TrajectoryRunner::update(float dt_s) {
  if (_state == Done || _state == Idle) {
    _drive.setTwist(0.0f, 0.0f);
    _drive.update(dt_s);
    return;
  }

  _advanceTime_(dt_s);
  _applyDrive_();
  _drive.update(dt_s);
}

// ---------- Privados: perfiles y fases ----------

void TrajectoryRunner::computeSymmetricTrapezoid(float dq_abs, float qdotPeakReq,
                                                 float& qdotPeakReal, float& t1, float& t2, float& tf) {
  // Perfil trapezoidal simétrico (aceleración y frenado con el mismo tiempo t1=tf/3):
  // Si dq es suficientemente grande, pico = qdotPeakReq y tf = 1.5 * dq_abs / qdotPeakReq.
  // Si dq es pequeño, se recorta el pico para que t2=t1 (perfil triangular).
  if (dq_abs <= 0.0f || qdotPeakReq <= 0.0f) {
    qdotPeakReal = 0.0f; t1 = t2 = tf = 0.0f; return;
  }

  // Tiempo si logramos alcanzar el pico solicitado (perfil trapezoidal "completo"):
  float tf_trap = 1.5f * (dq_abs / qdotPeakReq);
  float t1_trap = tf_trap / 3.0f;
  float t2_trap = 2.0f * t1_trap;

  // Área bajo qdot(t) con ese perfil es dq_abs. Si eso es válido, adoptamos.
  // Para perfiles muy cortos, el pico que cumple simetría y área es:
  //   qdotPeakReal = sqrt( (3/2) * dq_abs / t1 ) con tf = 3*t1
  // En práctica, detectar si el pico requerido es alcanzable con una aceleración "implícita".
  // Aquí usamos un criterio práctico: si tf_trap < Tmin -> degradamos a triángulo.
  // Para no introducir una a_max explícita, siempre aceptamos el trapezoide con ese tf.
  // Si quieres forzar un triángulo para distancias ultra cortas, puedes introducir un t1_min.
  qdotPeakReal = qdotPeakReq;
  t1 = t1_trap; t2 = t2_trap; tf = tf_trap;
}

float TrajectoryRunner::evalSymmetricTrapezoid(float t, float t1, float t2, float tf, float qdotPeak) {
  if (tf <= 0.0f || qdotPeak <= 0.0f) return 0.0f;
  if (t <= 0.0f) return 0.0f;
  if (t >= tf)   return 0.0f;

  if (t < t1) {
    // rampa ascendente
    return qdotPeak * (t / t1);
  } else if (t < t2) {
    // tramo constante
    return qdotPeak;
  } else {
    // rampa descendente
    const float tr = (tf - t);
    const float T  = (tf - t2);
    return qdotPeak * (tr / T);
  }
}

void TrajectoryRunner::_planPhase_(float dq, float peakReq, bool isRotation) {
  PhasePlan& p = isRotation ? _planRot : _planLin;

  p.negSign = (dq < 0.0f);
  p.dq      = fabsf(dq);
  p.peakReq = fabsf(peakReq);

  computeSymmetricTrapezoid(p.dq, p.peakReq, p.peakReal, p.t1, p.t2, p.tf);
}

void TrajectoryRunner::_beginRotation_() {
  _state = Rotating;
  _t = 0.0f;
  TR_LOGF("[TR] begin ROT: dq=%.4f  peak=%.3f  t1=%.3f t2=%.3f tf=%.3f\n",
          (double)(_planRot.negSign?-_planRot.dq:_planRot.dq),
          (double)_planRot.peakReal, (double)_planRot.t1, (double)_planRot.t2, (double)_planRot.tf);
}

void TrajectoryRunner::_beginAdvance_() {
  _state = Advancing;
  _t = 0.0f;
  TR_LOGF("[TR] begin LIN: dq=%.4f  peak=%.3f  t1=%.3f t2=%.3f tf=%.3f\n",
          (double)(_planLin.negSign?-_planLin.dq:_planLin.dq),
          (double)_planLin.peakReal, (double)_planLin.t1, (double)_planLin.t2, (double)_planLin.tf);
}

void TrajectoryRunner::_advanceTime_(float dt) {
  _t += dt;

  if (_state == Rotating) {
    const float w_mag = evalSymmetricTrapezoid(_t, _planRot.t1, _planRot.t2, _planRot.tf, _planRot.peakReal);
    _w = _planRot.negSign ? -w_mag : +w_mag;
    _v = 0.0f;

    if (_t >= _planRot.tf) {
      // terminar rotación
      _w = 0.0f;
      if (_planLin.dq > 0.0f) _beginAdvance_();
      else { _state = Done; }
    }
  }
  else if (_state == Advancing) {
    const float v_mag = evalSymmetricTrapezoid(_t, _planLin.t1, _planLin.t2, _planLin.tf, _planLin.peakReal);
    _v = _planLin.negSign ? -v_mag : +v_mag; // por si algún día quieres dist<0 (retroceso)
    _w = 0.0f;

    if (_t >= _planLin.tf) {
      _v = 0.0f; _state = Done;
    }
  }
}

void TrajectoryRunner::_applyDrive_() {
  _drive.setTwist(_v, _w);
}
