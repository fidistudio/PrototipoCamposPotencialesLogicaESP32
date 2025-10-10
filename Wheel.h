#ifndef WHEEL_H
#define WHEEL_H

#include <Arduino.h>
#include "MotorPWM.h"
#include "EncoderPCNT.h"
#include "SectorCalibrator.h"
#include "PIDVel.h"

// ============================================================
// Wheel — Rueda diferencial con Motor + Encoder + PID + LUT
// - Entrada: omega_ref (rad/s, con signo).
// - PID por magnitud (|u|), signo desde omega_ref.
// - Alineación/Calibración LUT con asistente (u=±assistU según sentido).
// - Ajusta enc.setStepDirection(+1/-1) según signo aplicado.
// - Compatibilidad LUT dual (FWD/REV) sin perder alineación por sentido.
// ============================================================
class Wheel {
public:
  struct Config {
    // Subconfiguraciones
    MotorPWM::Config         motor;
    EncoderPCNT::Config      encoder;
    SectorCalibrator::Config cal;
    PIDVel::Config           pid;

    // Asistente para cal/align (u sostenido)
    bool  assistOnBoot = true;  // si hay patrón/LUT, permite usar asistente en rutinas
    float assistU      = 0.50f; // |u| a sostener durante cal/align

    // Parámetros de dirección (histeresis cerca de 0)
    float    dirEpsU    = 0.05f;  // umbral de |u_applied| para fijar signo
    uint32_t dirHoldMs  = 200;    // retener signo cuando estamos cerca de 0

    // Alineación automática al boot (si hay LUT/patrón) — se intenta en el sentido actual
    bool    autoAlignOnBoot = true;
    uint8_t alignLapsBoot   = 3;
  };

  explicit Wheel(const Config& cfg);

  // Inicializa todo (NVS->LUT, enc->PCNT/ISR, motor->LEDC, etc.)
  void begin();

  // --- Control de alto nivel ---
  void  setOmegaRef(float omega_ref_signed);  // rad/s (con signo)
  float omegaRef() const { return _omegaRef; }

  // Llamar periódicamente (p.ej., 100 Hz). Aplica PID, motor y dirección de encoder.
  void  update(float dt_s);

  // --- Calibración / Alineación LUT ---
  // Mantienen la firma pública; internamente seleccionan el sentido actual (_dir).
  bool  startCalibration(uint8_t lapsN);
  bool  startAlignment(uint8_t lapsN);

  // Estado de rutinas (expuestos para coordinación externa)
  bool  isCalibrating() const { return _cal.isCalibrating(); }
  bool  isAligning()   const { return _cal.isAligning(); }

  // --- Utilidades LUT (compat dual) ---
  void  setUseLUT(bool on) {
    _cal.setUseLUTFwd(on);
    _cal.setUseLUTRev(on);
    _cal.save();
  }
  bool  useLUT()      const { return _cal.useLUTFwd() || _cal.useLUTRev(); }
  bool  patternReady()const { return _cal.patternFwdReady() || _cal.patternRevReady(); }
  void  clearLUT()          { _cal.clear(); }
  void  printLUT(Stream& s = Serial) const { _cal.printLUT(s); }
  void  printSectorStats(Stream& s = Serial) const { _cal.printSectorStats(s); }

  // --- Estado / lecturas ---
  float omega() const { return _enc.omega(); }  // rad/s (magnitud)
  float rpm()   const { return _enc.rpm(); }
  float command() const { return _motor.commandApplied(); }     // u firmado aplicado
  float commandMag() const { return fabsf(_motor.commandApplied()); }
  int8_t signApplied() const { return (_motor.commandApplied()>=0.0f)? +1 : -1; }
  uint16_t sectorIdx() const { return _enc.sectorIdx(); }

  // --- Modo neutro / PID ---
  void neutral() { _motor.setCommand(0.0f); }
  void resetPID(float u0 = 0.0f) { _pid.reset(u0); }

  // --- Logging ---
  void setLog(Stream* s);
  void printDebugEvery(uint32_t periodMs = 200);

private:
  void _applyDirectionLogic_();     // decide k++/k-- según u aplicado
  void _assistBegin_(bool isCal, int dir);   // activa asistente con el signo pedido
  void _assistTrackEnd_();          // detecta fin de cal/align y restaura u
  void _maybeAutoAlignOnBoot_();    // inicia alineación en boot si procede (en _dir)

private:
  Config _cfg;

  // Componentes
  MotorPWM          _motor;
  EncoderPCNT       _enc;
  SectorCalibrator  _cal;
  PIDVel            _pid;

  // Estado de referencia y signo
  float  _omegaRef     = 0.0f;
  int8_t _refSign      = +1;
  int8_t _lastRefSign  = +1;

  // Asistente de cal/align
  enum AssistMode { AssistNone, AssistCal, AssistAlign };
  AssistMode _assistMode = AssistNone;
  float      _assistPrevU = 0.0f;

  // Dirección (histeresis) y dirección activa de rutina
  int8_t      _dir = +1;             // sentido inferido por mando aplicado
  int8_t      _routineDir = +1;      // sentido “fijado” durante cal/align
  uint32_t    _lastStrongCmdMs = 0;

  // Logging
  Stream*   _log = nullptr;
  uint32_t  _dbgLastMs = 0;
};

#endif // WHEEL_H
