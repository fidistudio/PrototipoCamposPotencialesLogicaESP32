#ifndef ENCODER_PCNT_H
#define ENCODER_PCNT_H

#include <Arduino.h>
#include "driver/pcnt.h"

// Forward declaration
class SectorCalibrator;

// ==============================
//  EncoderPCNT (KY-003 1 canal)
//  - Cuenta pulsos con PCNT (ESP32)
//  - Filtro HW antirruido (glitch) + ventana lógica (minGapUs)
//  - Estima RPM y rad/s con EMA del periodo
//  - Índice de sector con dirección (+1/-1) para casar LUT por sentido
//  - Integra calibración/alineación por sectores via SectorCalibrator (dual LUT)
// ==============================
class EncoderPCNT {
public:
  struct Config {
    gpio_num_t     pin;              // GPIO del KY-003 (open collector + pull-up)
    pcnt_unit_t    unit;             // PCNT_UNIT_0..PCNT_UNIT_7
    pcnt_channel_t channel;          // PCNT_CHANNEL_0 o _1
    int            pulsesPerRev;     // PPR efectivos (nº imanes)
    bool           countRising = false; // true: rising, false: falling (KY-003 suele ser falling fiable)
    bool           invert = false;      // invierte signo de lectura si usas dirección externa
    uint16_t       glitchCycles = 0;    // filtro HW: ciclos APB (80MHz). 0..1023 (0=off)
    uint32_t       minGapUs = 0;        // ventana lógica adicional (us), p.ej. 500
    float          alphaPeriod = 1.0f;  // EMA del período [0..1) 1 sin filtro, 0 retardo infinito
    uint32_t       timeoutStopMs = 2000;// declara 0 rpm si no hay pulsos (ms)
  };

  explicit EncoderPCNT(const Config& cfg);

  // Inicializa PCNT, filtros y servicio ISR
  void begin();

  // Consumir pulsos y actualizar rpm/omega (llamar a 100–200 Hz)
  void update(float dt_s);

  // Lecturas
  float rpm()   const { return _rpm; }          // RPM actuales (suavizadas)
  float omega() const { return _omega; }        // rad/s (≥0; magnitud)
  long  count() const { return _totalCount; }   // ticks SW acumulados
  uint32_t lastSeenMs() const { return _lastSeenMs; }

  // Sector actual y dirección de indexado
  void     setSectorIdx(uint16_t k) { _sectorIdx = (k % _ppr); }
  uint16_t sectorIdx() const { return _sectorIdx; }

  // +1: k++ por pulso; -1: k-- por pulso (para casar LUT del sentido real)
  void  setStepDirection(int dir) { _stepDir = (dir >= 0) ? +1 : -1; }
  int   stepDirection() const { return _stepDir; }

  // Integración con calibrador/LUT
  void attachCalibrator(SectorCalibrator* cal) { _cal = cal; }

  // Utilidades
  void zero();                    // borra contador SW y HW
  void setInvert(bool inv) { _cfg.invert = inv; }
  void printDebugEvery(uint32_t periodMs = 200);

  // Logging opcional
  void setLog(Stream* s) { _log = s; }

private:
  // ---- ISR ----
  static void IRAM_ATTR _pcnt_isr(void* arg);
  void IRAM_ATTR _onPulseIsr(uint32_t nowUs);

  // ---- Helpers ----
  void _setupPCNT();
  void _applyPeriodAndCompute(uint32_t dt_us);

private:
  Config   _cfg;
  int      _ppr;                // copia de pulsesPerRev

  // Calibrador
  SectorCalibrator* _cal = nullptr;

  // Dirección de indexado (+1/-1)
  int8_t   _stepDir = +1;

  // Estado de sectores
  uint16_t _sectorIdx = 0;

  // Snapshots escritos por ISR (protegidos con portMUX)
  volatile uint32_t _isrCount    = 0;   // # pulsos aceptados
  volatile uint32_t _isrLastUs   = 0;   // timestamp último pulso aceptado
  volatile uint32_t _isrPeriodUs = 0;   // último período válido (us)
  portMUX_TYPE      _mux         = portMUX_INITIALIZER_UNLOCKED;

  // Estado SW
  long     _totalCount   = 0;
  float    _periodEmaUs  = 0.0f;
  float    _rpm          = 0.0f;
  float    _omega        = 0.0f;  // magnitud (>=0)
  uint32_t _lastSeenMs   = 0;

  // Debug / Log
  Stream*  _log          = nullptr;
  uint32_t _dbgLastMs    = 0;
  uint32_t _dbgLastCount = 0;
};

#endif // ENCODER_PCNT_H
