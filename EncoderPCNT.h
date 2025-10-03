#ifndef ENCODER_PCNT_H
#define ENCODER_PCNT_H

#include <Arduino.h>
#include "driver/pcnt.h"

// Forward declaration de calibrador
class SectorCalibrator;

// ==============================
//  EncoderPCNT (KY-003 1 canal)
//  - Cuenta pulsos con PCNT
//  - Filtro HW antirruido (glitch)
//  - Ventana lógica (min gap us)
//  - Estima RPM y rad/s con EMA
//  - Aplica LUT sectorial (si hay calibrador)
//  - Índice de sector avanza según dirección (+1/-1)
// ==============================
class EncoderPCNT {
public:
  struct Config {
    gpio_num_t     pin;              // GPIO del KY-003 (open collector + pull-up)
    pcnt_unit_t    unit;             // PCNT_UNIT_0..PCNT_UNIT_7
    pcnt_channel_t channel;          // PCNT_CHANNEL_0 o _1
    int            pulsesPerRev;     // PPR efectivos
    bool           countRising   = false;   // true: rising, false: falling (KY-003 suele ser falling fiable)
    bool           invert        = false;   // invierte signo si aplicas dirección externa
    uint16_t       glitchCycles  = 0;       // filtro HW: 0..1023 (≈12.8us a 80MHz)
    uint32_t       minGapUs      = 0;       // ventana lógica adicional (us)
    float          alphaPeriod   = 0.25f;   // EMA del periodo por sector
    uint32_t       timeoutStopMs = 2000;    // declara 0 rpm si no hay pulsos (ms)
  };

  explicit EncoderPCNT(const Config& cfg);

  // HW + ISR
  void begin();

  // Llamar a 100–200 Hz (dt en segundos). Consume pulsos y actualiza rpm/omega
  void update(float dt_s);

  // Lecturas
  float rpm()   const { return _rpm; }         // RPM actuales (suavizadas)
  float omega() const { return _omega; }       // rad/s
  long  count() const { return _totalCount; }  // ticks acumulados SW
  uint32_t lastSeenMs() const { return _lastSeenMs; }

  // Utilidades
  void zero();
  void setInvert(bool inv) { _cfg.invert = inv; }

  // Calibrador (opcional)
  void attachCalibrator(SectorCalibrator* cal) { _cal = cal; }
  void setSectorIdx(uint16_t k) { _sectorIdx = (k % _ppr); }
  uint16_t sectorIdx() const { return _sectorIdx; }

  // Dirección de paso del índice de sector (+1 = k++, -1 = k--)
  void   setStepDirection(int8_t dir) { _stepDir = (dir >= 0) ? +1 : -1; }
  int8_t stepDirection() const { return _stepDir; }

  // Debug
  void printDebugEvery(uint32_t periodMs = 200);

  // Logging (opcional)
  void setLog(Stream* s) { _log = s; }

private:
  // ISR
  static void IRAM_ATTR _pcnt_isr(void* arg);
  void IRAM_ATTR _onPulseIsr(uint32_t nowUs);

  // Helpers
  void _setupPCNT();
  int16_t _readAndClearHW();  // lee delta de PCNT y limpia
  void    _applyPeriodAndCompute(uint32_t dt_us);

  // Estado
  Config   _cfg;
  int      _countsPerRev;
  uint16_t _ppr         = 1;      // PPR para sectorizado
  uint16_t _sectorIdx   = 0;      // sector actual [0.._ppr-1]
  int8_t   _stepDir     = +1;     // +1: k++; -1: k--

  // Calibrador
  SectorCalibrator* _cal = nullptr;

  // Compartido con ISR
  volatile uint32_t _isrCount     = 0;
  volatile uint32_t _isrLastUs    = 0;
  volatile uint32_t _isrPeriodUs  = 0;
  portMUX_TYPE      _mux          = portMUX_INITIALIZER_UNLOCKED;

  // SW
  long     _totalCount   = 0;
  float    _periodEmaUs  = 0.0f;
  float    _rpm          = 0.0f;
  float    _omega        = 0.0f;
  uint32_t _lastSeenMs   = 0;

  // Debug / logging
  uint32_t _dbgLastMs    = 0;
  uint32_t _dbgLastCount = 0;
  Stream*  _log          = nullptr;
};

#endif // ENCODER_PCNT_H
