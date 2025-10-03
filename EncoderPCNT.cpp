#include "EncoderPCNT.h"
#include "SectorCalibrator.h"

#define ENC_LOGF(fmt, ...) do { if (_log) _log->printf(fmt, ##__VA_ARGS__); } while(0)

// =======================
//  Público
// =======================
EncoderPCNT::EncoderPCNT(const Config& cfg)
: _cfg(cfg),
  _countsPerRev(max(1, cfg.pulsesPerRev)),
  _ppr(static_cast<uint16_t>(max(1, cfg.pulsesPerRev)))
{}

void EncoderPCNT::begin() {
  pinMode(_cfg.pin, INPUT_PULLUP);
  _setupPCNT();                // PCNT + filtro HW + evento por pulso
  _lastSeenMs  = millis();
  _periodEmaUs = 0.0f;
  _rpm = _omega = 0.0f;
  _sectorIdx = 0;
  _stepDir   = +1;
}

void EncoderPCNT::update(float /*dt_s*/) {
  // Snapshot atómico de variables compartidas con ISR
  uint32_t cntSnap, perSnap, lastUsSnap;
  portENTER_CRITICAL(&_mux);
  cntSnap     = _isrCount;
  perSnap     = _isrPeriodUs;
  lastUsSnap  = _isrLastUs;
  portEXIT_CRITICAL(&_mux);

  static uint32_t lastConsumed = 0;
  if (cntSnap == lastConsumed) {
    // Timeout: si pasó demasiado tiempo sin pulsos, baja a cero
    if (millis() - _lastSeenMs > _cfg.timeoutStopMs) {
      _rpm = 0.0f; _omega = 0.0f; _periodEmaUs = 0.0f;
    }
    return;
  }

  // Consume nuevos pulsos
  uint32_t delta = cntSnap - lastConsumed;
  lastConsumed = cntSnap;

  // Si delta > 1 no tenemos cola de periodos; usamos el último válido (simple).
  for (uint32_t i = 0; i < delta; ++i) {
    if (perSnap == 0) continue; // ignora primer pulso tras arranque
    _applyPeriodAndCompute(perSnap);
  }
}

void EncoderPCNT::zero() {
  portENTER_CRITICAL(&_mux);
  _isrCount = 0;
  _isrPeriodUs = 0;
  _isrLastUs = 0;
  portEXIT_CRITICAL(&_mux);

  _totalCount = 0;
  _periodEmaUs = 0.0f;
  _rpm = _omega = 0.0f;
  _sectorIdx = 0;
  _stepDir   = +1;

  // Limpia HW
  pcnt_counter_pause(_cfg.unit);
  pcnt_counter_clear(_cfg.unit);
  pcnt_counter_resume(_cfg.unit);
}

void EncoderPCNT::printDebugEvery(uint32_t periodMs) {
  const uint32_t now = millis();
  if (now - _dbgLastMs < periodMs) return;

  // snapshot atómico del contador de ISR
  uint32_t cntSnap;
  portENTER_CRITICAL(&_mux);
  cntSnap = _isrCount;
  portEXIT_CRITICAL(&_mux);

  const uint32_t d = cntSnap - _dbgLastCount;
  _dbgLastCount = cntSnap;
  _dbgLastMs = now;

  Serial.printf(
    "PCNT cnt:%6lu | pps*:%4lu | RPM:%7.3f | Omega:%7.3f rad/s | perEMA:%9.1f us | sector:%2u | dir:%+d\n",
    (unsigned long)cntSnap, (unsigned long)d, _rpm, _omega, _periodEmaUs,
    (unsigned)_sectorIdx, (int)_stepDir
  );
}

// =======================
//  Privado (ISR y helpers)
// =======================
void IRAM_ATTR EncoderPCNT::_pcnt_isr(void* arg) {
  auto* self = static_cast<EncoderPCNT*>(arg);
  const uint32_t now = micros();
  self->_onPulseIsr(now);
  // Rearmar umbral para que cada 1 vuelva a disparar
  pcnt_counter_clear(self->_cfg.unit);
}

void IRAM_ATTR EncoderPCNT::_onPulseIsr(uint32_t nowUs) {
  // Ventana lógica (MIN GAP)
  if (_cfg.minGapUs > 0) {
    const uint32_t gap = nowUs - _isrLastUs;
    if (_isrLastUs != 0 && gap < _cfg.minGapUs) {
      return; // rebote/ruido
    }
  }

  const uint32_t period = (_isrLastUs == 0) ? 0 : (nowUs - _isrLastUs);

  portENTER_CRITICAL_ISR(&_mux);
  _isrLastUs = nowUs;
  if (period) _isrPeriodUs = period;  // guarda último periodo válido
  _isrCount++;
  portEXIT_CRITICAL_ISR(&_mux);
}

void EncoderPCNT::_setupPCNT() {
  pcnt_config_t c = {};
  c.pulse_gpio_num = _cfg.pin;
  c.ctrl_gpio_num  = PCNT_PIN_NOT_USED;
  // Elegimos flanco: rising o falling
  if (_cfg.countRising) {
    c.pos_mode = PCNT_COUNT_INC;
    c.neg_mode = PCNT_COUNT_DIS;
  } else {
    c.pos_mode = PCNT_COUNT_DIS;
    c.neg_mode = PCNT_COUNT_INC; // falling
  }
  c.lctrl_mode = PCNT_MODE_KEEP;
  c.hctrl_mode = PCNT_MODE_KEEP;
  c.counter_h_lim = 32767;
  c.counter_l_lim = 0;
  c.unit    = _cfg.unit;
  c.channel = _cfg.channel;
  ESP_ERROR_CHECK(pcnt_unit_config(&c));

  // Filtro HW (glitch)
  if (_cfg.glitchCycles > 0) {
    ESP_ERROR_CHECK(pcnt_set_filter_value(_cfg.unit, _cfg.glitchCycles));
    ESP_ERROR_CHECK(pcnt_filter_enable(_cfg.unit));
  } else {
    pcnt_filter_disable(_cfg.unit);
  }

  // Evento por pulso: THRES_0=1
  ESP_ERROR_CHECK(pcnt_set_event_value(_cfg.unit, PCNT_EVT_THRES_0, 1));
  ESP_ERROR_CHECK(pcnt_event_enable(_cfg.unit, PCNT_EVT_THRES_0));

  // Preparar y arrancar
  ESP_ERROR_CHECK(pcnt_counter_pause(_cfg.unit));
  ESP_ERROR_CHECK(pcnt_counter_clear(_cfg.unit));
  ESP_ERROR_CHECK(pcnt_counter_resume(_cfg.unit));

  // Instalar ISR y registrar handler con "this" como arg
  static bool isrInstalled = false;
  if (!isrInstalled) {
    ESP_ERROR_CHECK(pcnt_isr_service_install(0)); // ISR en IRAM
    isrInstalled = true;
  }
  ESP_ERROR_CHECK(pcnt_isr_handler_add(_cfg.unit, &EncoderPCNT::_pcnt_isr, this));
}

int16_t EncoderPCNT::_readAndClearHW() {
  int16_t val = 0;
  pcnt_get_counter_value(_cfg.unit, &val);
  pcnt_counter_clear(_cfg.unit);
  return val;
}

void EncoderPCNT::_applyPeriodAndCompute(uint32_t dt_us) {
  float dt = (float)dt_us;

  // 1) Integración con calibrador (calibración/alineación + corrección LUT)
  if (_cal) {
    if (_cal->isCalibrating() || _cal->isAligning()) {
      _cal->feedPeriod(_sectorIdx, dt);
      if (_cal->isCalibrating()) _cal->finishCalibrationIfReady();
      if (_cal->isAligning()) {
        uint16_t off; float score;
        if (_cal->finishAlignmentIfReady(off, score)) {
          ENC_LOGF("[ALIGN] Offset aplicado=%u  score=%.4f\n", (unsigned)off, (double)score);
          _sectorIdx = off;      // aplica offset inicial
          _periodEmaUs = 0.0f;   // bumpless
          _rpm = _omega = 0.0f;
        }
      }
    }
    // Corrección LUT en operación normal
    dt = _cal->correctDt(_sectorIdx, dt);
  }

  // 2) EMA del periodo
  if (_periodEmaUs <= 0.0f) _periodEmaUs = dt;
  else {
    const float a = _cfg.alphaPeriod;
    _periodEmaUs = (1.0f - a) * _periodEmaUs + a * dt;
  }

  // 3) Convierte a rpm/omega
  if (_periodEmaUs > 0.0f) {
    const float rev_per_s = 1.0e6f / ( (_countsPerRev * _periodEmaUs) );
    float rpm   = 60.0f * rev_per_s;
    float omega = 2.0f * PI * rev_per_s;
    if (_cfg.invert) { rpm = -rpm; omega = -omega; }
    _rpm = rpm; _omega = omega; _lastSeenMs = millis();
    _totalCount += 1;
  }

  // 4) Avanza sector según dirección actual (+1: k++, -1: k--)
  if (_stepDir > 0) {
    _sectorIdx = (_sectorIdx + 1) % _ppr;
  } else {
    _sectorIdx = (_sectorIdx == 0) ? (_ppr - 1) : (_sectorIdx - 1);
  }
}
