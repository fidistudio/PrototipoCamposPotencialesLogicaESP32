#ifndef SECTOR_CALIBRATOR_H
#define SECTOR_CALIBRATOR_H

#include <Arduino.h>
#include <Preferences.h>

// ================================================
// SectorCalibrator
// - Corrige no-uniformidad de imanes (LUT s[k])
// - Calibración multi-vuelta por sector
// - Construye patrón normalizado (1/s[k])
// - Auto-alineación: estima offset inicial
// ================================================
class SectorCalibrator {
public:
  struct Config {
    const char* nvsNamespace = "encoder"; // distinto por rueda: "encR", "encL", etc.
    const char* nvsKeyUse    = "use_lut";
    const char* nvsKeyLut    = "lut";
    uint16_t    ppr;                     // nº de sectores (pulsos por vuelta)
    uint8_t     maxLaps = 12;            // límite seguridad
    bool        useLUTByDefault = true;  // si no hay NVS
  };

  explicit SectorCalibrator(const Config& cfg);
  ~SectorCalibrator();

  // Persistencia
  void   load();           // lee LUT (o init=1.0) y construye patrón
  void   save();           // guarda LUT + flag
  void   clear();          // LUT=1.0, useLUT=false, guarda

  // Estado LUT/patrón
  bool   useLUT() const { return _useLUT; }
  void   setUseLUT(bool on) { _useLUT = on; }
  float  scale(uint16_t k) const { return _lut[k]; }  // s[k]
  bool   patternReady() const { return _patternReady; }

  // Corrige periodo por sector: dt_corr = dt * s[k]
  inline float correctDt(uint16_t k, float dt_us) const {
    if (!_useLUT) return dt_us;
    return dt_us * _lut[k];
  }

  // Calibración multivuelta
  bool   startCalibration(uint8_t lapsN);  // devuelve false si inválido
  bool   isCalibrating() const { return _calibActive; }
  void   feedPeriod(uint16_t sectorK, float dt_us); // usar durante calib/align
  bool   finishCalibrationIfReady();        // compute LUT cuando se cumpla

  // Auto-alineación
  bool   startAlignment(uint8_t lapsN);     // requiere patternReady()=true
  bool   isAligning() const { return _alignActive; }
  bool   finishAlignmentIfReady(uint16_t& bestOffsetOut, float& scoreOut);

  // Debug opcional
  void   printLUT(Stream& s = Serial) const;
  void   printSectorStats(Stream& s = Serial) const; // última calib

  // Logging (opcional)
  void   setLog(Stream* s) { _log = s; }

private:
  // Helpers
  void   _alloc();
  void   _free();
  void   _buildPatternFromLUT();          // pattern[k] = (1/s[k]) / mean(1/s)

  // Calib helpers
  void   _resetCalibBuffers();
  float  _trimmedMean(float* vals, uint8_t n) const;

  // Align helpers
  void   _resetAlignBuffers();
  bool   _bestOffsetSingleLap(uint8_t lapIdx, uint16_t& bestOff, float& bestScore) const;

private:
  Config      _cfg;
  Preferences _prefs;

  // LUT y patrón
  float*  _lut            = nullptr; // s[k]
  float*  _pattern        = nullptr; // normalizado 1/s[k]
  bool    _useLUT         = true;
  bool    _patternReady   = false;

  // Calibración
  bool    _calibActive    = false;
  uint8_t _calibTargetN   = 0;
  uint8_t _calibLap       = 0;
  float*  _dtBuf          = nullptr; // [ppr x maxLaps] tiempos por sector y vuelta
  bool*   _dtFilled       = nullptr; // [ppr x maxLaps]

  // Alineación
  bool    _alignActive    = false;
  uint8_t _alignTargetN   = 0;
  uint8_t _alignLap       = 0;
  float*  _alignBuf       = nullptr; // [ppr x maxLaps]

  // Logging
  Stream* _log = nullptr;
};

#endif // SECTOR_CALIBRATOR_H
