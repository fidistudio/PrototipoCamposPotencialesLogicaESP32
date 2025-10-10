#ifndef SECTOR_CALIBRATOR_H
#define SECTOR_CALIBRATOR_H

#include <Arduino.h>
#include <Preferences.h>

// ================================================
// SectorCalibrator (dual-LUT por sentido)
// - Corrige no-uniformidad de imanes con LUT s_fwd[k] / s_rev[k]
// - Calibración multi-vuelta por sector, por sentido
// - Construye patrón normalizado (1/s[k]) por sentido
// - Auto-alineación por sentido: estima y guarda offset
// - Retro-compatibilidad con una sola LUT en NVS
// ================================================
class SectorCalibrator {
public:
  struct Config {
    const char* nvsNamespace = "encoder"; // distinto por rueda: "encR", "encL", etc.

    // Claves nuevas (dual)
    const char* nvsKeyUseFwd = "use_fwd";
    const char* nvsKeyUseRev = "use_rev";
    const char* nvsKeyLutFwd = "lut_fwd";
    const char* nvsKeyLutRev = "lut_rev";
    const char* nvsKeyOffFwd = "off_fwd";
    const char* nvsKeyOffRev = "off_rev";

    // Claves legacy (single) para migración
    const char* nvsKeyUse    = "use_lut";
    const char* nvsKeyLut    = "lut";

    uint16_t    ppr;                     // nº de sectores (pulsos por vuelta)
    uint8_t     maxLaps = 12;            // límite seguridad
    bool        useLUTByDefault = true;  // si no hay NVS
  };

  explicit SectorCalibrator(const Config& cfg);
  ~SectorCalibrator();

  // Persistencia
  void   load();    // lee LUTs/flags/offsets y construye patrones (con migración desde single)
  void   save();    // guarda LUTs/flags/offsets
  void   clear();   // LUT_fwd/REV = 1.0, use=false, offsets=0, guarda

  // Estado LUT/patrón
  bool   useLUTFwd() const { return _useFwd; }
  bool   useLUTRev() const { return _useRev; }
  void   setUseLUTFwd(bool on) { _useFwd = on; }
  void   setUseLUTRev(bool on) { _useRev = on; }

  uint16_t offsetFwd() const { return _offFwd; }
  uint16_t offsetRev() const { return _offRev; }

  bool   patternFwdReady() const { return _patFwdReady; }
  bool   patternRevReady() const { return _patRevReady; }

  float  scaleFwd(uint16_t k) const { return _lutFwd[k]; }  // s_fwd[k]
  float  scaleRev(uint16_t k) const { return _lutRev[k]; }  // s_rev[k]

  // Corrige periodo por sector y sentido: dt_corr = dt * s_dir[(k+off_dir)%PPR]
  inline float correctDtDir(uint16_t k, float dt_us, int stepDir) const {
    const bool forward = (stepDir >= 0);
    const uint16_t off = forward ? _offFwd : _offRev;
    const uint16_t idx = (k + off) % _cfg.ppr;
    if (forward)  return _useFwd ? dt_us * _lutFwd[idx] : dt_us;
    else          return _useRev ? dt_us * _lutRev[idx] : dt_us;
  }

  // ---- Calibración multivuelta por sentido ----
  // stepDir: +1 (fwd) / -1 (rev)
  bool   startCalibrationDir(uint8_t lapsN, int stepDir);
  bool   isCalibrating() const { return _calibActive; }
  void   feedPeriod(uint16_t sectorK, float dt_us); // usar durante calib/align
  bool   finishCalibrationIfReady();                // compute LUT_dir cuando se cumpla

  // ---- Auto-alineación por sentido ----
  bool   startAlignmentDir(uint8_t lapsN, int stepDir);   // requiere patrón listo en ese sentido
  bool   isAligning() const { return _alignActive; }
  bool   finishAlignmentIfReady(uint16_t& bestOffsetOut, float& scoreOut); // guarda internamente off_fwd/rev

  // Debug opcional
  void   printLUT(Stream& s = Serial) const;
  void   printSectorStats(Stream& s = Serial) const; // última calib (del sentido activo)

  // Logging (opcional)
  void   setLog(Stream* s) { _log = s; }

private:
  // Helpers
  void   _alloc();
  void   _free();

  void   _buildPatternFromLUT_Fwd(); // pattern_fwd[k] = (1/s_fwd[k]) / mean(1/s_fwd)
  void   _buildPatternFromLUT_Rev(); // pattern_rev[k] = (1/s_rev[k]) / mean(1/s_rev)

  // Calib helpers (buffers temporales reutilizados para uno u otro sentido)
  void   _resetCalibBuffers();
  float  _trimmedMean(float* vals, uint8_t n) const;

  // Align helpers
  void   _resetAlignBuffers();
  bool   _bestOffsetSingleLap(uint8_t lapIdx, uint16_t& bestOff, float& bestScore, bool forward) const;

private:
  Config      _cfg;
  Preferences _prefs;

  // LUTs y patrones por sentido
  float*  _lutFwd   = nullptr; // s_fwd[k]
  float*  _lutRev   = nullptr; // s_rev[k]
  float*  _patFwd   = nullptr; // normalizado 1/s_fwd[k]
  float*  _patRev   = nullptr; // normalizado 1/s_rev[k]
  bool    _useFwd   = true;
  bool    _useRev   = true;
  bool    _patFwdReady = false;
  bool    _patRevReady = false;

  // Offsets por sentido (aplicados solo dentro de correctDtDir)
  uint16_t _offFwd = 0;
  uint16_t _offRev = 0;

  // Estado "qué sentido estoy calibrando/alineando"
  int8_t   _modeDir = +1; // +1 fwd, -1 rev

  // Calibración (buffers temporales)
  bool     _calibActive    = false;
  uint8_t  _calibTargetN   = 0;
  uint8_t  _calibLap       = 0;
  float*   _dtBuf          = nullptr; // [ppr x maxLaps] tiempos por sector y vuelta
  bool*    _dtFilled       = nullptr; // [ppr x maxLaps]

  // Alineación (buffers temporales)
  bool     _alignActive    = false;
  uint8_t  _alignTargetN   = 0;
  uint8_t  _alignLap       = 0;
  float*   _alignBuf       = nullptr; // [ppr x maxLaps]

  // Logging
  Stream*  _log = nullptr;
};

#endif // SECTOR_CALIBRATOR_H
