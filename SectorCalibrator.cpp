#include "SectorCalibrator.h"
#include <math.h>
#include <string.h>

static inline size_t idx2D(uint16_t k, uint8_t lap, uint16_t ppr) {
  return (size_t)k + (size_t)lap * (size_t)ppr;
}

#define SC_LOGF(fmt, ...) do { if (_log) _log->printf(fmt, ##__VA_ARGS__); } while(0)

SectorCalibrator::SectorCalibrator(const Config& cfg) : _cfg(cfg) {
  _alloc();
}

SectorCalibrator::~SectorCalibrator() {
  _free();
}

void SectorCalibrator::_alloc() {
  const size_t nSectors = (size_t)_cfg.ppr;
  const size_t nCells   = nSectors * (size_t)_cfg.maxLaps;

  _lutFwd   = new float[nSectors];
  _lutRev   = new float[nSectors];
  _patFwd   = new float[nSectors];
  _patRev   = new float[nSectors];
  _dtBuf    = new float[nCells];
  _dtFilled = new bool [nCells];
  _alignBuf = new float[nCells];

  for (uint16_t k=0;k<_cfg.ppr;k++) { _lutFwd[k] = 1.0f; _lutRev[k] = 1.0f; }
}

void SectorCalibrator::_free() {
  delete[] _lutFwd;   _lutFwd = nullptr;
  delete[] _lutRev;   _lutRev = nullptr;
  delete[] _patFwd;   _patFwd = nullptr;
  delete[] _patRev;   _patRev = nullptr;
  delete[] _dtBuf;    _dtBuf = nullptr;
  delete[] _dtFilled; _dtFilled = nullptr;
  delete[] _alignBuf; _alignBuf = nullptr;
}

// ---------------- Persistencia ----------------
void SectorCalibrator::load() {
  _prefs.begin(_cfg.nvsNamespace, true);

  // Intentar leer dual
  const size_t need = (size_t)_cfg.ppr * sizeof(float);

  bool haveFwd = _prefs.isKey(_cfg.nvsKeyLutFwd) && _prefs.getBytesLength(_cfg.nvsKeyLutFwd) == need;
  bool haveRev = _prefs.isKey(_cfg.nvsKeyLutRev) && _prefs.getBytesLength(_cfg.nvsKeyLutRev) == need;

  if (haveFwd) _prefs.getBytes(_cfg.nvsKeyLutFwd, _lutFwd, need);
  if (haveRev) _prefs.getBytes(_cfg.nvsKeyLutRev, _lutRev, need);

  // Flags y offsets
  _useFwd = _prefs.getBool(_cfg.nvsKeyUseFwd, _cfg.useLUTByDefault);
  _useRev = _prefs.getBool(_cfg.nvsKeyUseRev, _cfg.useLUTByDefault);
  _offFwd = _prefs.getUShort(_cfg.nvsKeyOffFwd, 0);
  _offRev = _prefs.getUShort(_cfg.nvsKeyOffRev, 0);

  // Migración desde single LUT si no había dual
  if (!haveFwd && !haveRev) {
    // Leer legacy si existe
    bool useLegacy = _prefs.getBool(_cfg.nvsKeyUse, _cfg.useLUTByDefault);
    if (_prefs.isKey(_cfg.nvsKeyLut) && _prefs.getBytesLength(_cfg.nvsKeyLut) == need) {
      _prefs.getBytes(_cfg.nvsKeyLut, _lutFwd, need);
      for (uint16_t k=0;k<_cfg.ppr;k++) _lutRev[k] = 1.0f; // inicia REV neutra
      _useFwd = useLegacy;
      _useRev = useLegacy; // o false, si prefieres empezar sin REV
      SC_LOGF("[LUT] migrated legacy -> dual (REV=1.0)\n");
    } else {
      for (uint16_t k=0;k<_cfg.ppr;k++) { _lutFwd[k]=1.0f; _lutRev[k]=1.0f; }
      _useFwd = _useRev = _cfg.useLUTByDefault;
    }
  } else {
    // Completar faltantes con 1.0
    if (!haveFwd) for (uint16_t k=0;k<_cfg.ppr;k++) _lutFwd[k] = 1.0f;
    if (!haveRev) for (uint16_t k=0;k<_cfg.ppr;k++) _lutRev[k] = 1.0f;
  }

  _prefs.end();

  // Construye patrones
  _buildPatternFromLUT_Fwd();
  _buildPatternFromLUT_Rev();
}

void SectorCalibrator::save() {
  _prefs.begin(_cfg.nvsNamespace, false);

  const size_t need = (size_t)_cfg.ppr * sizeof(float);
  _prefs.putBool(_cfg.nvsKeyUseFwd, _useFwd);
  _prefs.putBool(_cfg.nvsKeyUseRev, _useRev);
  _prefs.putUShort(_cfg.nvsKeyOffFwd, _offFwd);
  _prefs.putUShort(_cfg.nvsKeyOffRev, _offRev);
  _prefs.putBytes(_cfg.nvsKeyLutFwd, _lutFwd, need);
  _prefs.putBytes(_cfg.nvsKeyLutRev, _lutRev, need);

  _prefs.end();

  _buildPatternFromLUT_Fwd();
  _buildPatternFromLUT_Rev();
}

void SectorCalibrator::clear() {
  for (uint16_t k=0;k<_cfg.ppr;k++) { _lutFwd[k]=1.0f; _lutRev[k]=1.0f; }
  _useFwd = _useRev = false;
  _offFwd = _offRev = 0;
  save();
}

// ---------------- Patrones ----------------
void SectorCalibrator::_buildPatternFromLUT_Fwd() {
  float sum = 0.0f, minv = 1e30f, maxv = -1e30f;
  for (uint16_t k=0;k<_cfg.ppr;k++) {
    float p = (_lutFwd[k] != 0.0f) ? (1.0f / _lutFwd[k]) : 1.0f;
    _patFwd[k] = p;
    sum += p;
    if (p < minv) minv=p;
    if (p > maxv) maxv=p;
  }
  float mean = (sum>0.0f)? (sum / (float)_cfg.ppr) : 1.0f;
  if (mean <= 0.0f) mean = 1.0f;
  for (uint16_t k=0;k<_cfg.ppr;k++) _patFwd[k] /= mean;
  _patFwdReady = (maxv - minv) > 1e-3f;
  SC_LOGF("[PATTERN FWD] ready=%d (range=%.6f)\n", _patFwdReady?1:0, (double)(maxv - minv));
}

void SectorCalibrator::_buildPatternFromLUT_Rev() {
  float sum = 0.0f, minv = 1e30f, maxv = -1e30f;
  for (uint16_t k=0;k<_cfg.ppr;k++) {
    float p = (_lutRev[k] != 0.0f) ? (1.0f / _lutRev[k]) : 1.0f;
    _patRev[k] = p;
    sum += p;
    if (p < minv) minv=p;
    if (p > maxv) maxv=p;
  }
  float mean = (sum>0.0f)? (sum / (float)_cfg.ppr) : 1.0f;
  if (mean <= 0.0f) mean = 1.0f;
  for (uint16_t k=0;k<_cfg.ppr;k++) _patRev[k] /= mean;
  _patRevReady = (maxv - minv) > 1e-3f;
  SC_LOGF("[PATTERN REV] ready=%d (range=%.6f)\n", _patRevReady?1:0, (double)(maxv - minv));
}

// ---------------- Calibración ----------------
bool SectorCalibrator::startCalibrationDir(uint8_t lapsN, int stepDir) {
  if (lapsN==0 || lapsN>_cfg.maxLaps) return false;
  _modeDir = (stepDir >= 0) ? +1 : -1;
  _calibTargetN = lapsN;
  _calibLap = 0;
  _calibActive = true;
  _resetCalibBuffers();
  SC_LOGF("[CAL %s] start N=%u\n", (_modeDir>=0)?"FWD":"REV", lapsN);
  return true;
}

void SectorCalibrator::_resetCalibBuffers() {
  for (uint8_t j=0;j<_calibTargetN;j++) {
    for (uint16_t k=0;k<_cfg.ppr;k++) {
      _dtBuf[idx2D(k,j,_cfg.ppr)]    = 0.0f;
      _dtFilled[idx2D(k,j,_cfg.ppr)] = false;
    }
  }
}

void SectorCalibrator::feedPeriod(uint16_t sectorK, float dt_us) {
  // Calibración
  if (_calibActive) {
    if (_calibLap < _calibTargetN) {
      const size_t id = idx2D(sectorK, _calibLap, _cfg.ppr);
      _dtBuf[id] = dt_us;
      _dtFilled[id] = true;

      if (sectorK == _cfg.ppr-1) {
        _calibLap++;
        SC_LOGF("[CAL %s] lap %u/%u\n", (_modeDir>=0)?"FWD":"REV", _calibLap, _calibTargetN);
      }
    }
  }

  // Alineación
  if (_alignActive) {
    if (_alignLap < _alignTargetN) {
      const size_t id = idx2D(sectorK, _alignLap, _cfg.ppr);
      _alignBuf[id] = dt_us;
      if (sectorK == _cfg.ppr-1) {
        _alignLap++;
        SC_LOGF("[ALIGN %s] lap %u/%u\n", (_modeDir>=0)?"FWD":"REV", _alignLap, _alignTargetN);
      }
    }
  }
}

bool SectorCalibrator::finishCalibrationIfReady() {
  if (!_calibActive) return false;
  if (_calibLap < _calibTargetN) return false;

  // media por sector (con trimming)
  float* sectorMean = new float[_cfg.ppr];
  float globalSum = 0.0f; uint32_t globalCount = 0;

  for (uint16_t k=0;k<_cfg.ppr;k++) {
    float tmp[16]; // maxLaps<=12
    uint8_t n=0;
    for (uint8_t j=0;j<_calibTargetN;j++) {
      const size_t id = idx2D(k,j,_cfg.ppr);
      if (_dtFilled[id]) tmp[n++] = _dtBuf[id];
    }
    float mk = (n>0) ? _trimmedMean(tmp, n) : 0.0f;
    sectorMean[k] = mk;
    if (mk>0.0f) { globalSum += mk; globalCount++; }
  }

  bool ok = (globalCount > 0);
  if (ok) {
    const float globalMean = globalSum / (float)globalCount;
    float* lut = (_modeDir>=0) ? _lutFwd : _lutRev;

    for (uint16_t k=0;k<_cfg.ppr;k++) {
      float mk = sectorMean[k];
      if (mk <= 0.0f) mk = globalMean;
      lut[k] = globalMean / mk; // s[k] = mean / sectorMean
    }
    // Estadísticas rápidas
    float minv=1e9f, maxv=-1e9f, sum=0.f;
    for (uint16_t k=0;k<_cfg.ppr;k++) {
      float s=lut[k];
      if (s<minv) minv=s;
      if (s>maxv) maxv=s;
      sum+=s;
    }
    const float mean = sum / (float)_cfg.ppr;

    // Reconstruir patrón del sentido calibrado
    if (_modeDir>=0) _buildPatternFromLUT_Fwd();
    else            _buildPatternFromLUT_Rev();

    save();
    SC_LOGF("[CAL %s] OK: LUT saved. s[k] min=%.6f max=%.6f mean=%.6f\n",
            (_modeDir>=0)?"FWD":"REV", (double)minv,(double)maxv,(double)mean);
  }

  delete[] sectorMean;
  _calibActive = false;
  return ok;
}

float SectorCalibrator::_trimmedMean(float* vals, uint8_t n) const {
  if (n==0) return 0.0f;
  if (n<=2) {
    float s=0; for (uint8_t i=0;i<n;i++) s+=vals[i];
    return s/(float)n;
  }
  // descarta min y max
  uint8_t iMin=0, iMax=0;
  for (uint8_t i=1;i<n;i++) { if (vals[i]<vals[iMin]) iMin=i; if (vals[i]>vals[iMax]) iMax=i; }
  float s=0.0f; uint8_t cnt=0;
  for (uint8_t i=0;i<n;i++) { if (i==iMin||i==iMax) continue; s+=vals[i]; cnt++; }
  return (cnt>0)? s/(float)cnt : 0.0f;
}

// ---------------- Alineación ----------------
bool SectorCalibrator::startAlignmentDir(uint8_t lapsN, int stepDir) {
  const bool forward = (stepDir >= 0);
  if (lapsN==0 || lapsN>_cfg.maxLaps) return false;
  if (forward && !_patFwdReady) return false;
  if (!forward && !_patRevReady) return false;

  _modeDir = forward ? +1 : -1;
  _alignTargetN = lapsN;
  _alignLap = 0;
  _alignActive = true;
  _resetAlignBuffers();
  SC_LOGF("[ALIGN %s] start N=%u\n", forward?"FWD":"REV", lapsN);
  return true;
}

void SectorCalibrator::_resetAlignBuffers() {
  for (uint8_t j=0;j<_alignTargetN;j++) {
    for (uint16_t k=0;k<_cfg.ppr;k++) _alignBuf[idx2D(k,j,_cfg.ppr)] = 0.0f;
  }
}

bool SectorCalibrator::_bestOffsetSingleLap(uint8_t lapIdx, uint16_t& bestOff, float& bestScore, bool forward) const {
  // normaliza ventana por su media y busca shift que minimiza error L1 vs patrón del sentido elegido
  float sum=0.0f;
  for (uint16_t k=0;k<_cfg.ppr;k++) sum += _alignBuf[idx2D(k,lapIdx,_cfg.ppr)];
  if (sum<=0) return false;
  const float mean = sum / (float)_cfg.ppr;

  const float* pattern = forward ? _patFwd : _patRev;

  bestScore = 1e30f; bestOff = 0;
  for (uint16_t shift=0; shift<_cfg.ppr; ++shift) {
    float err=0.0f;
    for (uint16_t k=0;k<_cfg.ppr;k++) {
      float win = _alignBuf[idx2D(k,lapIdx,_cfg.ppr)] / mean;
      float exp = pattern[(k+shift)%_cfg.ppr];
      float e = win - exp;
      if (e < 0) e = -e;
      err += e; // L1
    }
    float score = err / (float)_cfg.ppr;
    if (score < bestScore) { bestScore = score; bestOff = shift; }
  }
  return true;
}

bool SectorCalibrator::finishAlignmentIfReady(uint16_t& bestOffsetOut, float& scoreOut) {
  if (!_alignActive) return false;
  if (_alignLap < _alignTargetN) return false;

  const bool forward = (_modeDir >= 0);

  // Vota entre laps
  uint16_t* votes = new uint16_t[_cfg.ppr];
  for (uint16_t k=0;k<_cfg.ppr;k++) votes[k]=0;

  float bestGlobalScore = 1e30f;
  uint16_t bestGlobalOff = 0;

  for (uint8_t j=0;j<_alignTargetN;j++) {
    uint16_t off; float sc;
    if (_bestOffsetSingleLap(j, off, sc, forward)) {
      votes[off]++;
      if (sc < bestGlobalScore) { bestGlobalScore=sc; bestGlobalOff=off; }
      SC_LOGF("[ALIGN %s] lap %u bestOff=%u score=%.4f\n",
              forward?"FWD":"REV", (unsigned)(j+1), (unsigned)off, (double)sc);
    }
  }

  // Mayoría
  uint16_t finalOff=bestGlobalOff, maxVotes=0;
  for (uint16_t k=0;k<_cfg.ppr;k++) {
    if (votes[k] > maxVotes) { maxVotes=votes[k]; finalOff=k; }
  }
  delete[] votes;

  bestOffsetOut = finalOff;
  scoreOut      = bestGlobalScore;

  // Guardar offset según sentido — sin tocar índices en EncoderPCNT
  if (forward) _offFwd = finalOff;
  else         _offRev = finalOff;

  _alignActive = false;
  save(); // persiste nuevo offset
  SC_LOGF("[ALIGN %s] done: offset=%u score=%.4f\n", forward?"FWD":"REV",
          (unsigned)finalOff, (double)bestGlobalScore);
  return true;
}

// ---------------- Debug ----------------
void SectorCalibrator::printLUT(Stream& s) const {
  s.printf("useFWD=%d offFWD=%u | useREV=%d offREV=%u\n",
           _useFwd?1:0, _offFwd, _useRev?1:0, _offRev);
  s.println("[FWD] s_fwd[k]:");
  for (uint16_t k=0;k<_cfg.ppr;k++) s.printf("sF[%2u]=%.6f\n", k, _lutFwd[k]);
  s.println("[REV] s_rev[k]:");
  for (uint16_t k=0;k<_cfg.ppr;k++) s.printf("sR[%2u]=%.6f\n", k, _lutRev[k]);
}

void SectorCalibrator::printSectorStats(Stream& s) const {
  if (!_calibTargetN) { s.println("Sin estadísticos (aún no calibras)."); return; }
  s.printf("Sentido activo: %s\n", (_modeDir>=0)?"FWD":"+REV");
  s.println("Tiempos medios por sector (us):");
  for (uint16_t k=0;k<_cfg.ppr;k++) {
    float sum=0, minv=1e30f, maxv=-1e30f; uint8_t cnt=0;
    for (uint8_t j=0;j<_calibTargetN;j++) {
      float v = _dtBuf[idx2D(k,j,_cfg.ppr)];
      if (v>0) { sum+=v; cnt++; if (v<minv) minv=v; if (v>maxv) maxv=v; }
    }
    float mean = (cnt>0)? sum/(float)cnt : 0.0f;
    s.printf("k=%2u: mean=%.1f (min=%.1f max=%.1f) n=%u\n", k, mean, minv, maxv, cnt);
  }
}
