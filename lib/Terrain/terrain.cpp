// terrain.cpp - see terrain.h for the tile format and threading rules.

#include "terrain.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// ---------------------------------------------------------------------------
// File backend: SD on the ESP32, stdio on a PC (host tests read the real
// tiles). Everything above this block is backend-independent.
// ---------------------------------------------------------------------------
#ifdef ARDUINO
#include <SD.h>
#include <SPI.h>
namespace {
struct TileFile {
    File f;
    bool open(const char *path) { f = SD.open(path, FILE_READ); return static_cast<bool>(f); }
    bool readAt(long offset, void *buf, size_t n) {
        return f.seek(offset) && f.read(static_cast<uint8_t *>(buf), n) == n;
    }
    long size() { return (long)f.size(); }
    void close() { if (f) f.close(); }
};
bool mountAndCheck(uint8_t csPin, const char *dir) { return SD.begin(csPin) && SD.exists(dir); }
}  // namespace
#else
namespace {
struct TileFile {
    FILE *fp = nullptr;
    bool open(const char *path) { fp = fopen(path, "rb"); return fp != nullptr; }
    bool readAt(long offset, void *buf, size_t n) {
        return fseek(fp, offset, SEEK_SET) == 0 && fread(buf, 1, n, fp) == n;
    }
    long size() { return fseek(fp, 0, SEEK_END) == 0 ? ftell(fp) : -1; }
    void close() { if (fp) { fclose(fp); fp = nullptr; } }
};
bool mountAndCheck(uint8_t, const char *dir) {
    char probe[160];
    snprintf(probe, sizeof(probe), "%s/.", dir);
    FILE *fp = fopen(probe, "rb");   // directories open on POSIX; Windows fails -
    if (fp) { fclose(fp); }          // so don't require it, lookups report NAN anyway
    return true;
}
}  // namespace
#endif

namespace terrain {

namespace {
constexpr int     kN         = 3600;     // samples per degree / per tile edge
constexpr int16_t kVoid      = -32768;
constexpr int     kMaxRadius = 4;
constexpr int     kSlots     = 4;        // open tile files kept (LRU)
constexpr long    kTileBytes = (long)kN * kN * (long)sizeof(int16_t);   // 25,920,000

char dir_[96] = "/vtol_bin_tiles";
bool began_   = false;

struct Slot {
    int      lat0 = 0, lon0 = 0;
    bool     open = false;
    uint32_t lastUse = 0;
    TileFile file;
};
Slot     slots_[kSlots];
uint32_t useCounter_ = 0;

long floorDiv(long a, long b) { return (a >= 0) ? a / b : -((-a + b - 1) / b); }

// Global integer sample grid: gy counts northward from the equator, gx
// eastward from Greenwich, one per arcsecond.
void gridToTile(long gy, long gx, int &lat0, int &lon0, int &row, int &col) {
    lat0 = (int)floorDiv(gy, kN);
    lon0 = (int)floorDiv(gx, kN);
    row  = kN - 1 - (int)(gy - (long)lat0 * kN);   // row 0 = north edge
    col  = (int)(gx - (long)lon0 * kN);
}

TileFile *tile(int lat0, int lon0) {
    if (lat0 < 0 || lon0 < 0 || lat0 > 89 || lon0 > 179) return nullptr;   // N/E only
    Slot *victim = &slots_[0];
    for (Slot &s : slots_) {
        if (s.open && s.lat0 == lat0 && s.lon0 == lon0) { s.lastUse = ++useCounter_; return &s.file; }
        if (!s.open) victim = &s;
        else if (victim->open && s.lastUse < victim->lastUse) victim = &s;
    }
    if (victim->open) victim->file.close();
    victim->open = false;
    char path[128];
    snprintf(path, sizeof(path), "%s/N%02dE%03d.bin", dir_, lat0, lon0);
    if (!victim->file.open(path)) return nullptr;
    if (victim->file.size() != kTileBytes) {   // truncated copy: never trust it
        victim->file.close();
        return nullptr;
    }
    victim->open = true;
    victim->lat0 = lat0;
    victim->lon0 = lon0;
    victim->lastUse = ++useCounter_;
    return &victim->file;
}

// Max over `count` consecutive samples of one row (one read). Little-endian
// file into a little-endian CPU (ESP32 and x86 both are), so no swapping.
bool rowRunMax(int lat0, int lon0, int row, int col, int count, float &best) {
    TileFile *f = tile(lat0, lon0);
    if (f == nullptr) return false;
    int16_t buf[2 * kMaxRadius + 1];
    if (!f->readAt(byteOffset(row, col), buf, (size_t)count * sizeof(int16_t))) return false;
    for (int i = 0; i < count; ++i) {
        if (buf[i] == kVoid) return false;
        if (buf[i] > best) best = buf[i];
    }
    return true;
}
}  // namespace

void sampleIndex(double lat_deg, double lon_deg, int &tile_lat0, int &tile_lon0,
                 int &row, int &col) {
    gridToTile((long)floor(lat_deg * kN), (long)floor(lon_deg * kN), tile_lat0, tile_lon0, row, col);
}

long byteOffset(int row, int col) {
    return ((long)row * kN + col) * (long)sizeof(int16_t);
}

bool begin(uint8_t csPin, const char *dir) {
    snprintf(dir_, sizeof(dir_), "%s", dir);
    for (Slot &s : slots_) { if (s.open) s.file.close(); s.open = false; }
    began_ = mountAndCheck(csPin, dir_);
    return began_;
}

float lookupMaxElevation(double lat_deg, double lon_deg, int r) {
    if (!began_ || r < 0 || r > kMaxRadius) return NAN;
    const long gy0 = (long)floor(lat_deg * kN);
    const long gx0 = (long)floor(lon_deg * kN);
    float best = -1.0e9f;
    for (long gy = gy0 - r; gy <= gy0 + r; ++gy) {
        // One read per row, split where the run crosses into the next tile east.
        long gx = gx0 - r;
        int remaining = 2 * r + 1;
        while (remaining > 0) {
            int lat0, lon0, row, col;
            gridToTile(gy, gx, lat0, lon0, row, col);
            const int n = (remaining < kN - col) ? remaining : kN - col;
            if (!rowRunMax(lat0, lon0, row, col, n, best)) return NAN;
            gx += n;
            remaining -= n;
        }
    }
    return best;
}

float lookupElevation(double lat_deg, double lon_deg) {
    return lookupMaxElevation(lat_deg, lon_deg, 0);
}

bool tileAvailable(double lat_deg, double lon_deg) {
    if (!began_) return false;
    int lat0, lon0, row, col;
    sampleIndex(lat_deg, lon_deg, lat0, lon0, row, col);
    return tile(lat0, lon0) != nullptr;
}

RouteCheck checkRoute(float length_m, RoutePointFn pointAt, float step_m, float margin_m,
                      int radius) {
    RouteCheck rc;
    if (!began_ || pointAt == nullptr || !(step_m > 0.0f)) return rc;

    const double kMPerDegLat = 111132.0;   // = 30.87 m/arcsec, as fcode uses
    const int kMaxTiles = 32;
    int seenLat[kMaxTiles], seenLon[kMaxTiles];

    for (float s = 0.0f; ; s += step_m) {
        const float ss = (s < length_m) ? s : length_m;
        double lat, lon;
        bool good = pointAt(ss, lat, lon);
        if (good) {
            const float e = lookupMaxElevation(lat, lon, radius);
            if (isnan(e)) good = false;
            else if (e > rc.max_elev_m) rc.max_elev_m = e;
        }
        if (good) {
            // The corners of the +/-margin box around the point reach every
            // tile the box touches (it is far smaller than a tile).
            const double dLat = margin_m / kMPerDegLat;
            const double dLon = margin_m / (kMPerDegLat * cos(lat * 3.14159265358979 / 180.0));
            for (int i = 0; i < 4 && good; ++i) {
                const double la = lat + ((i & 1) ? dLat : -dLat);
                const double lo = lon + ((i & 2) ? dLon : -dLon);
                if (!tileAvailable(la, lo)) { good = false; break; }
                int lat0, lon0, row, col;
                sampleIndex(la, lo, lat0, lon0, row, col);
                bool seen = false;
                for (int k = 0; k < rc.tiles; ++k)
                    if (seenLat[k] == lat0 && seenLon[k] == lon0) { seen = true; break; }
                if (!seen && rc.tiles < kMaxTiles) {
                    seenLat[rc.tiles] = lat0;
                    seenLon[rc.tiles] = lon0;
                    ++rc.tiles;
                }
            }
        }
        ++rc.points;
        if (!good && rc.missing++ == 0) rc.first_missing_m = ss;
        if (ss >= length_m) break;
    }
    rc.ok = (rc.points > 0 && rc.missing == 0);
    return rc;
}

}  // namespace terrain
