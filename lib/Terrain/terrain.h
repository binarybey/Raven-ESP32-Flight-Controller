// terrain.h - on-demand elevation lookups from the DEM tiles on the SD card
// (Copernicus GLO-30, converted by tar-to-bin.py).
//
// TILE FORMAT (as written by tar-to-bin.py):
//   - one file per 1x1 degree, named by its SOUTH-WEST corner:
//     N39E032.bin covers 39..40 N, 32..33 E
//   - 3600 x 3600 samples, 1 arcsecond spacing, int16 little-endian, metres
//     (rounded; sea/negative clamped to 0) = 25,920,000 bytes
//   - row-major, row 0 = NORTH edge, column 0 = WEST edge
//   - voids = -32768: treated as "no data", never as a low elevation
//
// GLO-30 is a surface model (DSM): tree canopy and buildings are included,
// which makes ground clearance conservative over forest and towns.
//
// North-east hemisphere only (matches the tile naming).
//
// SD CARD: FAT32. The ESP32 SD library does not mount exFAT, and cards over
// 32 GB usually ship exFAT - reformat them. Copy the vtol_bin_tiles folder
// to the card root (~3 GB for the current 114 tiles).
//
// Every lookup seeks straight to a byte offset - no tile is ever loaded
// into RAM. Up to 4 tile files stay open (least-recently-used), so lookups
// near a tile corner don't reopen files. NOT thread-safe: call from ONE task
// (main.cpp's TaskTerrain, plus the boot-time route check before tasks
// start) - never from the control loop, SD reads take milliseconds.

#pragma once

#include <stdint.h>

namespace terrain {

// Call once at boot. On the ESP32 this mounts the SD card (a no-op if the
// mission loader already did) and checks that `dir` exists. On a PC (host
// tests) `dir` is a normal filesystem path and csPin is ignored.
bool begin(uint8_t csPin, const char *dir = "/vtol_bin_tiles");

// Elevation (m) of the sample containing the position, or NAN if
// unavailable (tile missing, void, read failure, outside N/E).
float lookupElevation(double lat_deg, double lon_deg);

// Maximum elevation over the (2r+1) x (2r+1) samples centred on the
// position (r <= 4; each sample is ~31 m N-S, ~24 m E-W at 40 N). NAN if ANY
// sample in the footprint is unavailable - unknown terrain is never assumed
// low. Use this for clearance decisions: r=2 covers GPS error plus the
// half-sample ambiguity of the grid registration.
float lookupMaxElevation(double lat_deg, double lon_deg, int radius_samples);

// ---- exposed for testing - pure math, no file I/O ----
// Which tile (south-west corner) and sample (row from the north edge,
// column from the west edge) a position falls in.
void sampleIndex(double lat_deg, double lon_deg, int &tile_lat0, int &tile_lon0,
                 int &row, int &col);

// Byte offset of a sample within its tile: (row*3600 + col) * 2.
long byteOffset(int row, int col);

}  // namespace terrain
