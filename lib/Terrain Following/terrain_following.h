// terrain_following.h - joins the mission path (F-Code) with the DEM
// (lib/Terrain) so the F43 clearance is held above real ground.
//
// The clearance is measured to the highest DEM sample under the vehicle
// (5x5 samples, ~+/-60 m: GPS error + grid registration) and along the next
// 150-600 m of the PLANNED path (3x3 every 60 m), so the aircraft climbs
// before rising ground instead of after it.

#pragma once

#include <Arduino.h>
#include "flight_kinematics.h"

#define TERRAIN_FRESH_US 3000000   // 3 missed 1 Hz updates = terrain unavailable

// Where to look - TaskFlightControl fills it, TaskTerrain reads it (under navMutex).
struct NavSnapshot {
    bool   valid;          // position estimate fresh
    double lat, lon;       // dead-reckoned position, not the 1 Hz raw fix
    bool   mission;        // a mission is loaded
    float  path_s;         // m flown along the planned path
    float  ground_speed;   // m/s, sets the look-ahead distance
};

// The result - TaskTerrain fills it, TaskFlightControl reads it (under terrainMutex).
struct TerrainSnapshot {
    bool     valid;
    float    elev_m;       // m MSL, highest under the vehicle + along the look-ahead
    float    under_m;      // m MSL, highest under the vehicle only
    uint32_t lookup_ms;    // how long the SD lookups took
    int64_t  sample_us;
};

// From the flight code's state. Before arming the look-ahead starts at the
// route's beginning (the takeoff climb must clear the first leg).
NavSnapshot makeNavSnapshot(const raven::VehicleState &st, bool armed);

// DEM lookups for one terrain update. Takes sdMutex around each footprint
// read so the SD logger can interleave. SD reads take milliseconds: call
// from TaskTerrain, never from the control loop.
TerrainSnapshot lookupTerrain(const NavSnapshot &nav, SemaphoreHandle_t sdMutex);

// Boot check: walks the whole mission route through the DEM and requires
// every tile within 500 m of it on the card at full size. Prints the result.
// Call once in setup(), after the mission and terrain::begin(). Returns
// false if no mission is loaded or the route isn't covered.
bool checkMissionRoute();

// Terrain snapshot -> the flight code's terrain input (valid only if fresh).
void terrainToRaw(const TerrainSnapshot &terrain, int64_t now_us, raven::RawSensors &raw);
