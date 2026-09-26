#include "terrain_following.h"
#include <esp_timer.h>
#include "fcode_interpreter.h"
#include "terrain.h"

static const int   UNDER_RADIUS     = 2;        // 5x5 samples under the vehicle
static const int   AHEAD_RADIUS     = 1;        // 3x3 samples on the path ahead
static const float LOOKAHEAD_TIME_S = 15.0f;    // look-ahead = ground speed * this,
static const float LOOKAHEAD_MIN_M  = 150.0f;   // clamped to [min, max]
static const float LOOKAHEAD_MAX_M  = 600.0f;
static const float LOOKAHEAD_STEP_M = 60.0f;    // 3x3 footprints every 60 m are contiguous
static const float ROUTE_MARGIN_M   = 500.0f;   // drift room off the path (boot check)

// Planned-path position at along-path distance s.
static bool pathPoint(float s, double &lat, double &lon) {
    float x, y;
    if (!fcode::pathPointAt(s, x, y)) return false;
    fcode::localToGps(x, y, lat, lon);
    return true;
}

static float lookupLocked(double lat, double lon, int radius, SemaphoreHandle_t sdMutex) {
    float e = NAN;
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        e = terrain::lookupMaxElevation(lat, lon, radius);
        xSemaphoreGive(sdMutex);
    }
    return e;
}

NavSnapshot makeNavSnapshot(const raven::VehicleState &st, bool armed) {
    NavSnapshot nav = {};
    nav.valid = st.nav_valid;
    if (st.nav_valid) fcode::localToGps(st.pos_x, st.pos_y, nav.lat, nav.lon);
    nav.mission      = fcode::loaded();
    nav.ground_speed = st.ground_speed;
    if (nav.mission && (armed || fcode::complete())) {
        const float total = fcode::info().total_length_m;
        nav.path_s = constrain(total - st.mission_distance_to_go, 0.0f, total);
    }
    return nav;
}

TerrainSnapshot lookupTerrain(const NavSnapshot &nav, SemaphoreHandle_t sdMutex) {
    TerrainSnapshot out = {};
    if (!nav.valid) return out;
    const uint32_t t0 = millis();

    float best = lookupLocked(nav.lat, nav.lon, UNDER_RADIUS, sdMutex);
    out.under_m = best;
    if (!isnan(best) && nav.mission) {
        const float total = fcode::info().total_length_m;
        const float ahead = constrain(nav.ground_speed * LOOKAHEAD_TIME_S, LOOKAHEAD_MIN_M, LOOKAHEAD_MAX_M);
        for (float d = LOOKAHEAD_STEP_M; d <= ahead + 1.0f; d += LOOKAHEAD_STEP_M) {
            const float s = fminf(nav.path_s + d, total);
            double lat, lon;
            const float e = pathPoint(s, lat, lon) ? lookupLocked(lat, lon, AHEAD_RADIUS, sdMutex) : NAN;
            if (isnan(e)) { best = NAN; break; }
            if (e > best) best = e;
            if (s >= total) break;
        }
    }
    out.valid     = !isnan(best);
    out.elev_m    = best;
    out.lookup_ms = millis() - t0;
    out.sample_us = esp_timer_get_time();
    return out;
}

bool checkMissionRoute() {
    if (!fcode::loaded()) return false;
    const uint32_t t0 = millis();
    const terrain::RouteCheck rc = terrain::checkRoute(fcode::info().total_length_m, pathPoint,
                                                       LOOKAHEAD_STEP_M, ROUTE_MARGIN_M, AHEAD_RADIUS);
    if (rc.ok) {
        Serial.printf("Terrain route check: OK, %d points, %d tile(s) within %.0f m of the route, "
                      "highest terrain %.0f m -> up to %.0f m MSL (%lu ms)\n",
            rc.points, rc.tiles, ROUTE_MARGIN_M, rc.max_elev_m,
            rc.max_elev_m + fcode::info().ground_clearance_m, (unsigned long)(millis() - t0));
    } else {
        Serial.printf("Terrain route check: FAILED at %d of %d points (first at %.0f m along the route): "
                      "DEM missing, truncated, or a needed tile within %.0f m of the route is not on the card "
                      "- mission cannot be armed\n",
            rc.missing, rc.points, rc.first_missing_m, ROUTE_MARGIN_M);
    }
    return rc.ok;
}

void terrainToRaw(const TerrainSnapshot &terrain, int64_t now_us, raven::RawSensors &raw) {
    raw.terrain_valid  = terrain.valid && (now_us - terrain.sample_us) < TERRAIN_FRESH_US;
    raw.terrain_elev_m = terrain.elev_m;
}
