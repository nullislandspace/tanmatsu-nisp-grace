#pragma once
// =====================================================================
//  Null Island Space Program  --  the ground, and the pad on it
// ---------------------------------------------------------------------
//  A sphere that turns. That is all a surface needs to be: a position, a
//  velocity, and a local frame (up / north / east) to read them against.
//
//  The pad is 0N 0E (Part O2). It is not a landmark, it is the origin the
//  launch starts from and the number the HUD's "range to pad" measures, so
//  it lives in code as a latitude and longitude of zero rather than as a
//  place in a world file.
//
//  Bob turns about +y, which makes a prograde orbit travel the same way the
//  ground does -- and makes the pad's east direction -z. Launch east and
//  174.9 m/s is free; launch west and it costs 349.8. That asymmetry is not
//  a rule written into the flight model, it is what the rotating sphere
//  already says.
//
//  No terrain. Bob is a sphere, so "altitude" is radius minus radius and a
//  landing is a radius test. Terrain is a future document, not a hole here.
// =====================================================================

#include <stdbool.h>

#include "sim/bodies.h"

// The pad. Latitude and longitude in radians.
#define NISP_PAD_LAT (0.0f)
#define NISP_PAD_LON (0.0f)

// A point fixed to the body's surface, in the inertial frame, kilometres.
void nisp_surface_point(int body, double t, float lat_rad, float lon_rad, double out_km[3]);

// The velocity of that point, km/s. On Bob at the equator this is 0.17493.
void nisp_surface_velocity(int body, double t, float lat_rad, float lon_rad, double out_kms[3]);

// Height above the surface, km. Negative means underground.
float nisp_altitude(int body, double const pos_km[3]);

// The local frame at a point: up is radial, east is the direction of the
// body's rotation, north completes the right-handed triple. All unit length.
void nisp_local_frame(int body, double const pos_km[3], double up[3], double north[3], double east[3]);

// Latitude / longitude of a point, radians. The HUD's "where am I".
void nisp_latlon(int body, double const pos_km[3], float* lat_rad, float* lon_rad);

// True when the point is under the surface -- the test a landing and a
// crash share.
bool nisp_below_surface(int body, double const pos_km[3]);
