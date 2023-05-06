// Module to handle geographic coordinates (lat/long)
//
// The '*_xy' variants are meant to convert coordinates in usual `xy_graph`
// format to floats using the DIMACS ratio.
#ifndef __GEOGRAPHY_H_
#define __GEOGRAPHY_H_

#include <cstdint>

namespace warthog
{

namespace geo
{

static const double DIMACS_RATIO = 1e6;
static const double EARTH_RADIUS = 6371.0009; // km

// Distance functions ordered by precision
enum distance
{
    SPHERICAL,
    GREAT_CIRCLE,
    VINCENTY
};

// Compute distance using a Spherical Earth projected to a plane
double
spherical_distance(double lat_a, double lng_a, double lat_b, double lng_b);

// Compute the distance between two points on a sphere.
double
great_circle_distance(double lat_a, double lng_a, double lat_b, double lng_b);

// Compute the distance between two points using (a special case of) the
// Vincenty formula.
double
vincenty_distance(double lat_a, double lng_a, double lat_b, double lng_b);

// Get the bearing from pos1 to pos2
double
get_bearing(double lat_a, double lng_a, double lat_b, double lng_b);

// Get the bearing of a location wrt North
double
true_bearing(double lat, double lng);

// Get angle between (ABC) given the lat/long of the three points
double
get_angle(
    double lat_a, double lng_a, double lat_b, double lng_b, double lat_c,
    double lng_c);

// Check whether, for three vectors rooted at the same point s, (sb) is between
// (sa) and (sc).
bool
between(
    double lat_a, double lng_a, double lat_b, double lng_b, double lat_c,
    double lng_c);

// Check whether vector (sb) is between (sa) and (sc)
bool
between(
    double lat_s, double lng_s, double lat_a, double lng_a, double lat_b,
    double lng_b, double lat_c, double lng_c);

double
get_bearing_xy(uint32_t lat1, uint32_t lng1, uint32_t lat2, uint32_t lng2);

double
true_bearing_xy(uint32_t lng, uint32_t lat);

double
get_angle_xy(
    uint32_t lat_a, uint32_t lng_a, uint32_t lat_b, uint32_t lng_b,
    uint32_t lat_c, uint32_t lng_c);

bool
between_xy(
    uint32_t lat_a, uint32_t lng_a, uint32_t lat_b, uint32_t lng_b,
    uint32_t lat_c, uint32_t lng_c);

bool
between_xy(
    uint32_t lat_s, uint32_t lng_s, uint32_t lat_a, uint32_t lng_a,
    uint32_t lat_b, uint32_t lng_b, uint32_t lat_c, uint32_t lng_c);

}
}

#endif // __GEOGRAPHY_H_
