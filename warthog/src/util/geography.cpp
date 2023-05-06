#include "geography.h"
#include "constants.h"
#include <cmath>

double
deg_to_rad(double deg)
{
    return deg * M_PI / 180;
}

double
rad_to_deg(double rad)
{
    return fmod(rad * 180 / M_PI + 360, 360);
}

// Project the Earth onto a plane and into account the variation between
// meridians with latitude.
//
// https://en.wikipedia.org/wiki/Geographical_distance
double
warthog::geo::spherical_distance(
    double lat_a, double lng_a, double lat_b, double lng_b)
{
    double p1 = deg_to_rad(lat_a);
    double l1 = deg_to_rad(lng_a);
    double p2 = deg_to_rad(lat_b);
    double l2 = deg_to_rad(lng_b);

    // (\Delta \phi) ^ 2
    double D_p_2 = pow(fabs(p1 - p2), 2);

    double cos_pm = cos((p1 + p2) / 2);
    double x = cos_pm * fabs(l1 - l2);

    return warthog::geo::EARTH_RADIUS * sqrt(D_p_2 + pow(x, 2));
}

// Compute the distance between two points on a sphere by using the Haversine
// formula. The Great-Circle distance is only accurate up to 0.5% and is less
// so for short distances.
//
// https://en.wikipedia.org/wiki/Great-circle_distance
double
warthog::geo::great_circle_distance(
    double lat_a, double lng_a, double lat_b, double lng_b)
{
    double p1 = deg_to_rad(lat_a);
    double l1 = deg_to_rad(lng_a);
    double p2 = deg_to_rad(lat_b);
    double l2 = deg_to_rad(lng_b);

    double D_l = fabs(l1 - l2);
    double D_p = fabs(p1 - p2);

    double hav_p = pow(sin(D_p / 2), 2);
    double hav_l = pow(sin(D_l / 2), 2);
    double cos_p = cos(p1) * cos(p2);

    double sigma = 2 * asin(sqrt(hav_p + cos_p * hav_l));

    return warthog::geo::EARTH_RADIUS * sigma;
}

// The Vincenty formula is an iterative procedure which can be accurate up to
// 0.5 mm. We use the special case where the ellipsoid is a sphere.
//
// https://en.wikipedia.org/wiki/Vincenty%27s_formulae
double
warthog::geo::vincenty_distance(
    double lat_a, double lng_a, double lat_b, double lng_b)
{
    double p1 = deg_to_rad(lat_a);
    double l1 = deg_to_rad(lng_a);
    double p2 = deg_to_rad(lat_b);
    double l2 = deg_to_rad(lng_b);

    double D_l = fabs(l1 - l2);

    double cos_p1 = cos(p1);
    double cos_p2 = cos(p2);
    double sin_p1 = sin(p1);
    double sin_p2 = sin(p2);

    double x_a = cos_p2 * sin(D_l);
    double x_b = cos_p1 * sin_p2 - sin_p1 * cos_p2 * cos(D_l);
    double num = pow(x_a, 2) + pow(x_b, 2);

    double den = sin_p1 * sin_p2 + cos_p1 * cos_p2 * cos(D_l);

    return warthog::geo::EARTH_RADIUS * atan(sqrt(num) / den);
}

// We calculate bearing with the following formula:
// θ = atan2( sin Δλ ⋅ cos φ2 , cos φ1 ⋅ sin φ2 − sin φ1 ⋅ cos φ2 ⋅ cos Δλ )
// where:
//   φ1,λ1 is the start point,
//   φ2,λ2 the end point
//   Δλ is the difference in longitude
double
warthog::geo::get_bearing(
    double lat_a, double lng_a, double lat_b, double lng_b)
{
    double l1 = deg_to_rad(lng_a);
    double l2 = deg_to_rad(lng_b);
    double p1 = deg_to_rad(lat_a);
    double p2 = deg_to_rad(lat_b);
    double y = sin(l2 - l1) * cos(p2);
    double x = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(l2 - l1);

    return rad_to_deg(std::atan2(y, x));
}

// We use the magnetic North (86°26′52.8″N 175°20′45.06″E) as to avoid angles
// being 180° for all points.
double
warthog::geo::true_bearing(double lat, double lng)
{
    return warthog::geo::get_bearing(
        deg_to_rad(86.448), deg_to_rad(175.5968), deg_to_rad(lng),
        deg_to_rad(lat));
}

// The angle (ABC) is defined as the bearing from C to A when B is the origin.
double
warthog::geo::get_angle(
    double lat_a, double lng_a, double lat_b, double lng_b, double lat_c,
    double lng_c)
{
    return warthog::geo::get_bearing(
        lat_a - lat_b, lng_a - lng_b, lat_c - lat_b, lng_c - lng_b);
}

// This works by checking that the longitude of A and C differ in sign wrt B.
bool
warthog::geo::between(
    double lat_a, double lng_a, double lat_b, double lng_b, double lat_c,
    double lng_c)
{
    return (lng_a - lng_b) * (lng_c - lng_b) < 0;
}

// Variant of 'between' where we pass the origin.
bool
warthog::geo::between(
    double lat_o, double lng_o, double lat_a, double lng_a, double lat_b,
    double lng_b, double lat_c, double lng_c)
{
    double lng_sa = lng_o - lng_a;
    double lng_sb = lng_o - lng_b;
    double lng_sc = lng_o - lng_c;
    // TODO Offset latitudes too?
    return warthog::geo::between(lat_a, lng_sa, lat_b, lng_sb, lat_c, lng_sc);
}

double
warthog::geo::get_bearing_xy(
    uint32_t lat1, uint32_t lng1, uint32_t lat2, uint32_t lng2)
{
    return warthog::geo::get_bearing(
        lat1 / warthog::geo::DIMACS_RATIO, lng1 / warthog::geo::DIMACS_RATIO,
        lat2 / warthog::geo::DIMACS_RATIO, lng2 / warthog::geo::DIMACS_RATIO);
}

double
warthog::geo::true_bearing_xy(uint32_t lng, uint32_t lat)
{
    return warthog::geo::true_bearing(
        lat / warthog::geo::DIMACS_RATIO, lng / warthog::geo::DIMACS_RATIO);
}

double
warthog::geo::get_angle_xy(
    uint32_t lat_a, uint32_t lng_a, uint32_t lat_b, uint32_t lng_b,
    uint32_t lat_c, uint32_t lng_c)
{
    return warthog::geo::get_angle(
        lat_a / warthog::geo::DIMACS_RATIO, lng_a / warthog::geo::DIMACS_RATIO,
        lat_b / warthog::geo::DIMACS_RATIO, lng_b / warthog::geo::DIMACS_RATIO,
        lat_c / warthog::geo::DIMACS_RATIO, lng_c / warthog::geo::DIMACS_RATIO);
}

bool
warthog::geo::between_xy(
    uint32_t lat_a, uint32_t lng_a, uint32_t lat_b, uint32_t lng_b,
    uint32_t lat_c, uint32_t lng_c)
{
    return warthog::geo::between(
        lat_a / warthog::geo::DIMACS_RATIO, lng_a / warthog::geo::DIMACS_RATIO,
        lat_b / warthog::geo::DIMACS_RATIO, lng_b / warthog::geo::DIMACS_RATIO,
        lat_c / warthog::geo::DIMACS_RATIO, lng_c / warthog::geo::DIMACS_RATIO);
}

bool
warthog::geo::between_xy(
    uint32_t lat_s, uint32_t lng_s, uint32_t lat_a, uint32_t lng_a,
    uint32_t lat_b, uint32_t lng_b, uint32_t lat_c, uint32_t lng_c)
{
    return warthog::geo::between(
        lat_s / warthog::geo::DIMACS_RATIO, lng_s / warthog::geo::DIMACS_RATIO,
        lat_a / warthog::geo::DIMACS_RATIO, lng_a / warthog::geo::DIMACS_RATIO,
        lat_b / warthog::geo::DIMACS_RATIO, lng_b / warthog::geo::DIMACS_RATIO,
        lat_c / warthog::geo::DIMACS_RATIO, lng_c / warthog::geo::DIMACS_RATIO);
}
