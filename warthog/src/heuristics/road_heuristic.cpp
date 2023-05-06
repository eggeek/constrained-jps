#include "road_heuristic.h"

template<>
warthog::cost_t
warthog::road_heuristic_base<warthog::geo::SPHERICAL>::distance_(
    double lat_s, double lon_s, double lat_t, double lon_t)
{
    return warthog::geo::spherical_distance(lat_s, lon_s, lat_t, lon_t);
}

template<>
warthog::cost_t
warthog::road_heuristic_base<warthog::geo::GREAT_CIRCLE>::distance_(
    double lat_s, double lon_s, double lat_t, double lon_t)
{
    return warthog::geo::great_circle_distance(lat_s, lon_s, lat_t, lon_t);
}

template<>
warthog::cost_t
warthog::road_heuristic_base<warthog::geo::VINCENTY>::distance_(
    double lat_s, double lon_s, double lat_t, double lon_t)
{
    return warthog::geo::vincenty_distance(lat_s, lon_s, lat_t, lon_t);
}
