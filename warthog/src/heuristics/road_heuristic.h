#ifndef __ROAD_HEURISTIC_H_
#define __ROAD_HEURISTIC_H_

// Heuristic(s) for measuring distance in road networks -- i.e., on some
// representation of the Earth.
#include <cassert>

#include "constants.h"
#include "geography.h"
#include "xy_graph.h"

namespace warthog
{

template<warthog::geo::distance D>
class road_heuristic_base
{
  public:
    explicit road_heuristic_base(warthog::graph::xy_graph* g) : g_(g) { };
    ~road_heuristic_base() = default;

    road_heuristic_base(road_heuristic_base&) = delete;
    road_heuristic_base&
    operator=(const road_heuristic_base&) = delete;

    road_heuristic_base(road_heuristic_base&&) noexcept = default;
    road_heuristic_base&
    operator=(road_heuristic_base&&) noexcept = default;

    void
    set_hscale(double hscale)
    {
        assert(hscale > 0);
        hscale_ = hscale;
    }

    double
    get_hscale()
    { return hscale_; }

    size_t
    mem()
    { return sizeof(*this); }

    warthog::cost_t
    h(warthog::sn_id_t start_id, warthog::sn_id_t target_id)
    {
        int32_t xs;
        int32_t ys;
        int32_t xt;
        int32_t yt;
        g_->get_xy(start_id, xs, ys);
        g_->get_xy(target_id, xt, yt);

        return distance_(
            xs / warthog::geo::DIMACS_RATIO, ys / warthog::geo::DIMACS_RATIO,
            xt / warthog::geo::DIMACS_RATIO, yt / warthog::geo::DIMACS_RATIO);
    }

  private:
    warthog::graph::xy_graph* g_;
    double hscale_ = 1.0;

    warthog::cost_t
    distance_(double lat_s, double lon_s, double lat_t, double lon_t);
};

}

#endif // __ROAD_HEURISTIC_H_
