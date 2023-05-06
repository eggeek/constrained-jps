#include "sipp/jpst_gridmap.h"
#include <algorithm>

warthog::jpst_gridmap::jpst_gridmap(warthog::gridmap* gm)
{
    gm_ = gm;
    sipp_map_ = new warthog::sipp_gridmap(gm);

    // one copy of the map for jumping E<->W; one copy for jumping N<->S
    t_gm_ = new warthog::gridmap(gm_->header_height(), gm_->header_width());
}

warthog::jpst_gridmap::~jpst_gridmap()
{
    delete t_gm_;
    delete sipp_map_;
}
