#include "euclidean_heuristic.h"
#include "xy_graph.h"

warthog::euclidean_heuristic::euclidean_heuristic(warthog::graph::xy_graph* g) 
{ 
    g_ = g;
    hscale_ = 1; 
}

warthog::euclidean_heuristic::~euclidean_heuristic() 
{ }

double
warthog::euclidean_heuristic::h(warthog::sn_id_t id, warthog::sn_id_t id2)
{
    int32_t x, x2;
    int32_t y, y2;
    g_->get_xy((uint32_t)id, x, y);
    g_->get_xy((uint32_t)id2, x2, y2);
    return warthog::euclidean_heuristic::h(x, y, x2, y2) * hscale_;
}

double
warthog::euclidean_heuristic::h(double x, double y, double x2, double y2)
{
    // NB: precision loss when warthog::cost_t is an integer
    double dx = x-x2;
    double dy = y-y2;
    return sqrt(dx*dx + dy*dy);
}

void
warthog::euclidean_heuristic::set_hscale(double hscale)
{
    if(hscale > 0)
    {
        hscale_ = hscale;
    }
}

double
warthog::euclidean_heuristic::get_hscale() 
{ 
    return hscale_; 
}

size_t
warthog::euclidean_heuristic::mem() 
{ 
    return sizeof(this); 
}

