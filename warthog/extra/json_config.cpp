#include "json_config.h"

void
to_json(nlohmann::json& j, const config& c)
{
    j = {
        {"hscale", c.hscale}, {"fscale", c.fscale}, {"time", c.time},
        {"itrs", c.itrs}, {"k_moves", c.k_moves}, {"threads", c.threads},
        {"verbose", c.verbose}, {"debug", c.debug},
        {"thread_alloc", c.thread_alloc}, {"no_cache", c.no_cache}
    };
}

void
from_json(const nlohmann::json& j, config& c)
{
    j.at("hscale").get_to(c.hscale);
    j.at("fscale").get_to(c.fscale);
    j.at("time").get_to(c.time);
    j.at("itrs").get_to(c.itrs);
    j.at("k_moves").get_to(c.k_moves);
    j.at("threads").get_to(c.threads);
    j.at("verbose").get_to(c.verbose);
    j.at("debug").get_to(c.debug);
    j.at("thread_alloc").get_to(c.thread_alloc);
    j.at("no_cache").get_to(c.no_cache);
}

// Configuration is passed around as a JSON object, we need to do the
// (de-)serialisation during the stream operation.
std::ostream&
operator<<(std::ostream& os, config& c)
{
    nlohmann::json j = c;

    os << j;

    return os;
}

std::istream&
operator>>(std::istream& is, config& c)
{
    nlohmann::json j;

    is >> j;
    c = j.get<config>(); // this will only read a complete object

    return is;
}

/**
 * Takes care of "default parameters" as we use a bunch of wildcards to
 * represent different unbounded values.
 *
 * TODO Should this be part of cpd search directly?
 */
void
sanitise_conf(config& conf)
{
    conf.fscale = std::max(0.0, conf.fscale);
    conf.hscale = std::max(1.0, conf.hscale);

    if (conf.itrs == 0)
    { conf.itrs = warthog::INF32; }

    if (conf.k_moves == 0)
    { conf.k_moves = warthog::INF32; }

    if (conf.time == 0)
    { conf.time = DBL_MAX; }

    // Enforce single threaded or use max threads
#ifdef SINGLE_THREADED
    conf.threads = 1;
#else
    if (conf.threads == 0)
    {
        conf.threads = omp_get_max_threads();
    }
#endif

}
