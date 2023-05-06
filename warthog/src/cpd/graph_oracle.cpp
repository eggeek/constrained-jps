#include "cpd.h"
#include "graph_oracle.h"
#include "helpers.h"

// When we store a first move table we compress 8 (4 bit) firstmoves into one
// `rle_run32`.
//
// TODO Replace with an actual datatype?
void
pack_row(std::vector<warthog::cpd::fm_coll>& row,
         std::vector<uint32_t>& order,
         std::vector<warthog::cpd::rle_run32>& fm)
{
    // We pack 8 first moves in the 32 bits field.
    for(uint32_t index = 0; index < row.size(); index += 8)
    {
        uint32_t moveset = 0x0;
        uint32_t entry_size = std::min<uint32_t>(row.size() - index, 8);

        for(uint8_t entry = 0; entry < entry_size; entry++)
        {
            uint8_t firstmove = __builtin_ffsl(
                row.at(order.at(index + entry))) - 1;
            assert(firstmove < warthog::cpd::CPD_FM_MAX);
            moveset |= firstmove << (entry * 4);
        }

        fm.push_back({ moveset });
    }
}

template<>
void
warthog::cpd::graph_oracle_base<warthog::cpd::REV_TABLE>::add_row(
    uint32_t target_id, std::vector<warthog::cpd::fm_coll>& row)
{
    // target gets a wildcard move
    row.at(target_id) = warthog::cpd::CPD_FM_NONE;

    pack_row(row, order_, fm_.at(target_id));
}

template<>
void
warthog::cpd::graph_oracle_base<warthog::cpd::TABLE>::add_row(
    uint32_t source_id, std::vector<warthog::cpd::fm_coll>& row)
{
    // source gets a wildcard move
    row.at(source_id) = warthog::cpd::CPD_FM_NONE;

    pack_row(row, order_, fm_.at(source_id));
}
