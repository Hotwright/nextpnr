/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

// UltraScale+ PS8 boundary, on its AXI and ACE ports. There is no
// UltraScale+ timing model, and left to the placer the routes between those
// PS8 pins and the registers on the other side ran up to 74 pips: on the KV260 the PS and the logic each caught
// bits of the other's next value (0-ultrascale-bits kv260_ps_axi_test). This
// pins every flip-flop wired straight to a PS8 pin (its Q drives a PS8 input,
// or its D is a PS8 output) to the flip-flop nearest the INT row where that
// pin crosses into the fabric, so those routes are a few local hops in every
// build. It changes no logic: a design that wants the pins half a cycle from
// the PS's edge still writes its own falling-edge stage, and this places it.

#include <algorithm>
#include <boost/algorithm/string/predicate.hpp>
#include <map>

#include "extra_data.h"
#include "log.h"
#include "nextpnr.h"
#include "pack.h"

#define HIMBAECHEL_CONSTIDS "uarch/xilinx/constids.inc"
#include "himbaechel_constids.h"

NEXTPNR_NAMESPACE_BEGIN

namespace {
// The fabric side of a PS8 pin: the first INT tile its wire reaches, uphill
// for a pin the fabric drives, downhill for one the PS drives. The path is
// fixed (PS8 -> INT_INTF_LEFT_TERM_PSS -> INT); returns -1 if there is none.
int ps_pin_int_tile(Context *ctx, XilinxImpl *uarch, WireId pin_wire, bool uphill)
{
    std::vector<WireId> front{pin_wire};
    pool<WireId> seen{pin_wire};
    for (int depth = 0; depth < 4 && !front.empty(); depth++) {
        std::vector<WireId> next;
        for (WireId w : front) {
            if (w != pin_wire && boost::starts_with(uarch->tile_name(w.tile), "INT_X"))
                return w.tile;
            if (uphill) {
                for (PipId p : ctx->getPipsUphill(w))
                    if (seen.insert(ctx->getPipSrcWire(p)).second)
                        next.push_back(ctx->getPipSrcWire(p));
            } else {
                for (PipId p : ctx->getPipsDownhill(w))
                    if (seen.insert(ctx->getPipDstWire(p)).second)
                        next.push_back(ctx->getPipDstWire(p));
            }
        }
        front = std::move(next);
    }
    return -1;
}

// What the flip-flops in one half of a slice (A-D or E-H) must share, as
// XilinxImpl::assign_cell_tags reads it for the placer's validity check.
std::string ff_control_set(Context *ctx, const CellInfo *ci)
{
    auto net = [&](IdString port) {
        const NetInfo *n = ci->getPort(port);
        return n == nullptr ? std::string("-") : n->name.str(ctx);
    };
    bool clkinv = bool_or_default(ci->params, id_IS_CLK_INVERTED, false) ||
                  bool_or_default(ci->params, id_IS_C_INVERTED, false);
    bool srinv = bool_or_default(ci->params, id_IS_R_INVERTED, false) ||
                 bool_or_default(ci->params, id_IS_S_INVERTED, false) ||
                 bool_or_default(ci->params, id_IS_CLR_INVERTED, false) ||
                 bool_or_default(ci->params, id_IS_PRE_INVERTED, false);
    return stringf("%s|%s|%s|%d%d%d%d", net(id_CLK).c_str(), net(id_CE).c_str(), net(id_SR).c_str(), clkinv, srinv,
                   int(ci->attrs.count(id_X_FF_AS_LATCH)), int(ci->attrs.count(id_X_FFSYNC)));
}

// "X161Y140/SLICE_X0Y0.EFF2" -> "X161Y140/SLICE_X0Y0.1": the half its control
// set belongs to.
std::string ff_half(const std::string &bel_name)
{
    size_t dot = bel_name.rfind('.');
    char letter = bel_name.at(dot + 1);
    return bel_name.substr(0, dot) + (letter >= 'E' ? ".1" : ".0");
}
} // namespace

void USPacker::place_ps_boundary()
{
    CellInfo *ps = nullptr;
    for (auto &cell : ctx->cells)
        if (cell.second->type == id_PSS_ALTO_CORE)
            ps = cell.second.get();
    if (ps == nullptr || ps->bel == BelId())
        return;

    // Each flip-flop on a PS8 pin, with the INT tile its pin crosses.
    std::vector<std::pair<CellInfo *, Loc>> want;
    pool<IdString> taken_cells;
    int clustered = 0, no_row = 0, direct_pins = 0;
    for (auto &port : ps->ports) {
        NetInfo *net = port.second.net;
        if (net == nullptr || net->name.in(ctx->id("$PACKER_GND_NET"), ctx->id("$PACKER_VCC_NET")))
            continue;
        // Only the AXI and ACE ports, which the PS samples and drives on their
        // clock. Interrupts and the rest are not sampled on a fabric clock, and
        // pulling their registers to the PS can only lengthen their other
        // routes (PL_PS_IRQ0 enters ninety rows below HPM0).
        const std::string &pin = port.first.str(ctx);
        if (!boost::starts_with(pin, "MAXIGP") && !boost::starts_with(pin, "SAXIGP") &&
            !boost::starts_with(pin, "SAXIACP") && !boost::starts_with(pin, "SACEFPD"))
            continue;
        std::vector<CellInfo *> ffs;
        if (port.second.type == PORT_IN) {
            if (net->driver.cell != nullptr && net->driver.cell->type == id_SLICE_FFX && net->driver.port == id_Q)
                ffs.push_back(net->driver.cell);
        } else {
            for (auto &usr : net->users)
                if (usr.cell->type == id_SLICE_FFX && usr.port == id_D)
                    ffs.push_back(usr.cell);
        }
        if (ffs.empty())
            continue;
        direct_pins++;
        int tile = ps_pin_int_tile(ctx, uarch, ctx->getBelPinWire(ps->bel, port.first), port.second.type == PORT_IN);
        if (tile == -1) {
            no_row++;
            continue;
        }
        Loc row;
        tile_xy(ctx->chip_info, tile, row.x, row.y);
        for (CellInfo *ff : ffs) {
            if (ff->attrs.count(id_BEL) || ff->bel != BelId() || !taken_cells.insert(ff->name).second)
                continue;
            if (ff->cluster != ClusterId()) {
                clustered++; // packed with its LUT: the cluster is placed as a whole
                continue;
            }
            want.emplace_back(ff, row);
        }
    }
    if (want.empty()) {
        if (direct_pins == 0)
            log_info("PS8 pins: no flip-flop is wired straight to one; nothing to place beside the PS.\n");
        return;
    }

    // Flip-flop bels within reach, by column (nearest the INT column first)
    // and row. A design's own BEL attributes keep their bels.
    int x_int = want.front().second.x;
    std::map<int, std::map<int, std::vector<BelId>>> free_ffs;
    pool<std::string> pinned_bels;
    dict<std::string, std::string> half_set; // control set of each half in use
    for (auto &cell : ctx->cells) {
        if (!cell.second->attrs.count(id_BEL))
            continue;
        std::string bel = cell.second->attrs.at(id_BEL).as_string();
        pinned_bels.insert(bel);
        if (cell.second->type == id_SLICE_FFX)
            half_set[ff_half(bel)] = ff_control_set(ctx, cell.second.get());
    }
    for (BelId bel : ctx->getBels()) {
        if (ctx->getBelType(bel) != id_SLICE_FFX)
            continue;
        Loc l = ctx->getBelLocation(bel);
        if (l.x <= x_int || l.x > x_int + 12 || !ctx->checkBelAvail(bel) || pinned_bels.count(ctx->nameOfBel(bel)))
            continue;
        free_ffs[l.x][l.y].push_back(bel);
    }
    std::vector<int> cols;
    for (auto &c : free_ffs)
        cols.push_back(c.first);
    for (auto &c : free_ffs)
        for (auto &r : c.second)
            std::sort(r.second.begin(), r.second.end(), [&](BelId a, BelId b) {
                return ctx->getBelLocation(a).z < ctx->getBelLocation(b).z;
            });

    // Nearest free bel; a row away costs as much as a column away. Rows are
    // taken in order so each pin's neighbours fill before the next column.
    std::stable_sort(want.begin(), want.end(),
                     [](const std::pair<CellInfo *, Loc> &a, const std::pair<CellInfo *, Loc> &b) {
                         return a.second.y < b.second.y;
                     });
    int placed = 0, unplaced = 0;
    for (auto &w : want) {
        std::string cs = ff_control_set(ctx, w.first);
        BelId best;
        int best_cost = 0;
        for (int ci = 0; ci < int(cols.size()); ci++) {
            for (auto &r : free_ffs[cols[ci]]) {
                int cost = std::abs(r.first - w.second.y) + 2 * ci;
                if (best != BelId() && cost >= best_cost)
                    continue;
                for (BelId bel : r.second) {
                    auto fnd = half_set.find(ff_half(ctx->nameOfBel(bel)));
                    if (fnd == half_set.end() || fnd->second == cs) {
                        best = bel;
                        best_cost = cost;
                        break;
                    }
                }
            }
        }
        if (best == BelId() || best_cost > 16) {
            unplaced++;
            continue;
        }
        Loc l = ctx->getBelLocation(best);
        auto &row = free_ffs[l.x][l.y];
        row.erase(std::find(row.begin(), row.end(), best));
        std::string name = ctx->nameOfBel(best);
        half_set[ff_half(name)] = cs;
        w.first->attrs[id_BEL] = name;
        placed++;
    }
    log_info("PS8 pins: placed %d flip-flops beside the INT rows their pins cross%s.\n", placed,
             unplaced ? stringf(", %d left to the placer (no free flip-flop nearby)", unplaced).c_str() : "");
    if (clustered)
        log_info("PS8 pins: %d flip-flops are packed with a LUT and placed with it.\n", clustered);
    if (no_row)
        log_warning("PS8 pins: %d pins with no INT tile within reach.\n", no_row);
}

NEXTPNR_NAMESPACE_END
