/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2019-23  Myrtle Shah <gatecat@ds0.me>
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

// UltraScale+ clock packing, ported from nextpnr-xilinx's
// pack_clocking_xcup.cc.

#include <boost/algorithm/string.hpp>
#include "log.h"
#include "nextpnr.h"
#include "pack.h"

#define HIMBAECHEL_CONSTIDS "uarch/xilinx/constids.inc"
#include "himbaechel_constids.h"

NEXTPNR_NAMESPACE_BEGIN

void USPacker::prepare_clocking()
{
    log_info("Preparing clocking...\n");
    dict<IdString, IdString> upgrade;
    upgrade[id_MMCME2_ADV] = id_MMCME4_ADV;
    upgrade[id_MMCME4_BASIC] = id_MMCME4_ADV;
    upgrade[id_PLLE4_BASIC] = id_PLLE4_ADV;
    upgrade[id_BUFG] = id_BUFGCE;

    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (upgrade.count(ci->type))
            ci->type = upgrade.at(ci->type);
    }
}

void USPacker::pack_plls()
{
    log_info("Packing PLLs...\n");

    auto set_default = [](CellInfo *ci, IdString param, const Property &value) {
        if (!ci->params.count(param))
            ci->params[param] = value;
    };

    dict<IdString, XFormRule> pll_rules;
    pll_rules[id_MMCME4_ADV].new_type = id_MMCM_MMCM_TOP;
    pll_rules[id_PLLE4_ADV].new_type = id_PLL_PLL_TOP;
    generic_xform(pll_rules);
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type == id_MMCM_MMCM_TOP) {
            for (int i = 1; i <= 2; i++)
                set_default(ci, ctx->idf("CLKIN%d_PERIOD", i), Property("0.0"));
            for (int i = 0; i <= 6; i++) {
                set_default(ci, ctx->idf("CLKOUT%d_CASCADE", i), Property("FALSE"));
                set_default(ci, ctx->idf("CLKOUT%d_DIVIDE", i), Property(1));
                set_default(ci, ctx->idf("CLKOUT%d_DUTY_CYCLE", i), Property("0.5"));
                set_default(ci, ctx->idf("CLKOUT%d_PHASE", i), Property(0));
                set_default(ci, ctx->idf("CLKOUT%d_USE_FINE_PS", i), Property("FALSE"));
            }
        }
        if (ci->type.in(id_MMCM_MMCM_TOP, id_PLL_PLL_TOP)) {
            // Feedback stays inside the CMT, as Vivado does it when nothing
            // else is on the feedback net: CLKFBIN tied (kv260_cmt_char).
            set_default(ci, id_COMPENSATION, Property("INTERNAL"));
            if (str_or_default(ci->params, id_COMPENSATION, "INTERNAL") == "INTERNAL") {
                ci->disconnectPort(id_CLKFBIN);
                ci->connectPort(id_CLKFBIN, ctx->nets.at(ctx->id("$PACKER_VCC_NET")).get());
            }
        }
    }
}

void USPacker::pack_gbs()
{
    log_info("Packing global buffers...\n");
    dict<IdString, XFormRule> gb_rules;
    gb_rules[id_BUFG_PS].new_type = id_BUFCE_BUFG_PS;
    gb_rules[id_BUFGCE].new_type = id_BUFCE_BUFCE;
    gb_rules[id_BUFGCTRL].new_type = id_BUFGCTRL;
    gb_rules[id_BUFGCE_DIV].new_type = id_BUFGCE_DIV_BUFGCE_DIV;

    // Make sure prerequisites are set up first
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type == id_PSS_ALTO_CORE)
            preplace_unique(ci);
        if (ci->type.in(id_PSEUDO_GND, id_PSEUDO_VCC))
            preplace_unique(ci);
    }

    generic_xform(gb_rules);
}

void USPacker::preplace_clocking()
{
    bool did_something = false;
    do {
        did_something = false;
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            if (ci->bel != BelId())
                continue;
            if (ci->type.in(id_BUFCE_BUFG_PS, id_BUFCE_BUFCE, id_BUFGCE_DIV_BUFGCE_DIV))
                did_something |= try_preplace(ci, id_I);
            else if (ci->type == id_BUFGCTRL)
                did_something |= try_preplace(ci, id_I0);
            else if (ci->type.in(id_MMCM_MMCM_TOP, id_PLL_PLL_TOP))
                did_something |= try_preplace(ci, ci->type == id_PLL_PLL_TOP ? id_CLKIN : id_CLKIN1);
        }
    } while (did_something);
}

// A BUFGCE fed from fabric has no placed driver to preplace it by, and the
// placer would take any BUFCE bel -- the HDIO banks' BUFGCE_HDIO included,
// whose configuration is not characterised. Bind it to a CMT_L BUFGCE
// instead. BUFGCE_X0Y<24r+t> drives only HROUTE/HDISTR track t, so every
// global buffer gets its own track (a BUFG_PS_X0Y<24r+k> takes track k), as
// Vivado requires (0-ultrascale-bits/designs/kv260_cmt_char/gen.py).
void USPacker::place_fabric_bufgce()
{
    auto site_y = [&](BelId bel) {
        std::string name = ctx->getBelName(bel)[1].str(ctx);
        auto y = name.find('Y'), dot = name.find('.');
        return std::stoi(name.substr(y + 1, dot - y - 1));
    };
    std::vector<bool> track_used(24, false);
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type.in(id_BUFCE_BUFG_PS, id_BUFCE_BUFCE) && ci->bel != BelId())
            track_used.at(site_y(ci->bel) % 24) = true;
    }
    std::vector<BelId> cmt_bufgce;
    for (BelId bel : ctx->getBels())
        if (ctx->getBelType(bel) == id_BUFCE_BUFCE && boost::starts_with(uarch->tile_name(bel.tile), "CMT_L"))
            cmt_bufgce.push_back(bel);
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type != id_BUFCE_BUFCE || ci->bel != BelId() || ci->attrs.count(id_BEL))
            continue;
        BelId found;
        for (BelId bel : cmt_bufgce) {
            if (!track_used.at(site_y(bel) % 24) && ctx->checkBelAvail(bel)) {
                found = bel;
                break;
            }
        }
        if (found == BelId())
            log_error("No free clock track left for BUFGCE '%s'\n", ctx->nameOf(ci));
        track_used.at(site_y(found) % 24) = true;
        ctx->bindBel(found, ci, STRENGTH_LOCKED);
        log_info("    Constrained BUFGCE '%s' to bel '%s' (clock track %d)\n", ctx->nameOf(ci), ctx->nameOfBel(found),
                 site_y(found) % 24);
    }
}

void USPacker::pack_clocking()
{
    pack_plls();
    pack_gbs();
    preplace_clocking();
    place_fabric_bufgce();
}

NEXTPNR_NAMESPACE_END
