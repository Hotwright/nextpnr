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
    NetInfo *gnd = ctx->nets.at(ctx->id("$PACKER_GND_NET")).get();
    NetInfo *vcc = ctx->nets.at(ctx->id("$PACKER_VCC_NET")).get();
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
        if (!ci->type.in(id_MMCM_MMCM_TOP, id_PLL_PLL_TOP))
            continue;
        bool mmcm = ci->type == id_MMCM_MMCM_TOP;
        auto refuse = [&](bool cond, const std::string &what) {
            if (cond)
                log_error("%s '%s': %s is not supported on UltraScale+ yet\n",
                          ci->attrs.at(id_X_ORIG_TYPE).as_string().c_str(), ctx->nameOf(ci), what.c_str());
        };
        auto used = [&](const std::string &port) {
            NetInfo *n = ci->getPort(ctx->id(port));
            return n != nullptr && n->users.entries() > 0;
        };
        // What kv260_mmcm_char characterised: internal feedback, no spread
        // spectrum or fine phase shift, true outputs only.
        std::string comp = str_or_default(ci->params, id_COMPENSATION, "AUTO");
        refuse(comp != "AUTO" && comp != "INTERNAL", "COMPENSATION " + comp);
        for (auto out : mmcm ? std::vector<std::string>{"CLKOUT0B", "CLKOUT1B", "CLKOUT2B", "CLKOUT3B", "CLKFBOUTB",
                                                        "CLKFBSTOPPED", "CLKINSTOPPED", "PSDONE", "CDDCDONE"}
                             : std::vector<std::string>{"CLKOUT0B", "CLKOUT1B", "CLKOUTPHY"})
            refuse(used(out), "output " + out);
        if (mmcm) {
            refuse(str_or_default(ci->params, ctx->id("SS_EN"), "FALSE") == "TRUE", "SS_EN");
            for (int i = 0; i <= 7; i++) {
                std::string p = i == 7 ? "CLKFBOUT_USE_FINE_PS" : stringf("CLKOUT%d_USE_FINE_PS", i);
                refuse(str_or_default(ci->params, ctx->id(p), "FALSE") == "TRUE", p);
            }
        }
        IdString mult_id = ctx->id(mmcm ? "CLKFBOUT_MULT_F" : "CLKFBOUT_MULT");
        double mult = 5.0;
        if (ci->params.count(mult_id)) {
            const Property &p = ci->params.at(mult_id);
            mult = p.is_string ? std::stod(p.as_string()) : double(p.as_int64());
        }
        refuse(mmcm ? (mult < 2 || mult > 128) : (mult < 1 || mult > 19), stringf("CLKFBOUT_MULT %g", mult));

        // Feedback stays inside the CMT, as Vivado does it when CLKFBOUT
        // drives CLKFBIN and nothing else: both ports go, and the FASM
        // writer ties CLKFBIN (kv260_mmcm_char: every specimen).
        NetInfo *fb = ci->getPort(id_CLKFBIN);
        refuse(fb == nullptr || fb != ci->getPort(ctx->id("CLKFBOUT")) || fb->users.entries() != 1,
               "feedback other than CLKFBOUT -> CLKFBIN alone");
        ci->disconnectPort(id_CLKFBIN);
        ci->disconnectPort(ctx->id("CLKFBOUT"));
        if (mmcm) {
            // The second input is characterised only unused (the writer
            // ties it).
            NetInfo *in2 = ci->getPort(id_CLKIN2);
            refuse(in2 != nullptr && in2 != gnd && in2 != vcc, "CLKIN2 in use");
            ci->disconnectPort(id_CLKIN2);
        }
        // Vivado leaves these unrouted when tied and sets the value by the
        // pin's ZINV / IS_*_INVERTED bit; the writer does the same from
        // X_TIE_<pin> (kv260_mmcm_test reference). A RST from logic is
        // routed; the others are characterised tied only.
        for (auto pin : mmcm ? std::vector<IdString>{id_RST, id_PWRDWN, id_PSEN, id_PSINCDEC}
                             : std::vector<IdString>{id_RST, id_PWRDWN}) {
            NetInfo *n = ci->getPort(pin);
            if (pin == id_RST && n != gnd && n != vcc)
                continue;
            refuse(n != gnd && n != vcc, pin.str(ctx) + " driven by logic");
            ci->params[ctx->id("X_TIE_" + pin.str(ctx))] = Property(n == vcc ? 1 : 0, 1);
            ci->disconnectPort(pin);
        }
    }
}

// An MMCM/PLL output reaches the clock network only through a BUFGCE of its
// own CMT (CLK_BUFGCE_<n>_CLK_IN <- CMT_*_CLKOUTn, kv260_mmcm_char
// bufg_enc.py), so the MMCM/PLL is bound here -- at its LOC, or the first
// free one -- and each BUFGCE it drives next to it, on a clock track of its
// own. Track 0 is left alone: its BUFGCE_X0Y0 select bits are inferred, not
// observed.
void USPacker::place_cmt()
{
    auto site_y = [&](BelId bel) {
        std::string name = ctx->getBelName(bel)[1].str(ctx);
        auto y = name.find('Y'), dot = name.find('.');
        return std::stoi(name.substr(y + 1, dot - y - 1));
    };
    dict<IdString, BelId> by_site;
    std::vector<BelId> cmt_bels, bufgce_bels;
    for (BelId bel : ctx->getBels()) {
        if (!boost::starts_with(uarch->tile_name(bel.tile), "CMT_L"))
            continue;
        IdString t = ctx->getBelType(bel);
        if (t.in(id_MMCM_MMCM_TOP, id_PLL_PLL_TOP)) {
            cmt_bels.push_back(bel);
            by_site[uarch->get_site_name(uarch->get_bel_site(bel))] = bel;
        } else if (t == id_BUFCE_BUFCE) {
            bufgce_bels.push_back(bel);
        }
    }
    std::vector<bool> track_used(24, false);
    track_used.at(0) = true;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type.in(id_BUFCE_BUFG_PS, id_BUFCE_BUFCE) && ci->bel != BelId())
            track_used.at(site_y(ci->bel) % 24) = true;
    }
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (!ci->type.in(id_MMCM_MMCM_TOP, id_PLL_PLL_TOP) || ci->bel != BelId())
            continue;
        BelId bel;
        if (ci->attrs.count(id_LOC)) {
            std::string loc = ci->attrs.at(id_LOC).as_string();
            auto found = by_site.find(ctx->id(loc));
            if (found == by_site.end() || ctx->getBelType(found->second) != ci->type)
                log_error("Can't place '%s' at LOC %s\n", ctx->nameOf(ci), loc.c_str());
            bel = found->second;
        } else {
            for (BelId b : cmt_bels)
                if (ctx->getBelType(b) == ci->type && ctx->checkBelAvail(b)) {
                    bel = b;
                    break;
                }
        }
        if (bel == BelId() || !ctx->checkBelAvail(bel))
            log_error("No free %s site for '%s'\n", ci->type == id_MMCM_MMCM_TOP ? "MMCM" : "PLL", ctx->nameOf(ci));
        ctx->bindBel(bel, ci, STRENGTH_LOCKED);
        log_info("    Constrained '%s' to bel '%s'\n", ctx->nameOf(ci), ctx->nameOfBel(bel));

        for (int o = 0; o < (ci->type == id_MMCM_MMCM_TOP ? 7 : 2); o++) {
            NetInfo *n = ci->getPort(ctx->idf("CLKOUT%d", o));
            if (n == nullptr)
                continue;
            for (auto &usr : n->users) {
                if (usr.cell->type != id_BUFCE_BUFCE || usr.port != id_I)
                    log_error("'%s' CLKOUT%d drives %s '%s'; only a BUFGCE is supported there on UltraScale+\n",
                              ctx->nameOf(ci), o, usr.cell->type.c_str(ctx), ctx->nameOf(usr.cell));
                if (usr.cell->bel != BelId() || usr.cell->attrs.count(id_BEL))
                    log_error("BUFGCE '%s' is fed by '%s' and must sit in its CMT; remove its placement constraint\n",
                              ctx->nameOf(usr.cell), ctx->nameOf(ci));
                BelId found;
                for (BelId b : bufgce_bels)
                    if (b.tile == bel.tile && !track_used.at(site_y(b) % 24) && ctx->checkBelAvail(b)) {
                        found = b;
                        break;
                    }
                if (found == BelId())
                    log_error("No free clock track left in the CMT of '%s' for BUFGCE '%s'\n", ctx->nameOf(ci),
                              ctx->nameOf(usr.cell));
                track_used.at(site_y(found) % 24) = true;
                ctx->bindBel(found, usr.cell, STRENGTH_LOCKED);
                log_info("    Constrained BUFGCE '%s' to bel '%s' (clock track %d)\n", ctx->nameOf(usr.cell),
                         ctx->nameOfBel(found), site_y(found) % 24);
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
            // A BUFGCE fed by an MMCM/PLL is place_cmt's.
            NetInfo *i = ci->getPort(id_I);
            if (i != nullptr && i->driver.cell != nullptr &&
                i->driver.cell->type.in(id_MMCM_MMCM_TOP, id_PLL_PLL_TOP))
                continue;
            if (ci->type.in(id_BUFCE_BUFG_PS, id_BUFCE_BUFCE, id_BUFGCE_DIV_BUFGCE_DIV))
                did_something |= try_preplace(ci, id_I);
            else if (ci->type == id_BUFGCTRL)
                did_something |= try_preplace(ci, id_I0);
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
    place_cmt();
    place_fabric_bufgce();
}

NEXTPNR_NAMESPACE_END
