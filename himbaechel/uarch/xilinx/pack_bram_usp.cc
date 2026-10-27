/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2019-2023  Myrtle Shah <gatecat@ds0.me>
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

// UltraScale+ block RAM packing, ported from nextpnr-xilinx's
// USPacker::pack_bram. RAMB18E2 and RAMB36E2 only: FIFOs are rejected by
// check_unsupported(), and cascading (CASCADE_ORDER_A/B) is not handled.

#include <boost/algorithm/string.hpp>
#include "extra_data.h"
#include "log.h"
#include "nextpnr.h"
#include "pack.h"
#include "pins.h"

#define HIMBAECHEL_CONSTIDS "uarch/xilinx/constids.inc"
#include "himbaechel_constids.h"

NEXTPNR_NAMESPACE_BEGIN

void USPacker::pack_bram()
{
    log_info("Packing BRAM..\n");

    // A LOC on a BRAM names a site (RAMB18_X0Y60, RAMB36_X0Y30). Turn it into
    // the BEL attribute the placer honours.
    dict<IdString, BelId> bram_site_bels;
    auto find_loc_bel = [&](IdString site_name) {
        if (bram_site_bels.empty()) {
            for (BelId bel : ctx->getBels()) {
                IdString t = ctx->getBelType(bel);
                if (t.in(id_RAMB18E2_RAMB18E2, id_RAMB36E2_RAMB36E2))
                    bram_site_bels[uarch->get_site_name(uarch->get_bel_site(bel))] = bel;
            }
        }
        auto found = bram_site_bels.find(site_name);
        return found == bram_site_bels.end() ? BelId() : found->second;
    };
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (!ci->type.in(id_RAMB18E2, id_RAMB36E2) || !ci->attrs.count(id_LOC))
            continue;
        std::string loc = ci->attrs.at(id_LOC).as_string();
        BelId bel = find_loc_bel(ctx->id(loc));
        if (bel == BelId() || ctx->getBelType(bel) != (ci->type == id_RAMB18E2 ? id_RAMB18E2_RAMB18E2
                                                                               : id_RAMB36E2_RAMB36E2))
            log_error("Can't place %s '%s' at LOC %s\n", ci->type.c_str(ctx), ctx->nameOf(ci), loc.c_str());
        ci->attrs[id_BEL] = std::string(ctx->nameOfBel(bel));
    }

    // Rules for normal TDP BRAM
    dict<IdString, XFormRule> bram_rules;
    bram_rules[id_RAMB18E2].new_type = id_RAMB18E2_RAMB18E2;
    bram_rules[id_RAMB18E2].port_multixform[ctx->id("WEA[0]")] = {id_WEA0, id_WEA1};
    bram_rules[id_RAMB18E2].port_multixform[ctx->id("WEA[1]")] = {id_WEA2, id_WEA3};
    bram_rules[id_RAMB36E2].new_type = id_RAMB36E2_RAMB36E2;

    // Some ports have upper/lower bel pins in 36-bit mode
    std::vector<std::pair<IdString, std::vector<std::string>>> ul_pins;
    get_bram36_ul_pins(ctx, ul_pins, true);
    for (auto &ul : ul_pins) {
        for (auto &bp : ul.second)
            bram_rules[id_RAMB36E2].port_multixform[ul.first].push_back(ctx->id(bp));
    }
    bram_rules[id_RAMB36E2].port_multixform[id_ECCPIPECE] = {id_ECCPIPECEL};

    // Special rules for SDP rules, relating to WE connectivity
    dict<IdString, XFormRule> sdp_bram_rules = bram_rules;
    for (int i = 0; i < 2; i++) {
        // Connects to two WEBWE bel pins
        sdp_bram_rules[id_RAMB18E2].port_multixform[ctx->idf("WEBWE[%d]", i)].push_back(ctx->idf("WEBWE%d", i * 2));
        sdp_bram_rules[id_RAMB18E2].port_multixform[ctx->idf("WEBWE[%d]", i)].push_back(
                ctx->idf("WEBWE%d", i * 2 + 1));
        // Not used in SDP mode
        sdp_bram_rules[id_RAMB18E2].port_multixform[ctx->idf("WEA[%d]", i)] = {};
    }
    for (int i = 0; i < 2; i++) {
        // Connects to two WEA bel pins
        sdp_bram_rules[id_RAMB18E2].port_multixform[ctx->idf("WEBWE[%d]", i + 2)].push_back(ctx->idf("WEA%d", i * 2));
        sdp_bram_rules[id_RAMB18E2].port_multixform[ctx->idf("WEBWE[%d]", i + 2)].push_back(
                ctx->idf("WEA%d", i * 2 + 1));
    }

    for (int i = 0; i < 4; i++) {
        sdp_bram_rules[id_RAMB36E2].port_multixform[ctx->idf("WEBWE[%d]", i)].clear();
        sdp_bram_rules[id_RAMB36E2].port_multixform[ctx->idf("WEBWE[%d]", i + 4)].clear();
        // Connects to two WEBWE bel pins
        sdp_bram_rules[id_RAMB36E2].port_multixform[ctx->idf("WEBWE[%d]", i)].push_back(ctx->idf("WEBWEL%d", i));
        sdp_bram_rules[id_RAMB36E2].port_multixform[ctx->idf("WEBWE[%d]", i)].push_back(ctx->idf("WEBWEU%d", i));
        sdp_bram_rules[id_RAMB36E2].port_multixform[ctx->idf("WEBWE[%d]", i + 4)].push_back(ctx->idf("WEAL%d", i));
        sdp_bram_rules[id_RAMB36E2].port_multixform[ctx->idf("WEBWE[%d]", i + 4)].push_back(ctx->idf("WEAU%d", i));
        // Not used in SDP mode
        sdp_bram_rules[id_RAMB36E2].port_multixform[ctx->idf("WEA[%d]", i)] = {};
    }

    // 72-bit BRAMs: drop upper bits of WEB in TDP mode
    for (int i = 4; i < 8; i++)
        bram_rules[id_RAMB36E2].port_multixform[ctx->idf("WEBWE[%d]", i)] = {};

    // Process SDP BRAM first
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if ((ci->type == id_RAMB18E2 && int_or_default(ci->params, id_WRITE_WIDTH_B, 0) == 36) ||
            (ci->type == id_RAMB36E2 && int_or_default(ci->params, id_WRITE_WIDTH_B, 0) == 72))
            xform_cell(sdp_bram_rules, ci);
    }

    // Rewrite byte enables according to data width
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type.in(id_RAMB18E2, id_RAMB36E2)) {
            for (char port : {'A', 'B'}) {
                int write_width = int_or_default(ci->params, ctx->idf("WRITE_WIDTH_%c", port), 18);
                int we_width;
                if (ci->type == id_RAMB36E2)
                    we_width = 4;
                else
                    we_width = (port == 'B') ? 4 : 2;
                if (write_width >= (9 * we_width))
                    continue;
                int used_we_width = std::max(write_width / 9, 1);
                for (int i = used_we_width; i < we_width; i++) {
                    NetInfo *low_we = ci->getPort(
                            ctx->idf("%s[%d]", port == 'B' ? "WEBWE" : "WEA", i % used_we_width));
                    IdString curr_we = ctx->idf("%s[%d]", port == 'B' ? "WEBWE" : "WEA", i);
                    if (!ci->ports.count(curr_we)) {
                        ci->ports[curr_we].type = PORT_IN;
                        ci->ports[curr_we].name = curr_we;
                    }
                    ci->disconnectPort(curr_we);
                    ci->connectPort(curr_we, low_we);
                }
            }
        }
    }

    generic_xform(bram_rules, false);

    // These pins have no logical mapping, so must be tied after transformation
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type == id_RAMB18E2_RAMB18E2) {
            for (int i = 2; i < 4; i++) {
                IdString port = ctx->idf("WEA%d", i);
                if (!ci->ports.count(port)) {
                    ci->ports[port].name = port;
                    ci->ports[port].type = PORT_IN;
                    ci->connectPort(port, ctx->nets[ctx->id("$PACKER_VCC_NET")].get());
                }
            }
        }
    }
}

// URAM288 (0-ultrascale-bits designs/kv260_uram_char). One cell per site, no
// cascades: a URAM288 with CASCADE_ORDER other than NONE, REG_CAS, or a
// driven CAS_IN_* pin is refused. Tied pins were settled by pack_constants:
// an invertible pin tied 0 is fed VCC with IS_<pin>_INVERTED toggled, as
// Vivado feeds it.
void USPacker::pack_uram()
{
    log_info("Packing URAM..\n");
    dict<IdString, BelId> site_bels;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type != id_URAM288)
            continue;
        for (const char *p : {"CASCADE_ORDER_A", "CASCADE_ORDER_B"})
            if (str_or_default(ci->params, ctx->id(p), "NONE") != "NONE")
                log_error("URAM288 '%s': %s is not NONE; URAM cascades are not supported\n", ctx->nameOf(ci), p);
        for (const char *p : {"REG_CAS_A", "REG_CAS_B"})
            if (str_or_default(ci->params, ctx->id(p), "FALSE") != "FALSE")
                log_error("URAM288 '%s': %s is set; URAM cascades are not supported\n", ctx->nameOf(ci), p);
        std::vector<IdString> cas;
        for (auto &port : ci->ports) {
            if (!boost::starts_with(port.first.str(ctx), "CAS_"))
                continue;
            NetInfo *n = port.second.net;
            bool used = n != nullptr && (port.second.type == PORT_OUT
                                                 ? !n->users.empty()
                                                 : n->name != ctx->id("$PACKER_GND_NET"));
            if (used)
                log_error("URAM288 '%s': cascade pin %s is connected; URAM cascades are not supported\n",
                          ctx->nameOf(ci), port.first.c_str(ctx));
            cas.push_back(port.first);
        }
        for (IdString p : cas)
            ci->disconnectPort(p);
        if (ci->attrs.count(id_LOC)) {
            if (site_bels.empty())
                for (BelId bel : ctx->getBels())
                    if (ctx->getBelType(bel) == id_BEL_URAM288)
                        site_bels[uarch->get_site_name(uarch->get_bel_site(bel))] = bel;
            std::string loc = ci->attrs.at(id_LOC).as_string();
            auto found = site_bels.find(ctx->id(loc));
            if (found == site_bels.end())
                log_error("Can't place URAM288 '%s' at LOC %s\n", ctx->nameOf(ci), loc.c_str());
            ci->attrs[id_BEL] = std::string(ctx->nameOfBel(found->second));
        }
    }
    dict<IdString, XFormRule> uram_rules;
    uram_rules[id_URAM288].new_type = id_BEL_URAM288;
    generic_xform(uram_rules, false);
}

NEXTPNR_NAMESPACE_END
