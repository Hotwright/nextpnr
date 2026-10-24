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
// pack_clocking_xcup.cc. Global buffers only for now: the MMCM/PLL path is
// rejected by check_unsupported() until it is ported.

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

void USPacker::pack_gbs()
{
    log_info("Packing global buffers...\n");
    dict<IdString, XFormRule> gb_rules;
    gb_rules[id_BUFG_PS].new_type = id_BUFCE_BUFG_PS;
    gb_rules[id_BUFGCE].new_type = id_BUFCE_BUFCE;

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
            if (ci->type.in(id_BUFCE_BUFG_PS, id_BUFCE_BUFCE))
                did_something |= try_preplace(ci, id_I);
        }
    } while (did_something);
}

void USPacker::pack_clocking()
{
    pack_gbs();
    preplace_clocking();
}

NEXTPNR_NAMESPACE_END
