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

// UltraScale+ configuration primitives: ICAPE3, STARTUPE3, BSCANE2,
// DNA_PORTE2, USR_ACCESSE2, FRAME_ECCE4 and EFUSE_USR. Each has exactly one
// place to go in CONFIG_SITE (a single-SLR part has one), so the packer binds
// the BEL itself: ICAPE3 to ICAP_TOP unless the design asks for ICAP_BOT, and
// BSCANE2 to BSCAN<JTAG_CHAIN>. Bus ports lose their brackets in xform_cell
// (I[3] -> I3), which is how the bel pins are named.

#include "extra_data.h"
#include "log.h"
#include "nextpnr.h"
#include "pack.h"

#define HIMBAECHEL_CONSTIDS "uarch/xilinx/constids.inc"
#include "himbaechel_constids.h"

NEXTPNR_NAMESPACE_BEGIN

void USPacker::pack_config()
{
    // The CONFIG_SITE bels by their name within the site (ICAP_TOP, BSCAN3...)
    dict<std::string, BelId> config_bels;
    auto config_bel = [&](const std::string &name) {
        if (config_bels.empty()) {
            for (BelId bel : ctx->getBels()) {
                if (!ctx->getBelType(bel).in(id_ICAP_TOP, id_ICAP_BOT, id_STARTUP, id_BSCAN, id_DNA_PORT,
                                             id_USR_ACCESS, id_FRAME_ECC, id_EFUSE_USR))
                    continue;
                std::string full = ctx->getBelName(bel)[1].str(ctx);
                config_bels[full.substr(full.rfind('.') + 1)] = bel;
            }
        }
        auto found = config_bels.find(name);
        if (found == config_bels.end())
            log_error("This device has no CONFIG_SITE bel %s\n", name.c_str());
        return found->second;
    };

    dict<IdString, XFormRule> rules;
    rules[id_ICAPE3].new_type = id_ICAP_TOP;
    rules[id_STARTUPE3].new_type = id_STARTUP;
    rules[id_BSCANE2].new_type = id_BSCAN;
    rules[id_DNA_PORTE2].new_type = id_DNA_PORT;
    rules[id_USR_ACCESSE2].new_type = id_USR_ACCESS;
    rules[id_FRAME_ECCE4].new_type = id_FRAME_ECC;
    rules[id_EFUSE_USR].new_type = id_EFUSE_USR;

    dict<BelId, CellInfo *> used;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (!rules.count(ci->type))
            continue;
        std::string bel_name;
        if (ci->type == id_ICAPE3) {
            // A BEL (or LOC) of ICAP_BOT selects the other ICAP port
            std::string want = ci->attrs.count(id_BEL) ? ci->attrs.at(id_BEL).as_string() : "";
            bel_name = (want.find("ICAP_BOT") != std::string::npos) ? "ICAP_BOT" : "ICAP_TOP";
        } else if (ci->type == id_BSCANE2) {
            int chain = int_or_default(ci->params, id_JTAG_CHAIN, 1);
            if (chain < 1 || chain > 4)
                log_error("BSCANE2 '%s' has JTAG_CHAIN %d, which must be 1-4\n", ctx->nameOf(ci), chain);
            bel_name = "BSCAN" + std::to_string(chain);
        } else {
            bel_name = rules.at(ci->type).new_type.str(ctx);
        }
        BelId bel = config_bel(bel_name);
        if (used.count(bel))
            log_error("'%s' and '%s' both need CONFIG_SITE bel %s\n", ctx->nameOf(used.at(bel)), ctx->nameOf(ci),
                      bel_name.c_str());
        used[bel] = ci;
        xform_cell(rules, ci);
        if (bel_name == "ICAP_BOT")
            ci->type = id_ICAP_BOT;
        ci->attrs[id_BEL] = std::string(ctx->nameOfBel(bel));
    }
}

NEXTPNR_NAMESPACE_END
