/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2019  David Shah <dave@ds0.me>
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

// UltraScale+ DSP48E2 packing, after nextpnr-xilinx's pack_dsp_xcup.cc.
//
// A DSP48E2 site holds eight bels; the primitive is split into one cell per
// bel, clustered on DSP_PREADD_DATA with the others at fixed z offsets (the
// order of gen/filters.py). Each primitive port goes to the bel pin(s) on
// its site wire: A/B/ACIN/BCIN on DSP_A_B_DATA, D (as DIN) and INMODE on
// DSP_PREADD_DATA, C on DSP_C_DATA, the ALU controls on DSP_ALU, P and the
// flags on DSP_OUTPUT, CLK on every clocked bel and CEM on both DSP_ALU and
// DSP_M_DATA. Every sub-cell carries the primitive's parameters; the FASM
// writer reads them from the DSP_ALU cell.
//
// Cascades (ACIN/BCIN/PCIN/CARRYCASCIN/MULTSIGNIN from another DSP) are not
// supported yet and are refused.

#include <array>
#include <boost/algorithm/string.hpp>
#include "log.h"
#include "nextpnr.h"
#include "pack.h"

#define HIMBAECHEL_CONSTIDS "uarch/xilinx/constids.inc"
#include "himbaechel_constids.h"

NEXTPNR_NAMESPACE_BEGIN

namespace {
enum DspSub
{
    PREADD_DATA = 0,
    PREADD,
    A_B_DATA,
    MULTIPLIER,
    C_DATA,
    M_DATA,
    ALU,
    OUTPUT,
    NUM_SUB
};

bool is_cascade_input(IdString port, const Context *ctx)
{
    const std::string &s = port.str(ctx);
    return boost::starts_with(s, "ACIN") || boost::starts_with(s, "BCIN") || boost::starts_with(s, "PCIN") ||
           s == "CARRYCASCIN" || s == "MULTSIGNIN";
}

bool is_cascade_output(IdString port, const Context *ctx)
{
    const std::string &s = port.str(ctx);
    return boost::starts_with(s, "ACOUT") || boost::starts_with(s, "BCOUT") || boost::starts_with(s, "PCOUT") ||
           s == "CARRYCASCOUT" || s == "MULTSIGNOUT";
}

// The invertible pins: tied, Vivado feeds them VCC and inverts it for a 0.
bool is_invertible(const std::string &port)
{
    std::string bus = port.substr(0, port.find('['));
    return bus == "ALUMODE" || bus == "CARRYIN" || bus == "INMODE" || bus == "OPMODE" || boost::starts_with(bus, "RST");
}

// Pins with tie bits of their own, set when tied (kv260_dsp_char): the
// invertible ones (the inversion bit), CARRYINSEL and D. A tied A, B, C or CE
// pin has none and is just routed from its constant.
bool is_self_tied(const std::string &port)
{
    std::string bus = port.substr(0, port.find('['));
    return is_invertible(port) || bus == "CARRYINSEL" || bus == "D";
}

// The bel pins a DSP48E2 port lands on: (sub-cell, pin name).
std::vector<std::pair<int, std::string>> dsp_port_map(const std::string &port)
{
    std::string bus = port, idx;
    auto br = port.find('[');
    if (br != std::string::npos) {
        bus = port.substr(0, br);
        idx = port.substr(br + 1, port.size() - br - 2);
    }
    auto one = [&](int sub, const std::string &pin) { return std::vector<std::pair<int, std::string>>{{sub, pin}}; };
    if (bus == "A" || bus == "B" || bus == "CEA1" || bus == "CEA2" || bus == "CEB1" || bus == "CEB2" ||
        bus == "RSTA" || bus == "RSTB")
        return one(A_B_DATA, bus + idx);
    if (bus == "ACIN" || bus == "BCIN")
        return one(A_B_DATA, bus + idx);
    if (bus == "C" || bus == "CEC" || bus == "RSTC")
        return one(C_DATA, bus + idx);
    if (bus == "D")
        return one(PREADD_DATA, "DIN" + idx);
    if (bus == "INMODE" || bus == "CEAD" || bus == "CED" || bus == "CEINMODE" || bus == "RSTD" ||
        bus == "RSTINMODE")
        return one(PREADD_DATA, bus + idx);
    if (bus == "ALUMODE" || bus == "OPMODE" || bus == "CARRYINSEL" || bus == "CARRYIN" || bus == "PCIN" ||
        bus == "CARRYCASCIN" || bus == "MULTSIGNIN" || bus == "CEALUMODE" || bus == "CECARRYIN" ||
        bus == "CECTRL" || bus == "RSTALLCARRYIN" || bus == "RSTALUMODE" || bus == "RSTCTRL")
        return one(ALU, bus + idx);
    if (bus == "CEM")
        return {{ALU, "CEM"}, {M_DATA, "CEM"}};
    if (bus == "RSTM")
        return one(M_DATA, "RSTM");
    if (bus == "CEP" || bus == "RSTP" || bus == "P" || bus == "CARRYOUT" || bus == "XOROUT" || bus == "OVERFLOW" ||
        bus == "UNDERFLOW" || bus == "PCOUT" || bus == "CARRYCASCOUT" || bus == "MULTSIGNOUT")
        return one(OUTPUT, bus + idx);
    if (bus == "PATTERNDETECT")
        return one(OUTPUT, "PATTERN_DETECT");
    if (bus == "PATTERNBDETECT")
        return one(OUTPUT, "PATTERN_B_DETECT");
    if (bus == "ACOUT" || bus == "BCOUT")
        return one(A_B_DATA, bus + idx);
    if (bus == "CLK")
        return {{ALU, "CLK"}, {A_B_DATA, "CLK"}, {C_DATA, "CLK"}, {M_DATA, "CLK"}, {OUTPUT, "CLK"}, {PREADD_DATA, "CLK"}};
    return {};
}
} // namespace

void USPacker::pack_dsps()
{
    log_info("Packing DSPs..\n");
    const IdString sub_types[NUM_SUB] = {id_DSP_PREADD_DATA, id_DSP_PREADD, id_DSP_A_B_DATA, id_DSP_MULTIPLIER,
                                         id_DSP_C_DATA,      id_DSP_M_DATA, id_DSP_ALU,      id_DSP_OUTPUT};

    // A LOC names a DSP48E2 site. The constraint placer binds only cells with
    // a BEL of their own (not a cluster's children), so every sub-cell gets
    // its bel in that site.
    dict<IdString, std::array<BelId, NUM_SUB>> site_bels;
    auto loc_bels = [&](IdString site) -> const std::array<BelId, NUM_SUB> * {
        if (site_bels.empty())
            for (BelId bel : ctx->getBels())
                for (int i = 0; i < NUM_SUB; i++)
                    if (ctx->getBelType(bel) == sub_types[i])
                        site_bels[uarch->get_site_name(uarch->get_bel_site(bel))][i] = bel;
        auto found = site_bels.find(site);
        return found == site_bels.end() ? nullptr : &found->second;
    };

    int count = 0;
    std::vector<CellInfo *> dsps;
    for (auto &cell : ctx->cells)
        if (cell.second->type == id_DSP48E2)
            dsps.push_back(cell.second.get());
    for (CellInfo *ci : dsps) {
        std::vector<IdString> ports;
        for (auto &port : ci->ports)
            ports.push_back(port.first);
        for (IdString p : ports) {
            NetInfo *n = ci->getPort(p);
            if (n == nullptr)
                continue;
            if (is_cascade_input(p, ctx)) {
                // Synthesis ties unused cascade inputs low; they have no routing.
                if (n->name != ctx->id("$PACKER_GND_NET"))
                    log_error("DSP48E2 '%s': cascade input %s is driven; DSP cascades are not supported on "
                              "UltraScale+ yet\n",
                              ctx->nameOf(ci), p.c_str(ctx));
                ci->disconnectPort(p);
            } else if (is_cascade_output(p, ctx)) {
                if (!n->users.empty())
                    log_error("DSP48E2 '%s': cascade output %s is used; DSP cascades are not supported on "
                              "UltraScale+ yet\n",
                              ctx->nameOf(ci), p.c_str(ctx));
                ci->disconnectPort(p);
            }
        }
        for (auto ab : {"A_INPUT", "B_INPUT"})
            if (str_or_default(ci->params, ctx->id(ab), "DIRECT") != "DIRECT")
                log_error("DSP48E2 '%s': %s=%s needs a cascade, which is not supported on UltraScale+ yet\n",
                          ctx->nameOf(ci), ab, str_or_default(ci->params, ctx->id(ab), "").c_str());

        std::vector<CellInfo *> subs;
        for (int i = 0; i < NUM_SUB; i++) {
            CellInfo *sc = create_cell(sub_types[i], int_name(ci->name, sub_types[i].str(ctx) + "_INST", true));
            sc->params = ci->params;
            sc->attrs[id_X_ORIG_MACRO_PRIM] = ci->type.str(ctx);
            if (i == 0) {
                sc->cluster = sc->name;
            } else {
                sc->cluster = subs.front()->name;
                subs.front()->constr_children.push_back(sc);
                sc->constr_x = 0;
                sc->constr_y = 0;
                sc->constr_z = i;
                sc->constr_abs_z = false;
            }
            subs.push_back(sc);
        }
        if (ci->attrs.count(id_LOC)) {
            std::string loc = ci->attrs.at(id_LOC).as_string();
            auto bels = loc_bels(ctx->id(loc));
            if (bels == nullptr)
                log_error("Can't place DSP48E2 '%s' at LOC %s\n", ctx->nameOf(ci), loc.c_str());
            for (int i = 0; i < NUM_SUB; i++)
                subs.at(i)->attrs[id_BEL] = std::string(ctx->nameOfBel(bels->at(i)));
        }

        // A tied pin is routed as Vivado routes it (kv260_dsp_test reference,
        // 2026-10-27): an invertible pin always from VCC, its inversion bit
        // making the constant; a CARRYINSEL or D bit from its own constant,
        // with the pin's tie bits set as well. The FASM writer reads
        // X_TIE_<pin>.
        dict<IdString, Property> ties;
        NetInfo *vcc_net = ctx->nets.at(ctx->id("$PACKER_VCC_NET")).get();
        for (IdString p : ports) {
            NetInfo *n = ci->getPort(p);
            if (n == nullptr || !is_self_tied(p.str(ctx)))
                continue;
            bool gnd = n->name == ctx->id("$PACKER_GND_NET"), vcc = n->name == ctx->id("$PACKER_VCC_NET");
            if (!gnd && !vcc)
                continue;
            ties[ctx->id("X_TIE_" + p.str(ctx))] = Property(vcc ? 1 : 0, 1);
            if (is_invertible(p.str(ctx)) && gnd) {
                ci->disconnectPort(p);
                ci->connectPort(p, vcc_net);
            }
        }
        for (auto &t : ties)
            subs.at(ALU)->attrs[t.first] = t.second;

        for (IdString p : ports) {
            NetInfo *n = ci->getPort(p);
            if (n == nullptr)
                continue;
            auto targets = dsp_port_map(p.str(ctx));
            if (targets.empty())
                log_error("DSP48E2 '%s': port %s is not known to the UltraScale+ packer\n", ctx->nameOf(ci),
                          p.c_str(ctx));
            PortType dir = ci->ports.at(p).type;
            ci->disconnectPort(p);
            for (auto &t : targets) {
                IdString pin = ctx->id(t.second);
                if (dir == PORT_OUT)
                    subs.at(t.first)->addOutput(pin);
                else
                    subs.at(t.first)->addInput(pin);
                subs.at(t.first)->connectPort(pin, n);
            }
        }
        packed_cells.insert(ci->name);
        count++;
    }
    flush_cells();
    if (count)
        log_info("    %d DSP48E2s, each on eight bels\n", count);
}

NEXTPNR_NAMESPACE_END
