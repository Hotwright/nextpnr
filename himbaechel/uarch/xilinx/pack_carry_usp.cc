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

// UltraScale+ carry packing, ported from nextpnr-xilinx's pack_carry_xcup.cc.
//
// Same scheme as the 7-series packer: split CARRY4/CARRY8 into MUXCY/XORCY,
// walk the chains, then fold eight of each into one CARRY8 per CLE. The chain
// is constrained one CLE row per CARRY8; XilinxImpl::getClusterPlacement steps
// over the RCLK/RBRK rows that break up every CLE column.

#include "chain_utils.h"
#include "design_utils.h"
#include "extra_data.h"
#include "log.h"
#include "nextpnr.h"
#include "pack.h"
#include "pins.h"

#define HIMBAECHEL_CONSTIDS "uarch/xilinx/constids.inc"
#include "himbaechel_constids.h"

NEXTPNR_NAMESPACE_BEGIN

bool USPacker::has_illegal_fanout(NetInfo *carry)
{
    // FIXME: sometimes we can feed out of the chain
    if (carry->users.entries() > 2)
        return true;
    CellInfo *muxcy = nullptr, *xorcy = nullptr;
    for (auto &usr : carry->users) {
        if (usr.cell->type == id_MUXCY) {
            if (muxcy != nullptr)
                return true;
            else if (usr.port != id_CI)
                return true;
            else
                muxcy = usr.cell;
        } else if (usr.cell->type == id_XORCY) {
            if (xorcy != nullptr)
                return true;
            else if (usr.port != id_CI)
                return true;
            else
                xorcy = usr.cell;
        } else {
            return true;
        }
    }
    if (muxcy && xorcy) {
        NetInfo *muxcy_s = muxcy->getPort(id_S);
        NetInfo *xorcy_li = xorcy->getPort(id_LI);
        if (muxcy_s != xorcy_li)
            return true;
    }
    return false;
}

// Newer yosys maps xcu/xcup arithmetic to CARRY8 rather than CARRY4.
void USPacker::split_carry8s()
{
    std::vector<CellInfo *> carry8s;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type != id_CARRY8)
            continue;
        carry8s.push_back(ci);
    }
    for (CellInfo *ci : carry8s) {
        if (str_or_default(ci->params, id_CARRY_TYPE, "SINGLE_CY8") != "SINGLE_CY8")
            log_error("CARRY8 '%s' has CARRY_TYPE %s; only SINGLE_CY8 is supported\n", ctx->nameOf(ci),
                      str_or_default(ci->params, id_CARRY_TYPE).c_str());
        NetInfo *cin = ci->getPort(id_CI);
        ci->disconnectPort(id_CI);
        ci->disconnectPort(ctx->id("CI_TOP"));

        for (int i = 0; i < 8; i++) {
            CellInfo *xorcy = create_cell(id_XORCY, ctx->idf("%s$split$xorcy%d", ci->name.c_str(ctx), i));
            CellInfo *muxcy = create_cell(id_MUXCY, ctx->idf("%s$split$muxcy%d", ci->name.c_str(ctx), i));
            muxcy->connectPort(id_CI, cin);
            xorcy->connectPort(id_CI, cin);
            ci->movePortTo(ctx->idf("DI[%d]", i), muxcy, id_DI);
            muxcy->connectPort(id_S, ci->getPort(ctx->idf("S[%d]", i)));
            ci->movePortTo(ctx->idf("S[%d]", i), xorcy, id_LI);
            ci->movePortTo(ctx->idf("O[%d]", i), xorcy, id_O);
            NetInfo *co = ci->getPort(ctx->idf("CO[%d]", i));
            ci->disconnectPort(ctx->idf("CO[%d]", i));
            if (!co)
                co = create_internal_net(ci->name, stringf("$split$co%d", i), false);
            muxcy->connectPort(id_O, co);
            cin = co;
        }
        packed_cells.insert(ci->name);
    }
    flush_cells();
}

void USPacker::pack_carries()
{
    log_info("Packing carries..\n");
    split_carry4s();
    split_carry8s();
    std::vector<CellInfo *> root_muxcys;
    // Find MUXCYs
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type != id_MUXCY)
            continue;
        NetInfo *ci_net = ci->getPort(id_CI);
        if (!ci_net || !ci_net->driver.cell || ci_net->driver.cell->type != id_MUXCY || has_illegal_fanout(ci_net)) {
            root_muxcys.push_back(ci);
        }
    }

    // Create chains from root MUXCYs
    std::vector<CarryGroup> groups;
    int muxcy_count = 0, xorcy_count = 0;
    for (auto root : root_muxcys) {
        CarryGroup group;

        CellInfo *muxcy = root;
        NetInfo *mux_ci = nullptr;
        while (true) {

            group.muxcys.push_back(muxcy);
            ++muxcy_count;
            mux_ci = muxcy->getPort(id_CI);
            NetInfo *mux_s = muxcy->getPort(id_S);
            group.xorcys.push_back(nullptr);
            if (mux_s != nullptr) {
                for (auto &user : mux_s->users) {
                    if (user.cell->type == id_XORCY && user.port == id_LI) {
                        CellInfo *xorcy = user.cell;
                        NetInfo *xor_ci = xorcy->getPort(id_CI);
                        if (xor_ci == mux_ci) {
                            group.xorcys.back() = xorcy;
                            ++xorcy_count;
                            break;
                        }
                    }
                }
            }

            mux_ci = muxcy->getPort(id_O);
            if (mux_ci == nullptr)
                break;
            if (has_illegal_fanout(mux_ci))
                break;
            muxcy = nullptr;
            for (auto &user : mux_ci->users) {
                if (user.cell->type == id_MUXCY) {
                    muxcy = user.cell;
                    break;
                }
            }
            if (muxcy == nullptr)
                break;
        }
        if (mux_ci != nullptr) {
            if (mux_ci->users.entries() == 1 && (*mux_ci->users.begin()).cell->type == id_XORCY &&
                (*mux_ci->users.begin()).port == id_CI) {
                // Trailing XORCY at end, can pack into chain.
                CellInfo *xorcy = (*mux_ci->users.begin()).cell;
                CellInfo *dummy_muxcy = create_cell(id_MUXCY, ctx->idf("%s$legal_muxcy$", xorcy->name.c_str(ctx)));
                dummy_muxcy->connectPort(id_CI, mux_ci);
                dummy_muxcy->connectPort(id_S, xorcy->getPort(id_LI));
                group.muxcys.push_back(dummy_muxcy);
                group.xorcys.push_back(xorcy);
            } else if (mux_ci->users.entries() > 0) {
                // Users other than a MUXCY
                // Feed out with a zero-driving LUT and a XORCY
                // (creating a zero-driver using Vcc and an inverter for now...)
                CellInfo *zero_lut = create_lut(mux_ci->name.str(ctx) + "$feed$zero",
                                                {ctx->nets.at(ctx->id("$PACKER_VCC_NET")).get()}, nullptr, Property(1));
                CellInfo *feed_xorcy = create_cell(id_XORCY, ctx->id(mux_ci->name.str(ctx) + "$feed$xor"));
                CellInfo *dummy_muxcy = create_cell(id_MUXCY, ctx->id(mux_ci->name.str(ctx) + "$feed$muxcy"));

                CellInfo *last_muxcy = mux_ci->driver.cell;

                last_muxcy->disconnectPort(id_O);

                zero_lut->connectPorts(id_O, feed_xorcy, id_LI);
                zero_lut->connectPorts(id_O, dummy_muxcy, id_S);
                last_muxcy->connectPorts(id_O, feed_xorcy, id_CI);
                last_muxcy->connectPorts(id_O, dummy_muxcy, id_CI);

                feed_xorcy->connectPort(id_O, mux_ci);

                group.muxcys.push_back(dummy_muxcy);
                group.xorcys.push_back(feed_xorcy);
            }
        }

        groups.push_back(group);
    }
    flush_cells();

    log_info("   Grouped %d MUXCYs and %d XORCYs into %d chains.\n", muxcy_count, xorcy_count, int(root_muxcys.size()));

    // N.B. LUT6 is not a valid type here, as CARRY requires dual outputs
    pool<IdString> lut_types{id_LUT1, id_LUT2, id_LUT3, id_LUT4, id_LUT5};

    pool<IdString> folded_nets;

    for (auto &grp : groups) {
        std::vector<CellInfo *> carry8s;
        for (int i = 0; i < int(grp.muxcys.size()); i++) {
            int z = i % 8;
            CellInfo *muxcy = grp.muxcys.at(i), *xorcy = grp.xorcys.at(i);
            if (z == 0)
                carry8s.push_back(create_cell(id_CARRY8, ctx->idf("%s$PACKED_CARRY8$", muxcy->name.c_str(ctx))));
            CellInfo *c8 = carry8s.back();
            CellInfo *root = carry8s.front();
            if (i == 0) {
                // Constrain initial CARRY8, forcing it to the CARRY8 of a logic tile
                c8->cluster = c8->name;
                c8->constr_abs_z = true;
                c8->constr_z = BEL_CARRY8;
            } else if (z == 0) {
                // Constrain relative to the root carry8. This counts CARRY8s, not
                // grid rows: getClusterPlacement maps it past the non-CLE rows.
                c8->cluster = root->name;
                root->constr_children.push_back(c8);
                c8->constr_x = 0;
                c8->constr_y = -i / 8;
                c8->constr_abs_z = true;
                c8->constr_z = BEL_CARRY8;
            }
            // Fold CI->CO connections into the CARRY8, except for those external ones every 8 units
            if (z == 0) {
                muxcy->movePortTo(id_CI, c8, id_CI);
            } else {
                NetInfo *muxcy_ci = muxcy->getPort(id_CI);
                if (muxcy_ci)
                    folded_nets.insert(muxcy_ci->name);
                muxcy->disconnectPort(id_CI);
            }
            if (z == 7) {
                muxcy->movePortTo(id_O, c8, ctx->id("CO[7]"));
            } else {
                NetInfo *muxcy_o = muxcy->getPort(id_O);
                if (muxcy_o)
                    folded_nets.insert(muxcy_o->name);
                muxcy->disconnectPort(id_O);
            }
            // Replace connections into the MUXCY with external CARRY8 ports
            muxcy->movePortTo(id_S, c8, ctx->idf("S[%d]", z));
            muxcy->movePortTo(id_DI, c8, ctx->idf("DI[%d]", z));
            packed_cells.insert(muxcy->name);
            // Fold MUXCY->XORCY into the CARRY8, if there is a XORCY
            if (xorcy) {
                // Replace XORCY output with external CARRY8 output
                xorcy->movePortTo(id_O, c8, ctx->idf("O[%d]", z));
                // Disconnect internal XORCY connectivity
                xorcy->disconnectPort(id_LI);
                xorcy->disconnectPort(id_DI);
                packed_cells.insert(xorcy->name);
            }
            // Check legality of LUTs driving CARRY8, making them legal if they aren't already
            NetInfo *c8_s = c8->getPort(ctx->idf("S[%d]", z));
            NetInfo *c8_di = c8->getPort(ctx->idf("DI[%d]", z));
            // Keep track of the total LUT input count; cannot exceed five or the LUTs cannot be packed together
            pool<IdString> unique_lut_inputs;
            int s_inputs = 0;
            // Check that S and DI are validy and unqiuely driven by LUTs
            // FIXME: in multiple fanout cases, cell duplication will probably be cheaper
            // than feed-throughs
            CellInfo *s_lut = nullptr, *di_lut = nullptr;
            if (c8_s) {
                if (c8_s->users.entries() == 1 && c8_s->driver.cell != nullptr &&
                    lut_types.count(c8_s->driver.cell->type)) {
                    s_lut = c8_s->driver.cell;
                    for (int j = 0; j < 5; j++) {
                        NetInfo *ix = s_lut->getPort(ctx->idf("I%d", j));
                        if (ix) {
                            unique_lut_inputs.insert(ix->name);
                            s_inputs++;
                        }
                    }
                }
            }
            if (c8_di) {
                if (c8_di->users.entries() == 1 && c8_di->driver.cell != nullptr &&
                    lut_types.count(c8_di->driver.cell->type)) {
                    di_lut = c8_di->driver.cell;
                    for (int j = 0; j < 5; j++) {
                        NetInfo *ix = di_lut->getPort(ctx->idf("I%d", j));
                        if (ix) {
                            unique_lut_inputs.insert(ix->name);
                        }
                    }
                }
            }
            int lut_inp_count = int(unique_lut_inputs.size());
            if (!s_lut)
                ++lut_inp_count; // for feedthrough
            if (!di_lut)
                ++lut_inp_count; // for feedthrough
            if (lut_inp_count > 5) {
                // Must use feedthrough for at least one LUT
                di_lut = nullptr;
                if (s_inputs > 4)
                    s_lut = nullptr;
            }
            // If LUTs are nullptr, that means we need a feedthrough lut. The
            // CARRY8 DI<n> bel pin is wired only to the <c>5LUT's O5, so DI
            // always comes through a LUT here.
            if (!s_lut && c8_s) {
                PortRef pr;
                pr.cell = c8;
                pr.port = ctx->idf("S[%d]", z);
                s_lut = feed_through_lut(c8_s, {pr});
            }
            if (!di_lut && c8_di) {
                PortRef pr;
                pr.cell = c8;
                pr.port = ctx->idf("DI[%d]", z);
                di_lut = feed_through_lut(c8_di, {pr});
            }
            // Constrain LUTs relative to root CARRY8
            if (s_lut) {
                root->constr_children.push_back(s_lut);
                s_lut->cluster = root->name;
                s_lut->constr_x = 0;
                s_lut->constr_y = -i / 8;
                s_lut->constr_abs_z = true;
                s_lut->constr_z = (z << 4 | BEL_6LUT);
            }
            if (di_lut) {
                root->constr_children.push_back(di_lut);
                di_lut->cluster = root->name;
                di_lut->constr_x = 0;
                di_lut->constr_y = -i / 8;
                di_lut->constr_abs_z = true;
                di_lut->constr_z = (z << 4 | BEL_5LUT);
            }
        }
    }
    flush_cells();

    for (auto net : folded_nets)
        ctx->nets.erase(net);

    // XORCYs and MUXCYs not part of any chain (and therefore not packed into a CARRY8) must now be blasted
    // to boring soft logic (LUT2 or LUT3 - these will become SLICE_LUTXs later in the flow.)
    int remaining_muxcy = 0, remaining_xorcy = 0;
    for (auto &cell : ctx->cells) {
        if (cell.second->type == id_MUXCY)
            ++remaining_muxcy;
        else if (cell.second->type == id_XORCY)
            ++remaining_xorcy;
    }
    dict<IdString, XFormRule> softlogic_rules;
    softlogic_rules[id_MUXCY].new_type = id_LUT3;
    softlogic_rules[id_MUXCY].port_xform[id_DI] = id_I0;
    softlogic_rules[id_MUXCY].port_xform[id_CI] = id_I1;
    softlogic_rules[id_MUXCY].port_xform[id_S] = id_I2;
    // DI 1010 1010
    // CI 1100 1100
    //  S 1111 0000
    //  O 1100 1010
    softlogic_rules[id_MUXCY].set_params.emplace_back(id_INIT, Property(0xCA));

    softlogic_rules[id_XORCY].new_type = id_LUT2;
    softlogic_rules[id_XORCY].port_xform[id_CI] = id_I0;
    softlogic_rules[id_XORCY].port_xform[id_LI] = id_I1;
    // CI 1100
    // LI 1010
    //  O 0110
    softlogic_rules[id_XORCY].set_params.emplace_back(id_INIT, Property(0x6));

    generic_xform(softlogic_rules, false);
    log_info("   Blasted %d non-chain MUXCYs and %d non-chain XORCYs to soft logic\n", remaining_muxcy,
             remaining_xorcy);

    // Finally, use xform_cell to remove the [] from bus ports, and move the
    // carry-in onto the bel pin it really arrives on.
    //
    // CIN is the dedicated carry from the CARRY8 in the CLE below, so only a
    // CARRY8 with a predecessor in its chain may use it. A chain root has
    // cluster == its own name, not ClusterId(); legacy nextpnr-xilinx tested
    // only the latter, sent the root down the chained path and asked the
    // router to reach SLICE/CIN from GND. A root's carry-in comes in on AX.
    //
    // A constant carry-in is not routed at all: the CARRY8 CI mux has V0 and
    // V1 settings, so the FASM writer selects one from X_CARRY8_CI_CONST and AX
    // stays free for the A flip-flop's bypass input. CI_TOP is only used in
    // DUAL_CY4 mode, which is never generated here; it is left unconnected
    // (CI_TOP.V0).
    dict<IdString, XFormRule> c8_chained_rules;
    c8_chained_rules[id_CARRY8].new_type = id_CARRY8;
    c8_chained_rules[id_CARRY8].port_xform[id_CI] = id_CIN;
    c8_chained_rules[id_CARRY8].set_params.emplace_back(id_CARRY_TYPE, Property("SINGLE_CY8"));
    dict<IdString, XFormRule> c8_init_rules;
    c8_init_rules[id_CARRY8].new_type = id_CARRY8;
    c8_init_rules[id_CARRY8].port_xform[id_CI] = id_AX;
    c8_init_rules[id_CARRY8].set_params.emplace_back(id_CARRY_TYPE, Property("SINGLE_CY8"));

    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type != id_CARRY8)
            continue;
        ci->disconnectPort(ctx->id("CI_TOP"));
        ci->ports.erase(ctx->id("CI_TOP"));
        bool is_chain_root = (ci->cluster == ClusterId()) || (ci->cluster == ClusterId(ci->name));
        if (is_chain_root) {
            NetInfo *cin = ci->getPort(id_CI);
            if (cin == nullptr || cin->name == ctx->id("$PACKER_GND_NET")) {
                ci->attrs[ctx->id("X_CARRY8_CI_CONST")] = Property(0, 1);
                ci->disconnectPort(id_CI);
            } else if (cin->name == ctx->id("$PACKER_VCC_NET")) {
                ci->attrs[ctx->id("X_CARRY8_CI_CONST")] = Property(1, 1);
                ci->disconnectPort(id_CI);
            }
            xform_cell(c8_init_rules, ci);
        } else {
            xform_cell(c8_chained_rules, ci);
        }
    }
}

NEXTPNR_NAMESPACE_END
