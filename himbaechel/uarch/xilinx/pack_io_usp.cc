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

// UltraScale+ IO packing, ported from nextpnr-xilinx's pack_io_xcup.cc.
//
// Only the single-ended paths are here so far. Differential buffers, HPIO
// IOLOGIC (ISERDES/OSERDES/IDELAY/ODELAY) and IDELAYCTRL are rejected in
// check_unsupported() until they are ported and checked against a Vivado
// reference.

#include <boost/algorithm/string.hpp>
#include "extra_data.h"
#include "log.h"
#include "nextpnr.h"
#include "pack.h"
#include "pins.h"

#define HIMBAECHEL_CONSTIDS "uarch/xilinx/constids.inc"
#include "himbaechel_constids.h"

NEXTPNR_NAMESPACE_BEGIN

CellInfo *USPacker::insert_ibufctrl(IdString name, NetInfo *i, NetInfo *o)
{
    auto ibufc = create_cell(id_IBUFCTRL, name);
    ibufc->connectPort(id_I, i);
    ibufc->connectPort(id_O, o);
    return ibufc;
}

CellInfo *USPacker::insert_inbuf(IdString name, NetInfo *pad, NetInfo *o)
{
    auto inbuf = create_cell(id_INBUF, name);
    inbuf->connectPort(id_PAD, pad);
    inbuf->connectPort(id_O, o);
    return inbuf;
}

void USPacker::decompose_iob(CellInfo *xil_iob, const std::string &iostandard)
{
    bool is_se_ibuf = xil_iob->type.in(id_IBUF, id_IBUF_IBUFDISABLE, id_IBUF_INTERMDISABLE, id_IBUFE3);
    bool is_se_iobuf = xil_iob->type.in(id_IOBUF, id_IOBUF_DCIEN, id_IOBUF_INTERMDISABLE, id_IOBUFE3);
    bool is_se_obuf = xil_iob->type.in(id_OBUF, id_OBUFT);

    if (!is_se_ibuf && !is_se_iobuf && !is_se_obuf)
        log_error("IO buffer '%s' of type %s is not supported on UltraScale+ yet (only single-ended buffers are)\n",
                  ctx->nameOf(xil_iob), xil_iob->type.c_str(ctx));

    auto pad_site = [&](NetInfo *n) {
        for (auto user : n->users)
            if (user.cell->type == id_PAD)
                return uarch->get_bel_site(ctx->getBelByNameStr(user.cell->attrs[id_BEL].as_string()));
        NPNR_ASSERT_FALSE(("can't find PAD for net " + n->name.str(ctx)).c_str());
    };

    auto bind_in_site = [&](SiteIndex site, const char *bel_name, CellInfo *cell) {
        BelId bel = uarch->get_site_bel(site, ctx->id(bel_name));
        if (bel == BelId())
            log_error("IO site %s has no %s bel (for '%s')\n", uarch->get_site_name(site).c_str(ctx), bel_name,
                      ctx->nameOf(cell));
        ctx->bindBel(bel, cell, STRENGTH_LOCKED);
    };

    if (is_se_ibuf || is_se_iobuf) {
        NetInfo *pad_net = xil_iob->getPort(is_se_iobuf ? id_IO : id_I);
        NPNR_ASSERT(pad_net != nullptr);
        auto site = pad_site(pad_net);
        if (!is_se_iobuf)
            xil_iob->disconnectPort(id_I);
        NetInfo *inb_out = create_internal_net(xil_iob->name, "OUT");
        CellInfo *inbuf = insert_inbuf(int_name(xil_iob->name, "INBUF_INST"), pad_net, inb_out);
        bind_in_site(site, "INBUF", inbuf);
        // Don't need to check cell type here, as movePortTo is a no-op if port doesn't exist
        xil_iob->movePortTo(id_VREF, inbuf, id_VREF);
        xil_iob->movePortTo(id_OSC_EN, inbuf, id_OSC_EN);
        for (int i = 0; i < 4; i++)
            xil_iob->movePortTo(ctx->idf("OSC[%d]", i), inbuf, ctx->idf("OSC[%d]", i));

        NetInfo *top_out = xil_iob->getPort(id_O);
        xil_iob->disconnectPort(id_O);
        CellInfo *ibufctrl = insert_ibufctrl(int_name(xil_iob->name, "IBUFCTRL_INST"), inb_out, top_out);
        bind_in_site(site, "IBUFCTRL", ibufctrl);
        xil_iob->movePortTo(id_IBUFDISABLE, ibufctrl, id_IBUFDISABLE);
        if (is_se_iobuf)
            ibufctrl->connectPort(id_T, xil_iob->getPort(id_T));
    }
    if (is_se_obuf || is_se_iobuf) {
        NetInfo *pad_net = xil_iob->getPort(is_se_iobuf ? id_IO : id_O);
        NPNR_ASSERT(pad_net != nullptr);
        auto site = pad_site(pad_net);
        xil_iob->disconnectPort(is_se_iobuf ? id_IO : id_O);
        bool has_dci = xil_iob->type.in(id_IOBUF_DCIEN, id_IOBUFE3);
        CellInfo *obuf = insert_obuf(
                int_name(xil_iob->name, (is_se_iobuf || xil_iob->type == id_OBUFT) ? "OBUFT" : "OBUF", !is_se_obuf),
                is_se_iobuf ? (has_dci ? id_OBUFT_DCIEN : id_OBUFT) : xil_iob->type, xil_iob->getPort(id_I), pad_net,
                xil_iob->getPort(id_T));
        bind_in_site(site, "OUTBUF", obuf);
        xil_iob->movePortTo(id_DCITERMDISABLE, obuf, id_DCITERMDISABLE);
    }
}

void USPacker::pack_io()
{
    log_info("Inserting IO buffers..\n");

    get_top_level_pins(ctx, toplevel_ports);
    // Insert PAD cells on top level IO, and IO buffers where one doesn't exist already
    std::vector<CellInfo *> npnr_io;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type == ctx->id("$nextpnr_ibuf") || ci->type == ctx->id("$nextpnr_iobuf") ||
            ci->type == ctx->id("$nextpnr_obuf"))
            npnr_io.push_back(ci);
    }
    std::vector<std::pair<CellInfo *, PortRef>> pad_and_buf;
    for (auto ci : npnr_io)
        pad_and_buf.push_back(insert_pad_and_buf(ci));
    flush_cells();
    for (auto &iob : pad_and_buf) {
        CellInfo *pad = iob.first;
        // Process location constraints
        if (pad->attrs.count(id_PACKAGE_PIN))
            pad->attrs[id_LOC] = pad->attrs.at(id_PACKAGE_PIN);
        if (pad->attrs.count(id_LOC)) {
            std::string loc = pad->attrs.at(id_LOC).to_string();
            BelId bel = ctx->get_package_pin_bel(ctx->id(loc));
            if (bel == BelId())
                log_error("Unable to constrain IO '%s', device does not have a pin named '%s'\n", pad->name.c_str(ctx),
                          loc.c_str());
            log_info("    Constraining '%s' to pad '%s' (site %s)\n", pad->name.c_str(ctx), ctx->nameOfBel(bel),
                     uarch->get_site_name(uarch->get_bel_site(bel)).c_str(ctx));
            pad->attrs[id_BEL] = std::string(ctx->nameOfBel(bel));
        }
        if (!pad->attrs.count(id_BEL))
            log_error("FIXME: unconstrained IO not supported (pad %s)\n", ctx->nameOf(pad));
    }
    // Decompose macro IO primitives to smaller primitives that map logically to the actual IO Bels
    for (auto &iob : pad_and_buf) {
        if (packed_cells.count(iob.second.cell->name))
            continue;
        decompose_iob(iob.second.cell, str_or_default(iob.first->attrs, id_IOSTANDARD, ""));
        packed_cells.insert(iob.second.cell->name);
    }
    flush_cells();

    // Type transformations from logical to physical. The bel types are the
    // collapsed IOB_* ones that gen/filters.py gives the HDIOB/HPIOB bels.
    dict<IdString, XFormRule> io_rules;
    io_rules[id_PAD].new_type = id_IOB_PAD;
    io_rules[id_OBUF].new_type = id_IOB_OUTBUF;
    io_rules[id_OBUFT].new_type = id_IOB_OUTBUF;
    io_rules[id_OBUFT].port_xform[id_T] = id_TRI;
    io_rules[id_OBUFT_DCIEN].new_type = id_IOB_OUTBUF;
    io_rules[id_OBUFT_DCIEN].port_xform[id_T] = id_TRI;
    io_rules[id_INBUF].new_type = id_IOB_INBUF;
    io_rules[id_IBUFCTRL].new_type = id_IOB_IBUFCTRL;
    io_rules[id_IBUFCTRL].port_xform[id_T] = id_TRI;

    io_rules[id_PS8].new_type = id_PSS_ALTO_CORE;

    generic_xform(io_rules, true);
}

void USPacker::check_unsupported()
{
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type.in(id_IDDRE1, id_ODDRE1, id_ISERDESE3, id_OSERDESE3, id_IDELAYE3, id_ODELAYE3, id_IDELAYCTRL,
                        id_FIFO18E2, id_FIFO36E2, id_URAM288, id_URAM288_BASE, id_DSP48E2, id_MMCME4_ADV,
                        id_MMCME4_BASIC, id_PLLE4_ADV, id_PLLE4_BASIC, id_BUFGCE_DIV, id_BUFGCTRL))
            log_error("Cell '%s' of type %s is not supported on UltraScale+ in himbaechel yet\n", ctx->nameOf(ci),
                      ci->type.c_str(ctx));
    }
}

NEXTPNR_NAMESPACE_END
