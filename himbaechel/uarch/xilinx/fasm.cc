/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2019-2023  gatecat <gatecat@ds0.me>
 *  Copyright (C) 2023  Hans Baier <hansfbaier@gmail.com>
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

#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/reversed.hpp>

#include <fstream>
#include <regex>

#include "extra_data.h"
#include "himbaechel_api.h"
#include "log.h"
#include "nextpnr.h"
#include "pins.h"
#include "usp_drp.h"
#include "usp_dsp_rows.h"
#include "usp_uram_rows.h"
#include "util.h"

#include "xilinx.h"

#define HIMBAECHEL_CONSTIDS "uarch/xilinx/constids.inc"
#include "himbaechel_constids.h"

NEXTPNR_NAMESPACE_BEGIN

namespace Xc7MMCM {
extern const uint16_t filter_lookup_low[];
extern const uint16_t filter_lookup_low_ss[];
extern const uint16_t filter_lookup_high[];
extern const uint16_t filter_lookup_optimized[];
extern const int64_t lk_table[];
}; // namespace Xc7MMCM

namespace {
struct FasmBackend
{
    Context *ctx;
    XilinxImpl *uarch;
    std::ostream &out;
    std::vector<std::string> fasm_ctx;
    dict<int, std::vector<PipId>> pips_by_tile;
    pool<int> outputs_enabled_tiles; // INT_INTF_R_PCIE4/L_CMT/L_IO tiles with OUTPUTS_ENABLED written

    dict<std::pair<int, int>, unsigned> lut_route_throughs;

    dict<IdString, pool<IdString>> invertible_pins;

    FasmBackend(Context *ctx, XilinxImpl *uarch, std::ostream &out) : ctx(ctx), uarch(uarch), out(out) {};

    void push(const std::string &x) { fasm_ctx.push_back(x); }

    void pop() { fasm_ctx.pop_back(); }

    void pop(int N)
    {
        for (int i = 0; i < N; i++)
            fasm_ctx.pop_back();
    }
    bool last_was_blank = true;
    void blank()
    {
        if (!last_was_blank)
            out << std::endl;
        last_was_blank = true;
    }

    void write_prefix()
    {
        for (auto &x : fasm_ctx)
            out << x << ".";
        last_was_blank = false;
    }

    void write_bit(const std::string &name, bool value = true)
    {
        if (value) {
            write_prefix();
            out << name << std::endl;
        }
    }

    void write_vector(const std::string &name, const std::vector<bool> &value, bool invert = false)
    {
        write_prefix();
        out << name << " = " << int(value.size()) << "'b";
        for (auto bit : boost::adaptors::reverse(value))
            out << ((bit ^ invert) ? '1' : '0');
        out << std::endl;
    }

    void write_int_vector(const std::string &name, uint64_t value, int width, bool invert = false)
    {
        std::vector<bool> bits(width, false);
        for (int i = 0; i < width; i++)
            bits[i] = (value & (1ULL << i)) != 0;
        write_vector(name, bits, invert);
    }

    struct PseudoPipKey
    {
        IdString tileType;
        IdString dest;
        IdString source;

        bool operator==(const PseudoPipKey &b) const
        {
            return std::tie(this->tileType, this->dest, this->source) == std::tie(b.tileType, b.dest, b.source);
        }

        unsigned int hash() const { return mkhash(mkhash(tileType.hash(), source.hash()), dest.hash()); }
    };

    dict<PseudoPipKey, std::vector<std::string>> pp_config;
    void get_pseudo_pip_data()
    {
        /*
         * Create the mapping from pseudo pip tile type, dest wire, and source wire, to
         * the config bits set when that pseudo pip is used
         */
        for (std::string s : {"L", "R"})
            for (std::string s2 : {"", "_TBYTESRC", "_TBYTETERM", "_SING"})
                for (std::string i :
                     (s2 == "_SING") ? std::vector<std::string>{"", "0", "1"} : std::vector<std::string>{"0", "1"}) {
                    pp_config[{ctx->id(s + "IOI3" + s2), ctx->id(s + "IOI_OLOGIC" + i + "_OQ"),
                               ctx->id("IOI_OLOGIC" + i + "_D1")}] = {"OLOGIC_Y" + i + ".OMUX.D1",
                                                                      "OLOGIC_Y" + i + ".OQUSED",
                                                                      "OLOGIC_Y" + i + ".OSERDES.DATA_RATE_TQ.BUF"};
                    pp_config[{ctx->id(s + "IOI3" + s2), ctx->id("IOI_ILOGIC" + i + "_O"),
                               ctx->id(s + "IOI_ILOGIC" + i + "_D")}] = {"IDELAY_Y" + i + ".IDELAY_TYPE_FIXED",
                                                                         "ILOGIC_Y" + i + ".ZINV_D"};
                    pp_config[{ctx->id(s + "IOI3" + s2), ctx->id("IOI_ILOGIC" + i + "_O"),
                               ctx->id(s + "IOI_ILOGIC" + i + "_DDLY")}] = {"ILOGIC_Y" + i + ".IDELMUXE3.P0",
                                                                            "ILOGIC_Y" + i + ".ZINV_D"};
                    pp_config[{ctx->id(s + "IOI3" + s2), ctx->id(s + "IOI_OLOGIC" + i + "_TQ"),
                               ctx->id("IOI_OLOGIC" + i + "_T1")}] = {"OLOGIC_Y" + i + ".ZINV_T1"};
                    if (i == "0") {
                        pp_config[{ctx->id(s + "IOB33" + s2), id_IOB_O_IN1, id_IOB_O_OUT0}] = {};
                        pp_config[{ctx->id(s + "IOB33" + s2), id_IOB_O_OUT0, id_IOB_O0}] = {};
                        pp_config[{ctx->id(s + "IOB33" + s2), id_IOB_T_IN1, id_IOB_T_OUT0}] = {};
                        pp_config[{ctx->id(s + "IOB33" + s2), id_IOB_T_OUT0, id_IOB_T0}] = {};
                        pp_config[{ctx->id(s + "IOB33" + s2), id_IOB_DIFFI_IN0, id_IOB_PADOUT1}] = {};
                    }
                }

        for (std::string s2 : {"", "_TBYTESRC", "_TBYTETERM", "_SING"})
            for (std::string i : (s2 == "_SING") ? std::vector<std::string>{"0"} : std::vector<std::string>{"0", "1"}) {
                pp_config[{ctx->id("RIOI" + s2), ctx->id("RIOI_OLOGIC" + i + "_OQ"),
                           ctx->id("IOI_OLOGIC" + i + "_D1")}] = {"OLOGIC_Y" + i + ".OMUX.D1",
                                                                  "OLOGIC_Y" + i + ".OQUSED",
                                                                  "OLOGIC_Y" + i + ".OSERDES.DATA_RATE_TQ.BUF"};
                pp_config[{ctx->id("RIOI" + s2), ctx->id("RIOI_OLOGIC" + i + "_OFB"),
                           ctx->id("RIOI_OLOGIC" + i + "_OQ")}] = {};
                pp_config[{ctx->id("RIOI" + s2), ctx->id("RIOI_O" + i), ctx->id("RIOI_ODELAY" + i + "_DATAOUT")}] = {};
                pp_config[{ctx->id("RIOI" + s2), ctx->id("RIOI_OLOGIC" + i + "_OFB"),
                           ctx->id("IOI_OLOGIC" + i + "_D1")}] = {"OLOGIC_Y" + i + ".OMUX.D1",
                                                                  "OLOGIC_Y" + i + ".OSERDES.DATA_RATE_TQ.BUF"};
                pp_config[{ctx->id("RIOI" + s2), ctx->id("IOI_ILOGIC" + i + "_O"), ctx->id("RIOI_ILOGIC" + i + "_D")}] =
                        {"ILOGIC_Y" + i + ".ZINV_D"};
                pp_config[{ctx->id("RIOI" + s2), ctx->id("IOI_ILOGIC" + i + "_O"),
                           ctx->id("RIOI_ILOGIC" + i + "_DDLY")}] = {"ILOGIC_Y" + i + ".IDELMUXE3.P0",
                                                                     "ILOGIC_Y" + i + ".ZINV_D"};
                pp_config[{ctx->id("RIOI" + s2), ctx->id("RIOI_OLOGIC" + i + "_TQ"),
                           ctx->id("IOI_OLOGIC" + i + "_T1")}] = {"OLOGIC_Y" + i + ".ZINV_T1"};
                pp_config[{ctx->id("RIOI" + s2), ctx->id("RIOI_OLOGIC" + i + "_OFB"),
                           ctx->id("RIOI_ODELAY" + i + "_ODATAIN")}] = {"OLOGIC_Y" + i + ".ZINV_ODATAIN"};
                if (i == "0") {
                    pp_config[{ctx->id("RIOB18" + s2), id_IOB_O_IN1, id_IOB_O_OUT0}] = {};
                    pp_config[{ctx->id("RIOB18" + s2), id_IOB_O_OUT0, id_IOB_O0}] = {};
                    pp_config[{ctx->id("RIOB18" + s2), id_IOB_T_IN1, id_IOB_T_OUT0}] = {};
                    pp_config[{ctx->id("RIOB18" + s2), id_IOB_T_OUT0, id_IOB_T0}] = {};
                    pp_config[{ctx->id("RIOB18" + s2), id_IOB_DIFFI_IN0, id_IOB_PADOUT1}] = {};
                }
            }

        for (std::string s1 : {"TOP", "BOT"}) {
            for (std::string s2 : {"L", "R"}) {
                for (int i = 0; i < 12; i++) {
                    std::string ii = std::to_string(i);
                    std::string hck = s2 + ii;
                    std::string buf = std::string((s2 == "R") ? "X1Y" : "X0Y") + ii;
                    pp_config[{ctx->id("CLK_HROW_" + s1 + "_R"), ctx->id("CLK_HROW_CK_HCLK_OUT_" + hck),
                               ctx->id("CLK_HROW_CK_MUX_OUT_" + hck)}] = {"BUFHCE.BUFHCE_" + buf + ".IN_USE",
                                                                          "BUFHCE.BUFHCE_" + buf + ".ZINV_CE"};
                }
            }

            for (int i = 0; i < 16; i++) {
                std::string ii = std::to_string(i);
                pp_config[{ctx->id("CLK_BUFG_" + s1 + "_R"), ctx->id("CLK_BUFG_BUFGCTRL" + ii + "_O"),
                           ctx->id("CLK_BUFG_BUFGCTRL" + ii + "_I0")}] = {
                        "BUFGCTRL.BUFGCTRL_X0Y" + ii + ".IN_USE", "BUFGCTRL.BUFGCTRL_X0Y" + ii + ".IS_IGNORE1_INVERTED",
                        "BUFGCTRL.BUFGCTRL_X0Y" + ii + ".ZINV_CE0", "BUFGCTRL.BUFGCTRL_X0Y" + ii + ".ZINV_S0"};
                pp_config[{ctx->id("CLK_BUFG_" + s1 + "_R"), ctx->id("CLK_BUFG_BUFGCTRL" + ii + "_O"),
                           ctx->id("CLK_BUFG_BUFGCTRL" + ii + "_I1")}] = {
                        "BUFGCTRL.BUFGCTRL_X0Y" + ii + ".IN_USE", "BUFGCTRL.BUFGCTRL_X0Y" + ii + ".IS_IGNORE0_INVERTED",
                        "BUFGCTRL.BUFGCTRL_X0Y" + ii + ".ZINV_CE1", "BUFGCTRL.BUFGCTRL_X0Y" + ii + ".ZINV_S1"};
            }
        }

        int rclk_y_to_i[4] = {2, 3, 0, 1};
        for (int y = 0; y < 4; y++) {
            std::string yy = std::to_string(y);
            std::string ii = std::to_string(rclk_y_to_i[y]);
            pp_config[{id_HCLK_IOI3, ctx->id("HCLK_IOI_RCLK_OUT" + ii), ctx->id("HCLK_IOI_RCLK_BEFORE_DIV" + ii)}] = {
                    "BUFR_Y" + yy + ".IN_USE", "BUFR_Y" + yy + ".BUFR_DIVIDE.BYPASS"};
            pp_config[{id_HCLK_IOI, ctx->id("HCLK_IOI_RCLK_OUT" + ii), ctx->id("HCLK_IOI_RCLK_BEFORE_DIV" + ii)}] = {
                    "BUFR_Y" + yy + ".IN_USE", "BUFR_Y" + yy + ".BUFR_DIVIDE.BYPASS"};
        }

        // FIXME: shouldn't these be in the X-RAY ppips database?
        for (char c : {'L', 'R'}) {
            for (int i = 0; i < 24; i++) {
                pp_config[{ctx->idf("INT_INTERFACE_%c", c), ctx->idf("INT_INTERFACE_LOGIC_OUTS_%c%d", c, i),
                           ctx->idf("INT_INTERFACE_LOGIC_OUTS_%c_B%d", c, i)}];
            }
        }
    }

    void write_pip(PipId pip, NetInfo *net)
    {

        pips_by_tile[pip.tile].push_back(pip);

        auto dst_intent = ctx->getWireType(ctx->getPipDstWire(pip));
        if (dst_intent == id_PSEUDO_GND || dst_intent == id_PSEUDO_VCC)
            return;

        auto &pd = chip_pip_info(ctx->chip_info, pip);
        const auto &extra_data = *reinterpret_cast<const XlnxPipExtraDataPOD *>(pd.extra_data.get());
        unsigned pip_type = pd.flags;

        if (pip_type != PIP_TILE_ROUTING && pip_type != PIP_SITE_INTERNAL)
            return;

        IdString src = IdString(chip_tile_info(ctx->chip_info, pip.tile).wires[pd.src_wire].name);
        IdString dst = IdString(chip_tile_info(ctx->chip_info, pip.tile).wires[pd.dst_wire].name);

        // handle certain site internal pips:
        // this is necessary, because in tristate outputs, the
        // ZINV_T1 bit needs to be set, because in the OLOGIC tiles the
        // tristate control signals are inverted if this bit is not set
        // this only applies to router1, because router2 does not generate
        // site internal pips here.
        if (pip_type == PIP_SITE_INTERNAL) {
            if (src.str(ctx) == "T1" && dst.str(ctx) == "T1INV_OUT") {
                auto srcwire_uphill_iter = ctx->getPipsUphill(ctx->getPipSrcWire(pip));
                auto uphill = srcwire_uphill_iter.begin();
                if (uphill != srcwire_uphill_iter.end()) {
                    // source wire should be like: LIOI3_X0Y73/IOI_OLOGIC1_T1
                    auto loc = ctx->getWireName(ctx->getPipSrcWire(*uphill)).str(ctx);
                    boost::replace_all(loc, "/", ".");
                    boost::erase_all(loc, "_T1");
                    boost::replace_all(loc, "IOI_OLOGIC", "OLOGIC_Y");
                    // the replacements transformed it into : LIOI3_X0Y73.OLOGIC_Y1
                    out << loc << "." << "ZINV_T1" << std::endl;
                }
            }
            return;
        }

        // handle tile routing pips
        IdString tile_type = IdString(chip_tile_info(ctx->chip_info, pip.tile).type_name);
        PseudoPipKey ppk{tile_type, dst, src};

        if (pp_config.count(ppk)) {
            auto &pp = pp_config.at(ppk);
            std::string tile_name = uarch->tile_name(pip.tile);
            for (auto c : pp) {
                if (boost::starts_with(tile_name, "RIOI3_SING") || boost::starts_with(tile_name, "LIOI3_SING") ||
                    boost::starts_with(tile_name, "RIOI_SING")) {
                    // Need to flip for top HCLK
                    bool is_top_sing = pip.tile < uarch->hclk_for_ioi(pip.tile);
                    if (is_top_sing) {
                        auto y0pos = c.find("Y0");
                        if (y0pos != std::string::npos)
                            c.replace(y0pos, 2, "Y1");
                    }
                }
                out << tile_name << "." << c << std::endl;
            }
            if (!pp.empty())
                last_was_blank = false;
        } else {
            if (uarch->usp) {
                write_pip_usp(pip, src.str(ctx), dst.str(ctx));
                return;
            }
            if (extra_data.pip_config & 1)
                log_warning("Unprocessed route-thru %s.%s.%s\n!", tile_type.c_str(ctx), src.c_str(ctx), dst.c_str(ctx));

            std::string tile_name = uarch->tile_name(pip.tile);
            std::string dst_name = dst.str(ctx);
            std::string src_name = src.str(ctx);

            if (boost::starts_with(tile_name, "DSP_L") || boost::starts_with(tile_name, "DSP_R")) {
                // FIXME: PPIPs missing for DSPs
                return;
            }
            std::string orig_dst_name = dst_name;
            if (boost::starts_with(tile_name, "RIOI3_SING") || boost::starts_with(tile_name, "LIOI3_SING") ||
                boost::starts_with(tile_name, "RIOI_SING")) {
                // FIXME: PPIPs missing for SING IOI3s
                if ((boost::contains(src_name, "IMUX") || boost::contains(src_name, "CTRL0")) &&
                    !boost::contains(dst_name, "CLK"))
                    return;
                auto spos = src_name.find("_SING_");
                if (spos != std::string::npos)
                    src_name.erase(spos, 5);
                // Need to flip for top HCLK
                // TODO

                bool is_top_sing = pip.tile < uarch->hclk_for_ioi(pip.tile);
                if (is_top_sing) {
                    auto us0pos = dst_name.find("_0");
                    if (us0pos != std::string::npos)
                        dst_name.replace(us0pos, 2, "_1");
                    auto ol0pos = dst_name.find("OLOGIC0");
                    if (ol0pos != std::string::npos) {
                        dst_name.replace(ol0pos, 7, "OLOGIC1");
                        us0pos = src_name.find("_0");
                        if (us0pos != std::string::npos)
                            src_name.replace(us0pos, 2, "_1");
                    }
                }
            }
            if (boost::contains(tile_name, "IOI")) {
                if (boost::contains(dst_name, "OCLKB") && boost::contains(src_name, "IOI_OCLKM_"))
                    return; // missing, not sure if really a ppip?
            }

            out << tile_name << ".";
            out << dst_name << ".";
            out << src_name << std::endl;

            if (boost::contains(tile_name, "IOI") && boost::starts_with(dst_name, "IOI_OCLK_")) {
                dst_name.insert(dst_name.find("OCLK") + 4, 1, 'M');
                orig_dst_name.insert(dst_name.find("OCLK") + 4, 1, 'M');

                WireId w = uarch->lookup_wire(pip.tile, ctx->id(orig_dst_name));

                NPNR_ASSERT(w != WireId());
                if (ctx->getBoundWireNet(w) == nullptr) {
                    out << tile_name << ".";
                    out << dst_name << ".";
                    out << src_name << std::endl;
                }
            }

            last_was_blank = false;
        }
    };

    // Get the set of input signals for a LUT-type cell
    std::vector<IdString> get_inputs(CellInfo *cell)
    {
        IdString type = ctx->id(str_or_default(cell->attrs, id_X_ORIG_TYPE, ""));
        if (type == id_LUT1)
            return {id_I0};
        else if (type == id_LUT2)
            return {id_I0, id_I1};
        else if (type == id_LUT3)
            return {id_I0, id_I1, id_I2};
        else if (type == id_LUT4)
            return {id_I0, id_I1, id_I2, id_I3};
        else if (type == id_LUT5)
            return {id_I0, id_I1, id_I2, id_I3, id_I4};
        else if (type == id_LUT6)
            return {id_I0, id_I1, id_I2, id_I3, id_I4, id_I5};
        else if (type == id_RAMD64E)
            return {id_RADR0, id_RADR1, id_RADR2, id_RADR3, id_RADR4, id_RADR5};
        else if (type == id_SRL16E)
            return {id_A0, id_A1, id_A2, id_A3};
        else if (type == id_SRLC32E)
            return {ctx->id("A[0]"), ctx->id("A[1]"), ctx->id("A[2]"), ctx->id("A[3]"), ctx->id("A[4]")};
        else if (type == id_RAMD32)
            return {id_RADR0, id_RADR1, id_RADR2, id_RADR3, id_RADR4};
        else
            NPNR_ASSERT_FALSE("unsupported LUT-type cell");
    }

    // Process LUT initialisation
    std::vector<bool> get_lut_init(CellInfo *lut6, CellInfo *lut5)
    {
        std::vector<bool> bits(64, false);

        std::vector<IdString> phys_inputs;
        for (int i = 1; i <= 6; i++)
            phys_inputs.push_back(ctx->id("A" + std::to_string(i)));

        for (int i = 0; i < 2; i++) {
            CellInfo *lut = (i == 1) ? lut5 : lut6;
            if (lut == nullptr)
                continue;
            auto lut_inputs = get_inputs(lut);
            dict<int, std::vector<std::string>> phys_to_log;
            dict<std::string, int> log_to_bit;
            for (int j = 0; j < int(lut_inputs.size()); j++)
                log_to_bit[lut_inputs[j].str(ctx)] = j;
            for (int j = 0; j < 6; j++) {
                // Get the LUT physical to logical mapping
                phys_to_log[j];
                if (!lut->attrs.count(ctx->idf("X_ORIG_PORT_%s", phys_inputs[j].c_str(ctx))))
                    continue;
                std::string orig = lut->attrs.at(ctx->idf("X_ORIG_PORT_%s", phys_inputs[j].c_str(ctx))).as_string();
                boost::split(phys_to_log[j], orig, boost::is_any_of(" "));
            }
            int lbound = 0, ubound = 64;
            // Fracturable LUTs
            if (lut5 && lut6) {
                lbound = (i == 1) ? 0 : 32;
                ubound = (i == 1) ? 32 : 64;
            }
            Property init = get_or_default(lut->params, id_INIT, Property()).extract(0, 64);
            for (int j = lbound; j < ubound; j++) {
                int log_index = 0;
                for (int k = 0; k < 6; k++) {
                    if ((j & (1 << k)) == 0)
                        continue;
                    for (auto &p2l : phys_to_log[k])
                        log_index |= (1 << log_to_bit[p2l]);
                }
                bits[j] = (init.str.at(log_index) == Property::S1);
            }
        }
        return bits;
    };

    // Return the name for a half-logic-tile
    std::string get_half_name(int half, bool is_m)
    {
        if (is_m)
            return half ? "SLICEL_X1" : "SLICEM_X0";
        else
            return half ? "SLICEL_X1" : "SLICEL_X0";
    }

    std::string get_bel_name(BelId bel) { return uarch->bel_name_in_site(bel).str(ctx); }

    void write_routing_bel(WireId dst_wire)
    {
        for (auto pip : ctx->getPipsUphill(dst_wire)) {
            if (ctx->getBoundPipNet(pip) != nullptr) {
                auto &pd = chip_pip_info(ctx->chip_info, pip);
                const auto &extra_data = *reinterpret_cast<const XlnxPipExtraDataPOD *>(pd.extra_data.get());
                std::string belname = IdString(extra_data.bel_name).str(ctx);
                std::string pinname = IdString(extra_data.pip_config).str(ctx);
                bool skip_pinname = false;
                // Ignore modes with no associated bit (X-ray omission??)
                if (belname == "WEMUX" && pinname == "WE")
                    continue;

                if (belname.substr(1) == "DI1MUX") {
                    belname = "DI1MUX";
                }

                if (belname.substr(1) == "CY0") {
                    if (pinname.substr(1) == "5")
                        skip_pinname = true;
                    else
                        continue;
                }

                write_prefix();
                out << belname;
                if (!skip_pinname)
                    out << "." << pinname;
                out << std::endl;
            }
        }
    }

    // Process flipflops in a half-tile
    void write_ffs_config(int tile, int half)
    {
        bool found_ff = false;
        bool negedge_ff = false;
        bool is_latch = false;
        bool is_sync = false;
        bool is_clkinv = false;
        bool is_srused = false;
        bool is_ceused = false;

#define SET_CHECK(dst, src)                                                                                            \
    do {                                                                                                               \
        if (found_ff)                                                                                                  \
            NPNR_ASSERT(dst == (src));                                                                                 \
        else                                                                                                           \
            dst = (src);                                                                                               \
    } while (0)

        std::string tname = uarch->tile_name(tile);

        const auto &lts = uarch->tile_status.at(tile).lts;
        if (!lts)
            return;

        push(tname);
        push(get_half_name(half, boost::contains(tname, "CLBLM")));

        for (int i = 0; i < 4; i++) {
            CellInfo *ff1 = lts->cells[(half << 6) | (i << 4) | BEL_FF];
            CellInfo *ff2 = lts->cells[(half << 6) | (i << 4) | BEL_FF2];
            for (int j = 0; j < 2; j++) {
                CellInfo *ff = (j == 1) ? ff2 : ff1;
                if (ff == nullptr)
                    continue;
                push(get_bel_name(ff->bel));
                bool zrst = false, zinit = false;
                zinit = (int_or_default(ff->params, id_INIT, 0) != 1);
                IdString srsig;
                std::string type = str_or_default(ff->attrs, id_X_ORIG_TYPE, "");
                if (type == "FDRE") {
                    zrst = true;
                    SET_CHECK(negedge_ff, false);
                    SET_CHECK(is_latch, false);
                    SET_CHECK(is_sync, true);
                } else if (type == "FDRE_1") {
                    zrst = true;
                    SET_CHECK(negedge_ff, true);
                    SET_CHECK(is_latch, false);
                    SET_CHECK(is_sync, true);
                } else if (type == "FDSE") {
                    zrst = false;
                    SET_CHECK(negedge_ff, false);
                    SET_CHECK(is_latch, false);
                    SET_CHECK(is_sync, true);
                } else if (type == "FDSE_1") {
                    zrst = false;
                    SET_CHECK(negedge_ff, true);
                    SET_CHECK(is_latch, false);
                    SET_CHECK(is_sync, true);
                } else if (type == "FDCE") {
                    zrst = true;
                    SET_CHECK(negedge_ff, false);
                    SET_CHECK(is_latch, false);
                    SET_CHECK(is_sync, false);
                } else if (type == "FDCE_1") {
                    zrst = true;
                    SET_CHECK(negedge_ff, true);
                    SET_CHECK(is_latch, false);
                    SET_CHECK(is_sync, false);
                } else if (type == "FDPE") {
                    zrst = false;
                    SET_CHECK(negedge_ff, false);
                    SET_CHECK(is_latch, false);
                    SET_CHECK(is_sync, false);
                } else if (type == "FDPE_1") {
                    zrst = false;
                    SET_CHECK(negedge_ff, true);
                    SET_CHECK(is_latch, false);
                    SET_CHECK(is_sync, false);
                } else {
                    log_error("unsupported FF type: '%s'\n", type.c_str());
                }

                write_bit("ZINI", zinit);
                write_bit("ZRST", zrst);

                pop();
                if (negedge_ff)
                    SET_CHECK(is_clkinv, true);
                else
                    SET_CHECK(is_clkinv, int_or_default(ff->params, id_IS_C_INVERTED) == 1);

                NetInfo *sr = ff->getPort(id_SR), *ce = ff->getPort(id_CE);

                SET_CHECK(is_srused, sr != nullptr && sr->name != ctx->id("$PACKER_GND_NET"));
                SET_CHECK(is_ceused, ce != nullptr && ce->name != ctx->id("$PACKER_VCC_NET"));

                // Input mux
                write_routing_bel(ctx->getBelPinWire(ff->bel, id_D));

                found_ff = true;
            }
        }
        write_bit("LATCH", is_latch);
        write_bit("FFSYNC", is_sync);
        write_bit("CLKINV", is_clkinv);
        write_bit("NOCLKINV", !is_clkinv);
        write_bit("SRUSEDMUX", is_srused);
        write_bit("CEUSEDMUX", is_ceused);
        pop(2);
    }

    // Get a named wire in the same site as a bel
    WireId get_site_wire(BelId site_bel, std::string name)
    {
        IdStringList bel_name = ctx->getBelName(site_bel);
        NPNR_ASSERT(bel_name.size() == 2);
        IdString tile_name = bel_name[0];
        const std::string &bel_name_str = bel_name[1].str(ctx);
        size_t sep_pos = bel_name_str.find('.');
        NPNR_ASSERT(sep_pos != std::string::npos);
        std::string site_name = bel_name_str.substr(0, sep_pos);
        IdString wire_name = ctx->idf("%s.%s", site_name.c_str(), name.c_str());
        WireId wire = ctx->getWireByName(IdStringList::concat(tile_name, wire_name));
        NPNR_ASSERT(wire != WireId());
        return wire;
    }

    // Process LUTs and associated functionality in a half
    void write_luts_config(int tile, int half)
    {
        bool wa7_used = false, wa8_used = false;

        std::string tname = uarch->tile_name(tile);
        bool is_mtile = boost::contains(tname, "CLBLM");
        bool is_slicem = is_mtile && (half == 0);

        const auto &lts = uarch->tile_status.at(tile).lts;

        push(tname);
        push(get_half_name(half, is_mtile));

        // Write route through pips
        for (int i = 0; i < 4; i++) {
            auto found_rt = lut_route_throughs.find(std::make_pair(tile, half * 4 + i));
            if (found_rt != lut_route_throughs.end()) {
                std::string lutname = stringf("%cLUT", "ABCD"[i]);
                push(lutname);
                std::vector<bool> rt_init(64, false);
                for (unsigned b = 0; b < 64; b++) {
                    if (b & (1U << found_rt->second))
                        rt_init[b] = true;
                }
                write_vector("INIT[63:0]", rt_init);
                pop();
            }
        }
        if (lts) {
            // Write logic
            BelId bel_in_half =
                    ctx->getBelByLocation(Loc(tile % ctx->chip_info->width, tile / ctx->chip_info->width, half << 6));

            for (int i = 0; i < 4; i++) {
                CellInfo *lut6 = lts->cells[(half << 6) | (i << 4) | BEL_6LUT];
                CellInfo *lut5 = lts->cells[(half << 6) | (i << 4) | BEL_5LUT];
                // Write LUT initialisation
                if (lut6 != nullptr || lut5 != nullptr) {
                    std::string lutname = stringf("%cLUT", "ABCD"[i]);
                    push(lutname);
                    write_vector("INIT[63:0]", get_lut_init(lut6, lut5));

                    // Write LUT mode config
                    bool is_small = false, is_ram = false, is_srl = false;
                    for (int j = 0; j < 2; j++) {
                        CellInfo *lut = (j == 1) ? lut5 : lut6;
                        if (lut == nullptr)
                            continue;
                        std::string type = str_or_default(lut->attrs, id_X_ORIG_TYPE);
                        if (type == "RAMD64E" || type == "RAMS64E") {
                            is_ram = true;
                        } else if (type == "RAMD32" || type == "RAMS32") {
                            is_ram = true;
                            is_small = true;
                        } else if (type == "SRL16E") {
                            is_srl = true;
                            is_small = true;
                        } else if (type == "SRLC32E") {
                            is_srl = true;
                        }
                        wa7_used |= (lut->getPort(id_WA7) != nullptr);
                        wa8_used |= (lut->getPort(id_WA8) != nullptr);
                    }
                    if (is_slicem && i != 3) {
                        write_routing_bel(get_site_wire(bel_in_half, stringf("%cDI1MUX_OUT", "ABCD"[i])));
                    }
                    write_bit("SMALL", is_small);
                    write_bit("RAM", is_ram);
                    write_bit("SRL", is_srl);
                    pop();
                }
                write_routing_bel(get_site_wire(bel_in_half, stringf("%cMUX", "ABCD"[i])));
            }
            write_bit("WA7USED", wa7_used);
            write_bit("WA8USED", wa8_used);
            if (is_slicem)
                write_routing_bel(get_site_wire(bel_in_half, "WEMUX_OUT"));
        }

        pop(2);
    }

    void write_carry_config(int tile, int half)
    {
        std::string tname = uarch->tile_name(tile);
        bool is_mtile = boost::contains(tname, "CLBLM");

        const auto &lts = uarch->tile_status.at(tile).lts;
        if (!lts)
            return;

        CellInfo *carry = lts->cells[half << 6 | BEL_CARRY4];
        if (carry == nullptr)
            return;

        push(tname);
        push(get_half_name(half, is_mtile));

        write_routing_bel(get_site_wire(carry->bel, "PRECYINIT_OUT"));
        if (carry->getPort(id_CIN) != nullptr)
            write_bit("PRECYINIT.CIN");
        push("CARRY4");
        for (char c : {'A', 'B', 'C', 'D'})
            write_routing_bel(get_site_wire(carry->bel, stringf("%cCY0_OUT", c)));
        pop(3);
    }

    void write_logic()
    {
        std::set<int> used_logic_tiles;
        for (auto &cell : ctx->cells) {
            if (uarch->is_logic_tile(cell.second->bel))
                used_logic_tiles.insert(cell.second->bel.tile);
        }
        for (auto &net : ctx->nets) {
            for (const auto &wire_pair : net.second->wires) {
                PipId pip = wire_pair.second.pip;
                if (pip == PipId())
                    continue;
                const auto &pip_data = chip_pip_info(ctx->chip_info, pip);
                const auto &extra_data = *reinterpret_cast<const XlnxPipExtraDataPOD *>(pip_data.extra_data.get());
                unsigned pip_type = pip_data.flags;
                if (pip_type != PIP_LUT_ROUTETHRU)
                    continue;
                unsigned lut_idx = (extra_data.pip_config >> 8);
                unsigned lut_input = (extra_data.pip_config >> 1) & 0x7;
                lut_route_throughs[std::make_pair(pip.tile, lut_idx)] = lut_input;
                used_logic_tiles.insert(pip.tile);
            }
        }
        for (int tile : used_logic_tiles) {
            if (uarch->usp) {
                write_luts_config_usp(tile);
                write_ffs_config_usp(tile);
                write_carry_config_usp(tile);
                blank();
                continue;
            }
            write_luts_config(tile, 0);
            write_luts_config(tile, 1);
            write_ffs_config(tile, 0);
            write_ffs_config(tile, 1);
            write_carry_config(tile, 0);
            write_carry_config(tile, 1);
            blank();
        }
    }

    void write_routing()
    {
        get_pseudo_pip_data();
        for (auto &net : ctx->nets) {
            NetInfo *ni = net.second.get();
            out << stringf("# routing for net %s", ni->name.c_str(ctx)) << std::endl;
            for (auto &w : ni->wires) {
                if (w.second.pip != PipId())
                    write_pip(w.second.pip, ni);
            }
            blank();
        }
    }

    struct BankIoConfig
    {
        bool stepdown = false;
        bool vref = false;
        bool tmds_33 = false;
        bool lvds_25 = false;
        bool only_diff = false;
    };

    dict<int, BankIoConfig> ioconfig_by_hclk;

    bool warned_dci = false;

    void write_io_config(CellInfo *pad)
    {
        NetInfo *pad_net = pad->getPort(id_PAD);
        NPNR_ASSERT(pad_net != nullptr);
        std::string iostandard = str_or_default(pad->attrs, id_IOSTANDARD, "LVCMOS33");
        std::string pulltype = str_or_default(pad->attrs, id_PULLTYPE, "NONE");
        std::string slew = str_or_default(pad->attrs, id_SLEW, "SLOW");

        Loc ioLoc = uarch->rel_site_loc(uarch->get_bel_site(pad->bel));
        bool is_output = false, is_input = false;
        if (pad_net->driver.cell != nullptr)
            is_output = true;
        for (auto &usr : pad_net->users)
            if (boost::contains(usr.cell->type.str(ctx), "INBUF"))
                is_input = true;
        std::string tile = uarch->tile_name(pad->bel.tile);
        push(tile);

        if (boost::ends_with(iostandard, "_T_DCI")) {
            if (!warned_dci)
                log_warning("DCI is not supported, will be removed.\n");
            warned_dci = true;
            iostandard.erase(iostandard.size() - 6, iostandard.size());
        }

        bool is_riob18 = boost::starts_with(tile, "RIOB18_");
        bool is_sing = boost::contains(tile, "_SING_");
        bool is_top_sing = pad->bel.tile < uarch->hclk_for_iob(pad->bel);
        bool is_stepdown = false;
        bool is_lvcmos = boost::starts_with(iostandard, "LVCMOS");
        bool is_low_volt_lvcmos = iostandard == "LVCMOS12" || iostandard == "LVCMOS15" || iostandard == "LVCMOS18";

        auto yLoc = is_sing ? (is_top_sing ? 1 : 0) : (1 - ioLoc.y);
        push("IOB_Y" + std::to_string(yLoc));

        bool has_diff_prefix = boost::starts_with(iostandard, "DIFF_");
        bool is_tmds33 = iostandard == "TMDS_33";
        bool is_lvds25 = iostandard == "LVDS_25";
        bool is_lvds = boost::starts_with(iostandard, "LVDS");
        bool only_diff = is_tmds33 || is_lvds;
        bool is_diff = only_diff || has_diff_prefix;
        if (has_diff_prefix)
            iostandard.erase(0, 5);
        bool is_sstl = iostandard == "SSTL12" || iostandard == "SSTL135" || iostandard == "SSTL15";

        int hclk = uarch->hclk_for_iob(pad->bel);

        if (only_diff)
            ioconfig_by_hclk[hclk].only_diff = true;
        if (is_tmds33)
            ioconfig_by_hclk[hclk].tmds_33 = true;
        if (is_lvds25)
            ioconfig_by_hclk[hclk].lvds_25 = true;

        if (is_output) {
            // DRIVE
            int default_drive = (is_riob18 && iostandard == "LVCMOS12") ? 8 : 12;
            int drive = int_or_default(pad->attrs, id_DRIVE, default_drive);

            if ((iostandard == "LVCMOS33" || iostandard == "LVTTL") && is_riob18)
                log_error("high performance banks (RIOB18) do not support IO standard %s\n", iostandard.c_str());

            if (iostandard == "SSTL135")
                write_bit("SSTL135.DRIVE.I_FIXED");
            else if (is_riob18) {
                if ((iostandard == "LVCMOS18" || iostandard == "LVCMOS15"))
                    write_bit("LVCMOS15_LVCMOS18.DRIVE.I12_I16_I2_I4_I6_I8");
                else if (iostandard == "LVCMOS12")
                    write_bit("LVCMOS12.DRIVE.I2_I4_I6_I8");
                else if (iostandard == "LVDS")
                    write_bit("LVDS.DRIVE.I_FIXED");
                else if (is_sstl) {
                    write_bit(iostandard + ".DRIVE.I_FIXED");
                }
            } else { // IOB33
                if (iostandard == "TMDS_33" && yLoc == 0) {
                    write_bit("TMDS_33.DRIVE.I_FIXED");
                    write_bit("TMDS_33.OUT");
                } else if (iostandard == "LVDS_25" && yLoc == 0) {
                    write_bit("LVDS_25.DRIVE.I_FIXED");
                    write_bit("LVDS_25.OUT");
                } else if ((iostandard == "LVCMOS15" && drive == 16) || iostandard == "SSTL15")
                    write_bit("LVCMOS15_SSTL15.DRIVE.I16_I_FIXED");
                else if (iostandard == "LVCMOS18" && (drive == 12 || drive == 8))
                    write_bit("LVCMOS18.DRIVE.I12_I8");
                else if ((iostandard == "LVCMOS33" && drive == 16) || (iostandard == "LVTTL" && drive == 16))
                    write_bit("LVCMOS33_LVTTL.DRIVE.I12_I16");
                else if ((iostandard == "LVCMOS33" && (drive == 8 || drive == 12)) ||
                         (iostandard == "LVTTL" && (drive == 8 || drive == 12)))
                    write_bit("LVCMOS33_LVTTL.DRIVE.I12_I8");
                else if ((iostandard == "LVCMOS33" && drive == 4) || (iostandard == "LVTTL" && drive == 4))
                    write_bit("LVCMOS33_LVTTL.DRIVE.I4");
                else if (drive == 8 && (iostandard == "LVCMOS12" || iostandard == "LVCMOS25"))
                    write_bit("LVCMOS12_LVCMOS25.DRIVE.I8");
                else if (drive == 4 &&
                         (iostandard == "LVCMOS15" || iostandard == "LVCMOS18" || iostandard == "LVCMOS25"))
                    write_bit("LVCMOS15_LVCMOS18_LVCMOS25.DRIVE.I4");
                else if (is_lvcmos || iostandard == "LVTTL")
                    write_bit(iostandard + ".DRIVE.I" + std::to_string(drive));
            }

            // SSTL output used
            if (is_riob18 && is_sstl)
                write_bit(iostandard + ".IN_USE");

            // SLEW
            if (is_riob18 && slew == "SLOW") {
                if (iostandard == "SSTL135")
                    write_bit("SSTL135.SLEW.SLOW");
                else if (iostandard == "SSTL15")
                    write_bit("SSTL15.SLEW.SLOW");
                else
                    write_bit("LVCMOS12_LVCMOS15_LVCMOS18.SLEW.SLOW");
            } else if (slew == "SLOW") {
                if (iostandard != "LVDS_25" && iostandard != "TMDS_33")
                    write_bit("LVCMOS12_LVCMOS15_LVCMOS18_LVCMOS25_LVCMOS33_LVTTL_SSTL135_SSTL15.SLEW.SLOW");
            } else if (is_riob18)
                write_bit(iostandard + ".SLEW.FAST");
            else if (iostandard == "SSTL135" || iostandard == "SSTL15")
                write_bit("SSTL135_SSTL15.SLEW.FAST");
            else
                write_bit("LVCMOS12_LVCMOS15_LVCMOS18_LVCMOS25_LVCMOS33_LVTTL.SLEW.FAST");
        }

        if (is_input) {
            if (!is_diff) {
                if (iostandard == "LVCMOS33" || iostandard == "LVTTL" || iostandard == "LVCMOS25") {
                    if (!is_riob18)
                        write_bit("LVCMOS25_LVCMOS33_LVTTL.IN");
                    else
                        log_error("high performance banks (RIOB18) do not support IO standard %s\n",
                                  iostandard.c_str());
                }

                if (is_sstl) {
                    ioconfig_by_hclk[hclk].vref = true;
                    if (!is_riob18)
                        write_bit("SSTL135_SSTL15.IN");

                    if (is_riob18) {
                        write_bit("SSTL12_SSTL135_SSTL15.IN");
                    }

                    if (!is_riob18 && pad->attrs.count(id_IN_TERM))
                        write_bit("IN_TERM." + pad->attrs.at(id_IN_TERM).as_string());
                }

                if (is_low_volt_lvcmos) {
                    write_bit("LVCMOS12_LVCMOS15_LVCMOS18.IN");
                }
            } else /* is_diff */ {
                if (is_riob18) {
                    // vivado generates these bits only for Y0 of a diff pair
                    if (yLoc == 0) {
                        write_bit("LVDS_SSTL12_SSTL135_SSTL15.IN_DIFF");
                        if (iostandard == "LVDS")
                            write_bit("LVDS.IN_USE");
                    }
                } else {
                    if (iostandard == "TDMS_33")
                        write_bit("TDMS_33.IN_DIFF");
                    else
                        write_bit("LVDS_25_SSTL135_SSTL15.IN_DIFF");
                }

                if (pad->attrs.count(id_IN_TERM))
                    write_bit("IN_TERM." + pad->attrs.at(id_IN_TERM).as_string());
            }

            // IN_ONLY
            if (!is_output) {
                if (is_riob18) {
                    // vivado also sets this bit for DIFF_SSTL
                    if (is_diff && (yLoc == 0))
                        write_bit("LVDS.IN_ONLY");
                    else
                        write_bit("LVCMOS12_LVCMOS15_LVCMOS18_SSTL12_SSTL135_SSTL15.IN_ONLY");
                } else
                    write_bit("LVCMOS12_LVCMOS15_LVCMOS18_LVCMOS25_LVCMOS33_LVDS_25_LVTTL_SSTL135_SSTL15_TMDS_33.IN_"
                              "ONLY");
            }
        }

        if (!is_riob18 && (is_low_volt_lvcmos || is_sstl)) {
            if (iostandard == "SSTL12") {
                log_error("SSTL12 is only available on high performance banks.");
            }
            write_bit("LVCMOS12_LVCMOS15_LVCMOS18_SSTL135_SSTL15.STEPDOWN");
            ioconfig_by_hclk[hclk].stepdown = true;
            is_stepdown = true;
        }

        if (is_riob18 && (is_input || is_output) && (boost::contains(iostandard, "SSTL") || iostandard == "LVDS")) {
            if (((yLoc == 0) && (iostandard == "LVDS")) || boost::contains(iostandard, "SSTL")) {
                // TODO: I get bit conflicts with this, it seems to work anyway. Test more.
                // write_bit("LVDS.IN_USE");
            }
        }

        if (is_input && is_output && !is_diff && yLoc == 1) {
            if (is_riob18 && boost::starts_with(iostandard, "SSTL"))
                write_bit("SSTL12_SSTL135_SSTL15.IN");
        }

        // IN_TERM.NONE and IN_ONLY for TMDS_33 output, e.g. HDMI signals
        if (is_output && is_diff) {
            if (is_tmds33 && yLoc == 1) {
                if (pad->attrs.count(id_IN_TERM))
                    write_bit("IN_TERM." + pad->attrs.at(id_IN_TERM).as_string());
                else
                    write_bit("IN_TERM.NONE");
                write_bit("LVCMOS12_LVCMOS15_LVCMOS18_LVCMOS25_LVCMOS33_LVDS_25_LVTTL_SSTL135_SSTL15_TMDS_33.IN_ONLY");
            }
        }

        write_bit("PULLTYPE." + pulltype);
        pop(); // IOB_YN

        BelId inv;

        auto pad_bel_site = uarch->get_bel_site(pad->bel);

        if (is_riob18)
            inv = uarch->get_site_bel(pad_bel_site, ctx->id("IOB18S.O_ININV"));
        else
            inv = uarch->get_site_bel(pad_bel_site, ctx->id("IOB33S.O_ININV"));

        if (inv != BelId() && ctx->getBoundBelCell(inv) != nullptr)
            write_bit("OUT_DIFF");

        if (is_stepdown && !is_sing)
            write_bit("IOB_Y" + std::to_string(ioLoc.y) + ".LVCMOS12_LVCMOS15_LVCMOS18_SSTL135_SSTL15.STEPDOWN");

        pop(); // tile
    }

    void write_iol_config(CellInfo *ci)
    {
        std::string tile = uarch->tile_name(ci->bel.tile);
        push(tile);
        bool is_sing = boost::contains(tile, "_SING_");
        bool is_top_sing = ci->bel.tile < uarch->hclk_for_ioi(ci->bel.tile);

        auto site_key = uarch->get_bel_site(ci->bel);
        std::string site = uarch->get_site_name(site_key).str(ctx);
        std::string sitetype = site.substr(0, site.find('_'));
        Loc siteloc = uarch->rel_site_loc(site_key);
        push(stringf("%s_Y%d", sitetype.c_str(), is_sing ? (is_top_sing ? 1 : 0) : (1 - siteloc.y)));

        if (ci->type == id_ILOGICE3_IFF) {
            write_bit("IDDR.IN_USE");
            write_bit("IDDR_OR_ISERDES.IN_USE");
            write_bit("ISERDES.MODE.MASTER");
            write_bit("ISERDES.NUM_CE.N1");

            // Switch IDELMUXE3 to include the IDELAY element, if we have an IDELAYE2 driving D
            NetInfo *d = ci->getPort(id_D);
            if (d == nullptr || d->driver.cell == nullptr)
                log_error("%s '%s' has disconnected D input\n", ci->type.c_str(ctx), ctx->nameOf(ci));
            CellInfo *drv = d->driver.cell;
            if (boost::contains(drv->type.str(ctx), "IDELAYE2"))
                write_bit("IDELMUXE3.P0");
            else
                write_bit("IDELMUXE3.P1");

            // clock edge
            std::string edge = str_or_default(ci->params, id_DDR_CLK_EDGE, "OPPOSITE_EDGE");
            if (edge == "SAME_EDGE")
                write_bit("IFF.DDR_CLK_EDGE.SAME_EDGE");
            else if (edge == "OPPOSITE_EDGE")
                write_bit("IFF.DDR_CLK_EDGE.OPPOSITE_EDGE");
            else
                log_error("unsupported clock edge parameter for cell '%s' at %s: %s. Supported are: SAME_EDGE and "
                          "OPPOSITE_EDGE",
                          ci->name.c_str(ctx), site.c_str(), edge.c_str());

            std::string srtype = str_or_default(ci->params, id_SRTYPE, "SYNC");
            if (srtype == "SYNC")
                write_bit("IFF.SRTYPE.SYNC");
            else
                write_bit("IFF.SRTYPE.ASYNC");

            write_bit("IFF.ZINV_C", !bool_or_default(ci->params, id_IS_CLK_INVERTED, false));
            write_bit("ZINV_D", !bool_or_default(ci->params, id_IS_D_INVERTED, false));

            auto init = int_or_default(ci->params, id_INIT_Q1, 0);
            if (init == 0)
                write_bit("IFF.ZINIT_Q1");
            init = int_or_default(ci->params, id_INIT_Q2, 0);
            if (init == 0)
                write_bit("IFF.ZINIT_Q2");

            auto sr_name = str_or_default(ci->attrs, id_X_ORIG_PORT_SR, "R");
            if (sr_name == "R") {
                write_bit("IFF.ZSRVAL_Q1");
                write_bit("IFF.ZSRVAL_Q2");
            }
        } else if (ci->type.in(id_OLOGICE2_OUTFF, id_OLOGICE3_OUTFF)) {
            std::string edge = str_or_default(ci->params, id_DDR_CLK_EDGE, "OPPOSITE_EDGE");
            if (edge == "SAME_EDGE")
                write_bit("ODDR.DDR_CLK_EDGE.SAME_EDGE");

            write_bit("ODDR_TDDR.IN_USE");
            write_bit("OQUSED");
            write_bit("OSERDES.DATA_RATE_OQ.DDR");
            write_bit("OSERDES.DATA_RATE_TQ.BUF");

            std::string srtype = str_or_default(ci->params, id_SRTYPE, "SYNC");
            if (srtype == "SYNC")
                write_bit("OSERDES.SRTYPE.SYNC");

            for (std::string d : {"D1", "D2"})
                write_bit("IS_" + d + "_INVERTED",
                          bool_or_default(ci->params, ctx->id("IS_" + d + "_INVERTED"), false));

            auto init = int_or_default(ci->params, id_INIT, 1);
            if (init == 0)
                write_bit("ZINIT_OQ");

            write_bit("ODDR.SRUSED", ci->getPort(id_SR) != nullptr);
            auto sr_name = str_or_default(ci->attrs, id_X_ORIG_PORT_SR, "R");
            if (sr_name == "R")
                write_bit("ZSRVAL_OQ");

            auto clk_inv = bool_or_default(ci->params, id_IS_CLK_INVERTED);
            if (!clk_inv)
                write_bit("ZINV_CLK");
        } else if (ci->type == id_OSERDESE2_OSERDESE2) {
            write_bit("ODDR.DDR_CLK_EDGE.SAME_EDGE");
            write_bit("ODDR.SRUSED");
            write_bit("ODDR_TDDR.IN_USE");

            auto serdes_mode = str_or_default(ci->params, id_SERDES_MODE, "MASTER");
            bool is_cascaded = (serdes_mode == "SLAVE");

            // For cascaded OSERDESE2, OQUSED must be set even though OQ is not connected
            write_bit("OQUSED", is_cascaded || ci->getPort(id_OQ));
            write_bit("ZINV_CLK", !bool_or_default(ci->params, id_IS_CLK_INVERTED, false));
            for (std::string t : {"T1", "T2", "T3", "T4"})
                write_bit("ZINV_" + t, (ci->getPort(ctx->id(t)) != nullptr || t == "T1") &&
                                               !bool_or_default(ci->params, ctx->id("IS_" + t + "_INVERTED"), false));
            for (std::string d : {"D1", "D2", "D3", "D4", "D5", "D6", "D7", "D8"})
                write_bit("IS_" + d + "_INVERTED",
                          bool_or_default(ci->params, ctx->id("IS_" + d + "_INVERTED"), false));
            write_bit("ZINIT_OQ", !bool_or_default(ci->params, id_INIT_OQ, false));
            write_bit("ZINIT_TQ", !bool_or_default(ci->params, id_INIT_TQ, false));
            write_bit("ZSRVAL_OQ", !bool_or_default(ci->params, id_SRVAL_OQ, false));
            write_bit("ZSRVAL_TQ", !bool_or_default(ci->params, id_SRVAL_TQ, false));

            push("OSERDES");
            write_bit("IN_USE");
            std::string type = str_or_default(ci->params, id_DATA_RATE_OQ, "BUF");
            write_bit(std::string("DATA_RATE_OQ.") + ((ci->getPort(id_OQ) != nullptr) ? type : "DDR"));
            write_bit(std::string("DATA_RATE_TQ.") +
                      ((ci->getPort(id_TQ) != nullptr) ? str_or_default(ci->params, id_DATA_RATE_TQ, "BUF") : "BUF"));
            int width = int_or_default(ci->params, id_DATA_WIDTH, 8);
#if 0
            write_bit("DATA_WIDTH.W" + std::to_string(width));
            if (type == "DDR" && (width == 6 || width == 8)) {
                write_bit("DATA_WIDTH.DDR.W6_8");
                write_bit("DATA_WIDTH.SDR.W2_4_5_6");
            } else if (type == "SDR" && (width == 2 || width == 4 || width == 5 || width == 6)) {
                write_bit("DATA_WIDTH.SDR.W2_4_5_6");
            }
#else
            if (type == "DDR")
                write_bit("DATA_WIDTH.DDR.W" + std::to_string(width));
            else if (type == "SDR")
                write_bit("DATA_WIDTH.SDR.W" + std::to_string(width));
            else
                write_bit("DATA_WIDTH.W" + std::to_string(width));
#endif
            write_bit("SRTYPE.SYNC");
            write_bit("TSRTYPE.SYNC");
            if (is_cascaded)
                write_bit("SERDES_MODE.SLAVE");
            pop();
        } else if (ci->type == id_ISERDESE2_ISERDESE2) {
            std::string data_rate = str_or_default(ci->params, id_DATA_RATE);
            write_bit("IDDR_OR_ISERDES.IN_USE");
            if (data_rate == "DDR")
                write_bit("IDDR.IN_USE");
            write_bit("IFF.DDR_CLK_EDGE.OPPOSITE_EDGE");
            write_bit("IFF.SRTYPE.SYNC");
            for (int i = 1; i <= 4; i++) {
                write_bit("IFF.ZINIT_Q" + std::to_string(i),
                          !bool_or_default(ci->params, ctx->idf("INIT_Q%d", i), false));
                write_bit("IFF.ZSRVAL_Q" + std::to_string(i),
                          !bool_or_default(ci->params, ctx->idf("SRVAL_Q%d", i), false));
            }
            write_bit("IFF.ZINV_C", !bool_or_default(ci->params, id_IS_CLK_INVERTED, false));
            write_bit("IFF.ZINV_OCLK", !bool_or_default(ci->params, id_IS_OCLK_INVERTED, false));

            std::string iobdelay = str_or_default(ci->params, id_IOBDELAY, "NONE");
            write_bit("IFFDELMUXE3.P0", (iobdelay == "IFD"));
            write_bit("ZINV_D", !bool_or_default(ci->params, id_IS_D_INVERTED, false) && (iobdelay != "IFD"));

            push("ISERDES");
            write_bit("IN_USE");
            int width = int_or_default(ci->params, id_DATA_WIDTH, 8);
            std::string mode = str_or_default(ci->params, id_INTERFACE_TYPE, "NETWORKING");
            std::string rate = str_or_default(ci->params, id_DATA_RATE, "DDR");
            write_bit(mode + "." + rate + ".W" + std::to_string(width));
            write_bit("MODE." + str_or_default(ci->params, id_SERDES_MODE, "MASTER"));
            write_bit("NUM_CE.N" + std::to_string(int_or_default(ci->params, id_NUM_CE, 1)));
            pop();
        } else if (ci->type == id_IDELAYE2_IDELAYE2) {
            write_bit("IN_USE");
            write_bit("CINVCTRL_SEL", str_or_default(ci->params, id_CINVCTRL_SEL, "FALSE") == "TRUE");
            write_bit("PIPE_SEL", str_or_default(ci->params, id_PIPE_SEL, "FALSE") == "TRUE");
            write_bit("HIGH_PERFORMANCE_MODE", str_or_default(ci->params, id_HIGH_PERFORMANCE_MODE, "FALSE") == "TRUE");
            write_bit("DELAY_SRC_" + str_or_default(ci->params, id_DELAY_SRC, "IDATAIN"));
            write_bit("IDELAY_TYPE_" + str_or_default(ci->params, id_IDELAY_TYPE, "FIXED"));
            write_int_vector("IDELAY_VALUE[4:0]", int_or_default(ci->params, id_IDELAY_VALUE, 0), 5, false);
            write_int_vector("ZIDELAY_VALUE[4:0]", int_or_default(ci->params, id_IDELAY_VALUE, 0), 5, true);
            write_bit("IS_DATAIN_INVERTED", bool_or_default(ci->params, id_IS_DATAIN_INVERTED, false));
            write_bit("IS_IDATAIN_INVERTED", bool_or_default(ci->params, id_IS_IDATAIN_INVERTED, false));
        } else if (ci->type == id_ODELAYE2_ODELAYE2) {
            write_bit("IN_USE");
            write_bit("CINVCTRL_SEL", str_or_default(ci->params, id_CINVCTRL_SEL, "FALSE") == "TRUE");
            write_bit("HIGH_PERFORMANCE_MODE", str_or_default(ci->params, id_HIGH_PERFORMANCE_MODE, "FALSE") == "TRUE");
            auto type = str_or_default(ci->params, id_ODELAY_TYPE, "FIXED");
            if (type != "FIXED")
                write_bit("ODELAY_TYPE_" + type);
            write_int_vector("ODELAY_VALUE[4:0]", int_or_default(ci->params, id_ODELAY_VALUE, 0), 5, false);
            write_int_vector("ZODELAY_VALUE[4:0]", int_or_default(ci->params, id_ODELAY_VALUE, 0), 5, true);
            write_bit("ZINV_ODATAIN", !bool_or_default(ci->params, id_IS_ODATAIN_INVERTED, false));
        } else {
            NPNR_ASSERT_FALSE("unsupported IOLOGIC");
        }
        pop(2);
    }

    void write_io()
    {
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            if (ci->type == id_PAD) {
                write_io_config(ci);
                blank();
            } else if (ci->type.in(id_ILOGICE3_IFF, id_OLOGICE2_OUTFF, id_OLOGICE3_OUTFF, id_OSERDESE2_OSERDESE2,
                                   id_ISERDESE2_ISERDESE2, id_IDELAYE2_IDELAYE2, id_ODELAYE2_ODELAYE2)) {
                write_iol_config(ci);
                blank();
            }
        }
        for (auto &hclk : ioconfig_by_hclk) {
            push(uarch->tile_name(hclk.first));
            write_bit("STEPDOWN", hclk.second.stepdown);
            write_bit("VREF.V_675_MV", hclk.second.vref);
            write_bit("ONLY_DIFF_IN_USE", hclk.second.only_diff);
            write_bit("TMDS_33_IN_USE", hclk.second.tmds_33);
            write_bit("LVDS_25_IN_USE", hclk.second.lvds_25);
            pop();
        }
    }

    std::vector<std::string> used_wires_starting_with(int tile, const std::string &prefix, bool is_source)
    {
        std::vector<std::string> wires;
        if (!pips_by_tile.count(tile))
            return wires;
        for (auto pip : pips_by_tile[tile]) {
            auto &pd = chip_pip_info(ctx->chip_info, pip);
            int wire_index = is_source ? pd.src_wire : pd.dst_wire;
            std::string wire = IdString(chip_wire_info(ctx->chip_info, WireId(pip.tile, wire_index)).name).str(ctx);
            if (boost::starts_with(wire, prefix))
                wires.push_back(wire);
        }
        return wires;
    }

    void write_clocking()
    {
        std::string name, type;

        std::set<std::string> all_gclk;
        dict<int, std::set<std::string>> hclk_by_row;

        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            if (ci->type == id_BUFGCTRL) {
                push(uarch->tile_name(ci->bel.tile));
                auto xy = uarch->rel_site_loc(uarch->get_bel_site(ci->bel));
                push(stringf("BUFGCTRL.BUFGCTRL_X%dY%d", xy.x, xy.y));
                write_bit("IN_USE");
                write_bit("INIT_OUT", bool_or_default(ci->params, id_INIT_OUT));
                write_bit("IS_IGNORE0_INVERTED", bool_or_default(ci->params, id_IS_IGNORE0_INVERTED));
                write_bit("IS_IGNORE1_INVERTED", bool_or_default(ci->params, id_IS_IGNORE1_INVERTED));
                write_bit("ZINV_CE0", !bool_or_default(ci->params, id_IS_CE0_INVERTED));
                write_bit("ZINV_CE1", !bool_or_default(ci->params, id_IS_CE1_INVERTED));
                write_bit("ZINV_S0", !bool_or_default(ci->params, id_IS_S0_INVERTED));
                write_bit("ZINV_S1", !bool_or_default(ci->params, id_IS_S1_INVERTED));
                pop(2);
            } else if (ci->type == id_PLLE2_ADV_PLLE2_ADV) {
                write_pll(ci);
            } else if (ci->type == id_MMCME2_ADV_MMCME2_ADV) {
                write_mmcm(ci);
            }
            blank();
        }

        for (int tile = 0; tile < ctx->chip_info->tile_insts.ssize(); tile++) {
            std::string name = uarch->tile_name(tile);
            std::string type = ctx->get_tile_type(tile).str(ctx);
            push(name);
            if (type == "HCLK_L" || type == "HCLK_R" || type == "HCLK_L_BOT_UTURN" || type == "HCLK_R_BOT_UTURN") {
                auto used_sources = used_wires_starting_with(tile, "HCLK_CK_", true);
                push("ENABLE_BUFFER");
                for (auto s : used_sources) {
                    if (boost::contains(s, "BUFHCLK")) {
                        write_bit(s);
                        hclk_by_row[tile / ctx->chip_info->width].insert(s.substr(s.find("BUFHCLK")));
                    }
                }
                pop();
            } else if (boost::starts_with(type, "CLK_HROW")) {
                auto used_gclk = used_wires_starting_with(tile, "CLK_HROW_R_CK_GCLK", true);
                auto used_ck_in = used_wires_starting_with(tile, "CLK_HROW_CK_IN", true);
                for (auto s : used_gclk) {
                    write_bit(s + "_ACTIVE");
                    all_gclk.insert(s.substr(s.find("GCLK")));
                }
                for (auto s : used_ck_in) {
                    if (boost::contains(s, "HROW_CK_INT"))
                        continue;
                    write_bit(s + "_ACTIVE");
                }
            } else if (boost::starts_with(type, "HCLK_CMT")) {
                auto used_ccio = used_wires_starting_with(tile, "HCLK_CMT_CCIO", true);
                for (auto s : used_ccio) {
                    write_bit(s + "_ACTIVE");
                    write_bit(s + "_USED");
                }
                auto used_hclk = used_wires_starting_with(tile, "HCLK_CMT_CK_", true);
                for (auto s : used_hclk) {
                    if (boost::contains(s, "BUFHCLK")) {
                        write_bit(s + "_USED");
                        hclk_by_row[tile / ctx->chip_info->width].insert(s.substr(s.find("BUFHCLK")));
                    }
                }
            }
            pop();
            blank();
        }

        for (int tile = 0; tile < ctx->chip_info->tile_insts.ssize(); tile++) {
            std::string name = uarch->tile_name(tile);
            std::string type = ctx->get_tile_type(tile).str(ctx);
            push(name);
            if (type == "CLK_BUFG_REBUF") {
                for (auto &gclk : all_gclk) {
                    write_bit(gclk + "_ENABLE_ABOVE");
                    write_bit(gclk + "_ENABLE_BELOW");
                }
            } else if (boost::starts_with(type, "HCLK_CMT")) {
                for (auto &hclk : hclk_by_row[tile / ctx->chip_info->width]) {
                    write_bit("HCLK_CMT_CK_" + hclk + "_USED");
                }
            }
            pop();
            blank();
        }
    }

    void write_bram_width(CellInfo *ci, const std::string &name, bool is_36, bool is_y1)
    {
        int width = int_or_default(ci->params, ctx->id(name), 0);
        if (width == 0)
            return;
        int actual_width = width;
        if (is_36) {
            if (width == 1)
                actual_width = 1;
            else
                actual_width = width / 2;
        }
        if (((is_36 && width == 72) || (is_y1 && actual_width == 36)) && name == "READ_WIDTH_A") {
            write_bit(name + "_18");
        }
        if (actual_width == 36) {
            write_bit("SDP_" + name.substr(0, name.length() - 2) + "_36");
            if (name.find("WRITE") == 0) {
                write_bit(name.substr(0, name.size() - 1) + "A_18");
                write_bit(name.substr(0, name.size() - 1) + "B_18");
            } else if (name.find("READ") == 0) {
                write_bit(name.substr(0, name.size() - 1) + "B_18");
            }
        } else {
            write_bit(name + "_" + std::to_string(actual_width));
        }
    }

    void write_bram_init(int half, CellInfo *ci, bool is_36)
    {
        for (std::string mode : {"", "P"}) {
            for (int i = 0; i < (mode == "P" ? 8 : 64); i++) {
                bool has_init = false;
                std::vector<bool> init_data(256, false);
                if (is_36) {
                    for (int j = 0; j < 2; j++) {
                        IdString param = ctx->idf("INIT%s_%02X", mode.c_str(), i * 2 + j);
                        if (ci->params.count(param)) {
                            auto &init0 = ci->params.at(param);
                            has_init = true;
                            for (int k = half; k < 256; k += 2) {
                                if (k >= int(init0.str.size()))
                                    break;
                                init_data[j * 128 + (k / 2)] = init0.str[k] == Property::S1;
                            }
                        }
                    }
                } else {
                    IdString param = ctx->idf("INIT%s_%02X", mode.c_str(), i);
                    if (ci->params.count(param)) {
                        auto &init = ci->params.at(param);
                        has_init = true;
                        for (int k = 0; k < 256; k++) {
                            if (k >= int(init.str.size()))
                                break;
                            init_data[k] = init.str[k] == Property::S1;
                        }
                    }
                }
                if (has_init)
                    write_vector(stringf("INIT%s_%02X[255:0]", mode.c_str(), i), init_data);
            }
        }
    }

    void write_bram_half(int tile, int half, CellInfo *ci)
    {
        push(uarch->tile_name(tile));
        push("RAMB18_Y" + std::to_string(half));
        if (ci != nullptr) {
            bool is_36 = ci->type == id_RAMB36E1_RAMB36E1;
            write_bit("IN_USE");
            write_bram_width(ci, "READ_WIDTH_A", is_36, half == 1);
            write_bram_width(ci, "READ_WIDTH_B", is_36, half == 1);
            write_bram_width(ci, "WRITE_WIDTH_A", is_36, half == 1);
            write_bram_width(ci, "WRITE_WIDTH_B", is_36, half == 1);
            write_bit("DOA_REG", bool_or_default(ci->params, id_DOA_REG, false));
            write_bit("DOB_REG", bool_or_default(ci->params, id_DOB_REG, false));
            for (auto &invpin : invertible_pins[ctx->id(ci->attrs[id_X_ORIG_TYPE].as_string())])
                write_bit("ZINV_" + invpin.str(ctx),
                          !bool_or_default(ci->params, ctx->id("IS_" + invpin.str(ctx) + "_INVERTED"), false));
            for (auto wrmode : {"WRITE_MODE_A", "WRITE_MODE_B"}) {
                std::string mode = str_or_default(ci->params, ctx->id(wrmode), "WRITE_FIRST");
                if (mode != "WRITE_FIRST")
                    write_bit(std::string(wrmode) + "_" + mode);
            }
            write_vector("ZINIT_A[17:0]", std::vector<bool>(18, true));
            write_vector("ZINIT_B[17:0]", std::vector<bool>(18, true));
            write_vector("ZSRVAL_A[17:0]", std::vector<bool>(18, true));
            write_vector("ZSRVAL_B[17:0]", std::vector<bool>(18, true));

            write_bram_init(half, ci, is_36);
        }
        pop();
        if (half == 0) {
            auto used_rdaddrcasc = used_wires_starting_with(tile, "BRAM_CASCOUT_ADDRARDADDR", false);
            auto used_wraddrcasc = used_wires_starting_with(tile, "BRAM_CASCOUT_ADDRBWRADDR", false);
            write_bit("CASCOUT_ARD_ACTIVE", !used_rdaddrcasc.empty());
            write_bit("CASCOUT_BWR_ACTIVE", !used_wraddrcasc.empty());
        }
        pop();
    }

    void write_bram()
    {
        for (int tile = 0; tile < ctx->chip_info->tile_insts.ssize(); tile++) {
            IdString type = ctx->get_tile_type(tile);
            if (type.in(id_BRAM_L, id_BRAM_R)) {
                CellInfo *l = nullptr, *u = nullptr;
                const auto &bts = uarch->tile_status[tile].bts;
                if (bts) {
                    if (bts->cells[BEL_RAM36] != nullptr) {
                        l = bts->cells[BEL_RAM36];
                        u = bts->cells[BEL_RAM36];
                    } else {
                        l = bts->cells[BEL_RAM18_L];
                        u = bts->cells[BEL_RAM18_U];
                    }
                }
                write_bram_half(tile, 0, l);
                write_bram_half(tile, 1, u);
                blank();
            }
        }
    }

    double float_or_default(CellInfo *ci, const std::string &name, double def)
    {
        IdString p = ctx->id(name);
        if (!ci->params.count(p))
            return def;
        auto &prop = ci->params.at(p);
        if (prop.is_string)
            return std::stod(prop.as_string());
        else
            return prop.as_int64();
    }

    void write_pll_clkout(const std::string &name, CellInfo *ci)
    {
        // FIXME: variable duty cycle
        int high = 1, low = 1, phasemux = 0, delaytime = 0, frac = 0;
        bool no_count = false, edge = false;
        double divide = float_or_default(ci, name + ((name == "CLKFBOUT") ? "_MULT" : "_DIVIDE"), 1);
        double phase = float_or_default(ci, name + "_PHASE", 1);
        if (divide <= 1) {
            no_count = true;
        } else {
            high = floor(divide / 2);
            low = int(floor(divide) - high);
            if (high != low)
                edge = true;
            if (name == "CLKOUT1" || name == "CLKFBOUT")
                frac = floor(divide * 8) - floor(divide) * 8;
            int phase_eights = floor((phase / 360) * divide * 8);
            phasemux = phase_eights % 8;
            delaytime = phase_eights / 8;
        }
        bool used = false;
        if (name == "DIVCLK" || name == "CLKFBOUT") {
            used = true;
        } else {
            used = ci->getPort(ctx->id(name)) != nullptr;
        }
        if (name == "DIVCLK") {
            write_int_vector("DIVCLK_DIVCLK_HIGH_TIME[5:0]", high, 6);
            write_int_vector("DIVCLK_DIVCLK_LOW_TIME[5:0]", low, 6);
            write_bit("DIVCLK_DIVCLK_EDGE[0]", edge);
            write_bit("DIVCLK_DIVCLK_NO_COUNT[0]", no_count);
        } else if (used) {
            write_bit(name + "_CLKOUT1_OUTPUT_ENABLE[0]");
            write_int_vector(name + "_CLKOUT1_HIGH_TIME[5:0]", high, 6);
            write_int_vector(name + "_CLKOUT1_LOW_TIME[5:0]", low, 6);
            write_int_vector(name + "_CLKOUT1_PHASE_MUX[2:0]", phasemux, 3);
            write_bit(name + "_CLKOUT2_EDGE[0]", edge);
            write_bit(name + "_CLKOUT2_NO_COUNT[0]", no_count);
            write_int_vector(name + "_CLKOUT2_DELAY_TIME[5:0]", delaytime, 6);
            if (frac != 0) {
                write_bit(name + "_CLKOUT2_FRAC_EN[0]", edge);
                write_int_vector(name + "_CLKOUT2_FRAC[2:0]", frac, 3);
            }
        }
    }

    void write_pll(CellInfo *ci)
    {
        push(uarch->tile_name(ci->bel.tile));
        push("PLLE2_ADV");
        write_bit("IN_USE");
        // FIXME: should be INV not ZINV (XRay error?)
        write_bit("ZINV_PWRDWN", bool_or_default(ci->params, id_IS_PWRDWN_INVERTED, false));
        write_bit("ZINV_RST", bool_or_default(ci->params, id_IS_RST_INVERTED, false));
        write_bit("INV_CLKINSEL", bool_or_default(ci->params, id_IS_CLKINSEL_INVERTED, false));
        write_pll_clkout("DIVCLK", ci);
        write_pll_clkout("CLKFBOUT", ci);
        write_pll_clkout("CLKOUT0", ci);
        write_pll_clkout("CLKOUT1", ci);
        write_pll_clkout("CLKOUT2", ci);
        write_pll_clkout("CLKOUT3", ci);
        write_pll_clkout("CLKOUT4", ci);
        write_pll_clkout("CLKOUT5", ci);

        std::string comp = str_or_default(ci->params, id_COMPENSATION, "INTERNAL");
        push("COMPENSATION");
        if (comp == "INTERNAL") {
            // write_bit("INTERNAL");
            write_bit("Z_ZHOLD_OR_CLKIN_BUF");
        } else {
            NPNR_ASSERT_FALSE("unsupported compensation type");
        }
        pop();

        // FIXME: should these be calculated somehow?
        write_int_vector("FILTREG1_RESERVED[11:0]", 0x8, 12);
        write_int_vector("LKTABLE[39:0]", 0xB5BE8FA401ULL, 40);
        write_bit("LOCKREG3_RESERVED[0]");
        write_int_vector("TABLE[9:0]", 0x3B4, 10);
        pop(2);
    }

    void write_mmcm_clkout(const std::string &name, CellInfo *ci)
    {
        // FIXME: variable duty cycle
        int high = 1, low = 1, phasemux = 0, delaytime = 0, frac = 0;
        bool no_count = false, edge = false;
        double divide = float_or_default(
                ci, name + ((name == "CLKFBOUT") ? "_MULT_F" : (name == "CLKOUT0" ? "_DIVIDE_F" : "_DIVIDE")), 1);
        double phase = float_or_default(ci, name + "_PHASE", 1);
        if (divide <= 1) {
            no_count = true;
        } else {
            high = floor(divide / 2);
            low = int(floor(divide) - high);
            if (high != low)
                edge = true;
            if (name == "CLKOUT0" || name == "CLKFBOUT")
                frac = floor(divide * 8) - floor(divide) * 8;
            int phase_eights = floor((phase / 360) * divide * 8);
            phasemux = phase_eights % 8;
            delaytime = phase_eights / 8;
        }
        bool used = false;
        if (name == "DIVCLK" || name == "CLKFBOUT") {
            used = true;
        } else {
            used = ci->getPort(ctx->id(name)) != nullptr;
        }
        if (name == "DIVCLK") {
            write_int_vector("DIVCLK_DIVCLK_HIGH_TIME[5:0]", high, 6);
            write_int_vector("DIVCLK_DIVCLK_LOW_TIME[5:0]", low, 6);
            write_bit("DIVCLK_DIVCLK_EDGE[0]", edge);
            write_bit("DIVCLK_DIVCLK_NO_COUNT[0]", no_count);
        } else if (used) {
            auto is_clkout_5_or_6 = name == "CLKOUT5" || name == "CLKOUT6";
            auto is_clkout0 = name == "CLKOUT0";
            auto is_clkfbout = name == "CLKFBOUT";

            if ((is_clkout0 || is_clkfbout) && frac != 0) {
                --high;
                --low;

                auto frac_shifted = frac >> 1;
                // CLKOUT0 controls CLKOUT5_CLKOUT2, CLKFBOUT controls CLKOUT6_CLKOUT2
                std::string frac_conf_name = is_clkout0 ? "CLKOUT5_CLKOUT2_" : "CLKOUT6_CLKOUT2_";

                if (1 <= frac_shifted) {
                    write_bit(frac_conf_name + "FRACTIONAL_FRAC_WF_F[0]");
                    write_int_vector(frac_conf_name + "FRACTIONAL_PHASE_MUX_F[1:0]", frac_shifted, 2);
                }
            }

            write_bit(name + "_CLKOUT1_OUTPUT_ENABLE[0]");
            write_int_vector(name + "_CLKOUT1_HIGH_TIME[5:0]", high, 6);
            write_int_vector(name + "_CLKOUT1_LOW_TIME[5:0]", low, 6);

            auto phase_mux_feature =
                    name + (is_clkout_5_or_6 ? "_CLKOUT2_FRACTIONAL_PHASE_MUX_F[0]" : "_CLKOUT2_PHASE_MUX[0]");
            write_int_vector(name + "_CLKOUT1_PHASE_MUX[2:0]", phasemux, 3);

            auto edge_feature = name + (is_clkout_5_or_6 ? "_CLKOUT2_FRACTIONAL_EDGE[0]" : "_CLKOUT2_EDGE[0]");
            write_bit(edge_feature, edge);

            auto no_count_feature =
                    name + (is_clkout_5_or_6 ? "_CLKOUT2_FRACTIONAL_NO_COUNT[0]" : "_CLKOUT2_NO_COUNT[0]");
            write_bit(no_count_feature, no_count);

            auto delay_time_feature =
                    name + (is_clkout_5_or_6 ? "_CLKOUT2_FRACTIONAL_DELAY_TIME[5:0]" : "_CLKOUT2_DELAY_TIME[5:0]");
            write_int_vector(delay_time_feature, delaytime, 6);

            if (!is_clkout_5_or_6 && frac != 0) {
                write_bit(name + "_CLKOUT2_FRAC_EN[0]", 1);
                write_bit(name + "_CLKOUT2_FRAC_WF_R[0]", 1);
                write_int_vector(name + "_CLKOUT2_FRAC[2:0]", frac, 3);
            }
        }
    }

    // From openXC7
    void write_mmcm(CellInfo *ci)
    {
        push(uarch->tile_name(ci->bel.tile));
        push("MMCME2_ADV");
        write_bit("IN_USE");
        // FIXME: should be INV not ZINV (XRay error?)
        write_bit("ZINV_PWRDWN", bool_or_default(ci->params, id_IS_PWRDWN_INVERTED, false));
        write_bit("ZINV_RST", bool_or_default(ci->params, id_IS_RST_INVERTED, false));
        write_bit("ZINV_PSEN", bool_or_default(ci->params, id_IS_PSEN_INVERTED, false));
        write_bit("ZINV_PSINCDEC", bool_or_default(ci->params, id_IS_PSINCDEC_INVERTED, false));
        write_bit("INV_CLKINSEL", bool_or_default(ci->params, id_IS_CLKINSEL_INVERTED, false));
        write_mmcm_clkout("DIVCLK", ci);
        write_mmcm_clkout("CLKFBOUT", ci);
        write_mmcm_clkout("CLKOUT0", ci);
        write_mmcm_clkout("CLKOUT1", ci);
        write_mmcm_clkout("CLKOUT2", ci);
        write_mmcm_clkout("CLKOUT3", ci);
        write_mmcm_clkout("CLKOUT4", ci);
        write_mmcm_clkout("CLKOUT5", ci);
        write_mmcm_clkout("CLKOUT6", ci);

        std::string comp = str_or_default(ci->params, id_COMPENSATION, "INTERNAL");
        push("COMP");
        if (comp == "INTERNAL" || comp == "ZHOLD") {
            // does not seem to make a difference in vivado
            // both modes set this bit
            write_bit("Z_ZHOLD");
        } else {
            log_error("unsupported COMPENSATION type '%s' for MMCM (supported compensation types: INTERNAL, ZHOLD)\n",
                      comp.c_str());
        }
        pop();

        auto clkfbout_mult = (int)float_or_default(ci, "CLKFBOUT_MULT_F", 5.000);
        if (63 < clkfbout_mult)
            log_error("MMCME2_ADV: CLKFBOUT_MULT_F must not be greater than 63");
        if (0 == clkfbout_mult)
            log_error("MMCME2_ADV: CLKFBOUT_MULT_F must not be 0");
        write_int_vector("LKTABLE[39:0]", Xc7MMCM::lk_table[clkfbout_mult - 1], 40);

        std::string bandwidth = str_or_default(ci->params, id_BANDWIDTH, "OPTIMIZED");
        const uint16_t *filter_lookup;
        if (bandwidth == "LOW")
            filter_lookup = Xc7MMCM::filter_lookup_low;
        else if (bandwidth == "LOW_SS")
            filter_lookup = Xc7MMCM::filter_lookup_low_ss;
        else if (bandwidth == "HIGH")
            filter_lookup = Xc7MMCM::filter_lookup_high;
        else
            filter_lookup = Xc7MMCM::filter_lookup_optimized;
        write_int_vector("FILTREG1_RESERVED[11:0]", filter_lookup[clkfbout_mult - 1], 12);

        // 0x9900 enables fractional counters
        // only int counters would be 0x1 << 8
        // 0xffff enables everything, I suppose, this is what is used in xap888
        write_int_vector("POWER_REG_POWER_REG_POWER_REG[15:0]", 0xffff, 16);
        write_bit("LOCKREG3_RESERVED[0]");
        write_int_vector("TABLE[9:0]", 0x3d4, 10);
        pop(2);
    }
    void write_dsp_cell(CellInfo *ci)
    {
        auto tile_name = uarch->tile_name(ci->bel.tile);
        auto tile_side = tile_name.at(4);
        push(tile_name);
        push("DSP48");
        auto xy = uarch->rel_site_loc(uarch->get_bel_site(ci->bel));
        auto dsp = stringf("DSP_%d", xy.y);
        push(dsp);

        auto write_bus_zinv = [&](std::string name, int width) {
            for (int i = 0; i < width; i++) {
                std::string b = stringf("[%d]", i);
                bool inv = (int_or_default(ci->params, ctx->id("IS_" + name + "_INVERTED"), 0) >> i) & 0x1;
                inv |= bool_or_default(ci->params, ctx->id("IS_" + name + b + "_INVERTED"), false);
                write_bit("ZIS_" + name + "_INVERTED" + b, !inv);
            }
        };

        // value 1 is equivalent to 2, according to UG479
        // but in real life, Vivado sets AREG_0 is 0,
        // no bit is 1, and AREG_2 is 2
        auto areg = int_or_default(ci->params, ctx->id("AREG"), 1);
        if (areg == 0 || areg == 2)
            write_bit("AREG_" + std::to_string(areg));

        auto ainput = str_or_default(ci->params, ctx->id("A_INPUT"), "DIRECT");
        if (ainput == "CASCADE")
            write_bit("A_INPUT[0]");

        // value 1 is equivalent to 2, according to UG479
        // but in real life, Vivado sets AREG_0 is 0,
        // no bit is 1, and AREG_2 is 2
        auto breg = int_or_default(ci->params, ctx->id("BREG"), 1);
        if (breg == 0 || breg == 2)
            write_bit("BREG_" + std::to_string(breg));

        auto binput = str_or_default(ci->params, ctx->id("B_INPUT"), "DIRECT");
        if (binput == "CASCADE")
            write_bit("B_INPUT[0]");

        // Tolerate both int and string types for interoperability purposes
        auto use_dport = boolstr_or_default(ci->params, ctx->id("USE_DPORT"), false);
        if (use_dport == true)
            write_bit("USE_DPORT[0]");

        auto use_simd = str_or_default(ci->params, ctx->id("USE_SIMD"), "ONE48");
        if (use_simd == "TWO24")
            write_bit("USE_SIMD_FOUR12_TWO24");
        if (use_simd == "FOUR12")
            write_bit("USE_SIMD_FOUR12");

        // PATTERN
        const size_t pattern_size = 48;
        std::vector<bool> pattern_vector(pattern_size, false);
        bool pattern_found = boolvec_populate(ci->params, ctx->id("PATTERN"), pattern_vector);
        if (pattern_found) {
            write_vector("PATTERN[47:0]", pattern_vector);
        }

        auto autoreset_patdet = str_or_default(ci->params, ctx->id("AUTORESET_PATDET"), "NO_RESET");
        if (autoreset_patdet == "RESET_MATCH")
            write_bit("AUTORESET_PATDET_RESET");
        if (autoreset_patdet == "RESET_NOT_MATCH")
            write_bit("AUTORESET_PATDET_RESET_NOT_MATCH");

        // MASK
        // Yosys gives us 48 bit, but prjxray only recognizes 46 bits
        // The most significant two bits seem to be zero, so let us just truncate them
        const size_t mask_size = 48;
        std::vector<bool> mask_vector(mask_size, true);
        bool mask_found = boolvec_populate(ci->params, ctx->id("MASK"), mask_vector);
        if (mask_found) {
            mask_vector.resize(46);
            write_vector("MASK[45:0]", mask_vector);
        }

        auto sel_mask = str_or_default(ci->params, ctx->id("SEL_MASK"), "MASK");
        if (sel_mask == "C")
            write_bit("SEL_MASK_C");
        if (sel_mask == "ROUNDING_MODE1")
            write_bit("SEL_MASK_ROUNDING_MODE1");
        if (sel_mask == "ROUNDING_MODE2")
            write_bit("SEL_MASK_ROUNDING_MODE2");

        write_bit("ZADREG[0]", !bool_or_default(ci->params, ctx->id("ADREG"), true));
        write_bit("ZALUMODEREG[0]", !bool_or_default(ci->params, ctx->id("ALUMODEREG")));
        write_bit("ZAREG_2_ACASCREG_1", !bool_or_default(ci->params, ctx->id("ACASCREG")));
        write_bit("ZBREG_2_BCASCREG_1", !bool_or_default(ci->params, ctx->id("BCASCREG")));
        write_bit("ZCARRYINREG[0]", !bool_or_default(ci->params, ctx->id("CARRYINREG")));
        write_bit("ZCARRYINSELREG[0]", !bool_or_default(ci->params, ctx->id("CARRYINSELREG")));
        write_bit("ZCREG[0]", !bool_or_default(ci->params, ctx->id("CREG"), true));
        write_bit("ZDREG[0]", !bool_or_default(ci->params, ctx->id("DREG"), true));
        write_bit("ZINMODEREG[0]", !bool_or_default(ci->params, ctx->id("INMODEREG")));
        write_bus_zinv("ALUMODE", 4);
        write_bus_zinv("INMODE", 5);
        write_bus_zinv("OPMODE", 7);
        write_bit("ZMREG[0]", !bool_or_default(ci->params, ctx->id("MREG")));
        write_bit("ZOPMODEREG[0]", !bool_or_default(ci->params, ctx->id("OPMODEREG")));
        write_bit("ZPREG[0]", !bool_or_default(ci->params, ctx->id("PREG")));
        write_bit("USE_DPORT[0]", boolstr_or_default(ci->params, ctx->id("USE_DPORT"), false));
        write_bit("ZIS_CLK_INVERTED", !bool_or_default(ci->params, ctx->id("IS_CLK_INVERTED")));
        write_bit("ZIS_CARRYIN_INVERTED", !bool_or_default(ci->params, ctx->id("IS_CARRYIN_INVERTED")));
        pop(2);

        auto write_const_pins = [&](std::string const_net_name) {
            std::vector<std::string> pins;
            const auto attr_name = "DSP_" + const_net_name + "_PINS";
            const auto attr_value = str_or_default(ci->attrs, ctx->id(attr_name), "");
            boost::split(pins, attr_value, boost::is_any_of(" "));
            for (auto pin : pins) {
                if (boost::empty(pin))
                    continue;
                auto pin_basename = pin;
                boost::erase_all(pin_basename, "0123456789");
                auto inv = bool_or_default(ci->params, ctx->id("IS_" + pin_basename + "_INVERTED"), 0);
                auto net_name = inv ? (const_net_name == "GND" ? "VCC" : "GND") : const_net_name;
                write_bit(stringf("%s_%s.DSP_%s_%c", dsp.c_str(), pin.c_str(), net_name.c_str(), tile_side));
            }
        };

        write_const_pins("GND");
        write_const_pins("VCC");

        pop();
    }

    void write_ip()
    {
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            if (ci->type == id_DSP48E1_DSP48E1) {
                write_dsp_cell(ci);
                blank();
            }
        }
    }


    // ---------------------------------------------------------------------
    // UltraScale+ (prjuray), ported from nextpnr-xilinx's US+ FASM writer
    // (0-ultrascale-bits/patches/nextpnr-xilinx-usp-fasm.patch). Feature names
    // and the values below were checked there against Vivado bitstreams
    // decoded with bit2fasm; see SCOPE-K26.md in that repository.
    // ---------------------------------------------------------------------

    void write_choice(const std::string &name, const std::string &value)
    {
        write_prefix();
        out << name << "." << value << std::endl;
    }

    void write_bool_usp(const std::string &name, bool value) { write_choice(name, value ? "V1" : "V0"); }

    // prjuray names a site by its position among the same-prefix sites of
    // its tile -- HDIO_TOP_RIGHT.IOB_X0Y7, RCLK_INT_L.BUFCE_LEAF_X1Y0 --
    // which is what SiteInstPOD::rel_x/rel_y hold.
    std::string usp_site_name(SiteIndex site)
    {
        const auto &sd = uarch->tile_extra_data(site.tile)->sites[site.site];
        return stringf("%s_X%dY%d", IdString(sd.name_prefix).c_str(ctx), sd.rel_x, sd.rel_y);
    }

    // The site that a tile wire is a site pin of, found through the
    // SITE_ENTRY pip from that wire into the site (site wires carry their
    // site index in flags). Route-through pips carry no site of their own;
    // nextpnr-xilinx needed generated tables for this.
    int usp_site_of_input_wire(int tile, WireId wire)
    {
        for (auto p : ctx->getPipsDownhill(wire)) {
            if (p.tile != tile)
                continue;
            const auto &pd = chip_pip_info(ctx->chip_info, p);
            if (pd.flags != PIP_SITE_ENTRY)
                continue;
            return chip_tile_info(ctx->chip_info, p.tile).wires[pd.dst_wire].flags;
        }
        return -1;
    }

    // A bidirectional pip is in the chipdb twice, the reverse copy tagged
    // with pip_config bit 1. prjuray names BOTH directions by the canonical
    // (dst, src) pair plus .FWD/.REV -- INT.PIP.BYPASS_W8.INT_NODE_IMUX_37_INT_OUT0.REV
    // -- so the reverse copy has its names swapped back. Returns "" for a
    // one-way pip.
    std::string usp_pip_direction_suffix(PipId pip, bool &swap)
    {
        swap = false;
        const auto &pd = chip_pip_info(ctx->chip_info, pip);
        const auto &ed = *reinterpret_cast<const XlnxPipExtraDataPOD *>(pd.extra_data.get());
        if (ed.pip_config & 2) {
            swap = true;
            return ".REV";
        }
        WireId src = ctx->getPipSrcWire(pip);
        for (auto p : ctx->getPipsDownhill(ctx->getPipDstWire(pip))) {
            if (p.tile != pip.tile || ctx->getPipDstWire(p) != src)
                continue;
            const auto &rpd = chip_pip_info(ctx->chip_info, p);
            if (rpd.flags == PIP_TILE_ROUTING &&
                (reinterpret_cast<const XlnxPipExtraDataPOD *>(rpd.extra_data.get())->pip_config & 2))
                return ".FWD";
        }
        return "";
    }

    pool<int> usp_pss_outputs_enabled;
    // (X, Y) of each INT_INTF_L row that a BRAM output leaves through.
    pool<std::pair<int, int>> usp_bram_output_rows;

    // The X/Y of a tile name such as BRAM_X19Y150.
    static bool usp_tile_xy(const std::string &name, int &x, int &y)
    {
        auto p = name.rfind("_X");
        return p != std::string::npos && std::sscanf(name.c_str() + p, "_X%dY%d", &x, &y) == 2;
    }

    // How many inverting clock-spine drivers (CLK_CMT_DRVR_TRI_ESD_*_SCHMITT_B)
    // lie on the routed path from the clock's source to this pip.
    int usp_clock_inversions(PipId pip)
    {
        NetInfo *net = ctx->getBoundPipNet(pip);
        if (net == nullptr)
            return 0;
        int n = 0;
        WireId w = ctx->getPipSrcWire(pip);
        for (int guard = 0; guard < 100000; guard++) {
            auto it = net->wires.find(w);
            if (it == net->wires.end() || it->second.pip == PipId())
                break;
            PipId up = it->second.pip;
            auto &pd = chip_pip_info(ctx->chip_info, up);
            std::string dst = IdString(chip_tile_info(ctx->chip_info, up.tile).wires[pd.dst_wire].name).str(ctx);
            if (boost::contains(dst, "CLK_CMT_DRVR_TRI_ESD") && boost::ends_with(dst, "_SCHMITT_B"))
                n++;
            w = ctx->getPipSrcWire(up);
        }
        return n;
    }

    // UltraScale+ pips whose configuration is NOT a routing bit: mostly
    // route-throughs, selected by the passed site's own configuration, plus
    // the PS interface. prjuray has no pip feature for any of them, and
    // writing one assembles to nothing. Returns true when consumed.
    bool write_pip_as_site_config_usp(PipId pip, const std::string &tt, const std::string &src, const std::string &dst)
    {
        // The HDIOB buffers: configured by write_io_usp(); prjuray has no
        // HDIO_IOBPAIR pip feature at all.
        if (boost::starts_with(dst, "HDIO_IOBPAIR_") || boost::starts_with(src, "HDIO_IOBPAIR_"))
            return true;
        // The HDIO input flip-flop bypass has no feature of any kind.
        if (boost::contains(dst, "_IPFF") || boost::contains(src, "_IPFF"))
            return true;

        // The leaf clock buffer: CLK_LEAF_SITES_<n>_CLK_IN -> _CLK_LEAF passes
        // through a BUFCE_LEAF and is <site>.BUFCE_LEAF.IN_USE.V1. The other
        // four values are those of the in-use leaf in the Vivado reference
        // for kv260_ps_blink: IINV.V1 and DELAY_TAP.V0 each SET a bit, and an
        // all-clear DELAY_TAP matches no tap value.
        if ((tt == "RCLK_INT_L" || tt == "RCLK_INT_R") && boost::starts_with(dst, "CLK_LEAF_SITES_") &&
            boost::ends_with(dst, "_CLK_LEAF") && boost::ends_with(src, "_CLK_IN")) {
            int si = usp_site_of_input_wire(pip.tile, ctx->getPipSrcWire(pip));
            if (si < 0)
                log_error("no BUFCE_LEAF site found for %s.%s\n", uarch->tile_name(pip.tile).c_str(), src.c_str());
            push(uarch->tile_name(pip.tile));
            push(usp_site_name(SiteIndex(pip.tile, si)));
            push("BUFCE_LEAF");
            write_choice("IN_USE", "V1");
            write_choice("CE_TYPE", "SYNC");
            write_choice("CEINV", "V0");
            // The vertical clock spine's drivers (CLK_CMT_DRVR_TRI_ESD_*_SCHMITT_B)
            // invert. A region whose clock passed through an odd number of them
            // runs on the inverted clock unless its leaves invert it back, as
            // Vivado does (IINV.V0 in those regions, V1 where the count is even).
            // Without this, logic in such a region captured half a cycle early:
            // every K26 design spanning several clock regions failed on silicon
            // (0-ultrascale-bits designs/kv260_ps8_block_test_3reg).
            write_choice("IINV", (usp_clock_inversions(pip) & 1) ? "V0" : "V1");
            write_choice("DELAY_TAP", "V0");
            pop(3);
            return true;
        }

        // The PS interface: one tile-level enable (write_pss_intf_usp), set
        // by fuzzers/070-ps8-int for any tile with a LOGIC_OUTS pip in use.
        // The pips themselves have no bits. prjuray names the enable
        // OUTPUTS_ENABLED.1, which FASM cannot parse; 0-ultrascale-bits'
        // db-corrections name the same bits OUTPUTS_ENABLED, as on
        // INT_INTF_R_PCIE4.
        if (tt == "INT_INTF_LEFT_TERM_PSS") {
            if (boost::contains(dst, "LOGIC_OUTS") || boost::contains(src, "LOGIC_OUTS"))
                usp_pss_outputs_enabled.insert(pip.tile);
            return true;
        }

        // The clock tri-state/ESD driver: the track that drives it is carried
        // by WIRE.CLK_VDISTR_*.USED.V1 (write_clocking_usp).
        if (boost::contains(dst, "CLK_CMT_DRVR_TRI_ESD") && boost::ends_with(dst, "_SCHMITT_B"))
            return true;

        // The row clock buffer itself has no feature; its input mux does.
        if (boost::starts_with(dst, "CLK_BUFCE_ROW_FSR") && boost::ends_with(dst, "_CLK_OUT"))
            return true;

        // The BRAM tile's pin crossovers carry no bits: with the parameters
        // written by write_bram_usp() every set bit in the tile is accounted
        // for (designs/kv260_bram_spec against Vivado).
        if (tt == "BRAM" && boost::starts_with(dst, "BRAM_BRAM_CORE_"))
            return true;
        // A BRAM output crossing into the fabric. INT_INTF_L has no frames;
        // what it needs is the BRAM tile's per-row output enable, written by
        // write_bram_usp() for each row recorded here.
        if (tt == "INT_INTF_L" && boost::starts_with(dst, "LOGIC_OUTS_")) {
            int x, y;
            if (usp_tile_xy(uarch->tile_name(pip.tile), x, y))
                usp_bram_output_rows.insert({x, y});
            return true;
        }

        return false;
    }

    void write_pip_usp(PipId pip, std::string src, std::string dst)
    {
        const auto &pd = chip_pip_info(ctx->chip_info, pip);
        const auto &ed = *reinterpret_cast<const XlnxPipExtraDataPOD *>(pd.extra_data.get());
        std::string tt = IdString(chip_tile_info(ctx->chip_info, pip.tile).type_name).str(ctx);
        std::string tile_name = uarch->tile_name(pip.tile);

        if (write_pip_as_site_config_usp(pip, tt, src, dst))
            return;

        // The HDIO output flip-flop bypass is two features: the pip, and the
        // site's output mux (OQ_MUX.NOT_OPTFF). Writing only the pip leaves
        // the mux bits matching neither choice; Vivado sets both.
        if (boost::contains(dst, "_OPFF") && boost::ends_with(dst, "_Q") && boost::contains(src, "_OPFF") &&
            boost::ends_with(src, "_D1")) {
            int si = usp_site_of_input_wire(pip.tile, ctx->getPipSrcWire(pip));
            if (si < 0)
                log_error("no HDIOLOGIC site found for %s.%s\n", tile_name.c_str(), src.c_str());
            push(tile_name);
            push(usp_site_name(SiteIndex(pip.tile, si)));
            write_choice("OQ_MUX", "NOT_OPTFF");
            pop(2);
        } else if (ed.pip_config & 1) {
            log_warning("Unprocessed route-thru %s.%s.%s\n", tt.c_str(), src.c_str(), dst.c_str());
        }

        // An INT_INTF_R_PCIE4 passes nothing from its site into the fabric
        // (LOGIC_OUTS_R -> LOGIC_OUTS_L) unless the tile's OUTPUTS_ENABLED is
        // set: without it an ICAPE3 reads as all-zero. Vivado sets it on
        // exactly the tiles whose LOGIC_OUTS are used. The CMT column's
        // interface tiles (LOGIC_OUTS_CMT -> LOGIC_OUTS_R: an MMCM's LOCKED)
        // have the same bits (0-ultrascale-bits/designs/kv260_cmt_char).
        if (((tt == "INT_INTF_R_PCIE4" && boost::starts_with(dst, "LOGIC_OUTS_L")) ||
             ((tt == "INT_INTF_L_CMT" || tt == "INT_INTF_L_IO") && boost::starts_with(dst, "LOGIC_OUTS_R"))) &&
            outputs_enabled_tiles.insert(pip.tile).second)
            out << tile_name << ".OUTPUTS_ENABLED" << std::endl;

        // prjuray spells a routing bit TILE.PIP.DST.SRC (segbits_int.db).
        bool swap = false;
        std::string dir_suffix = usp_pip_direction_suffix(pip, swap);
        if (swap)
            std::swap(dst, src);
        out << tile_name << ".PIP." << dst << "." << src << dir_suffix << std::endl;
        last_was_blank = false;
    }

    // A wire inside the SLICE of a CLE tile, or WireId() if there is none.
    WireId usp_slice_wire(int tile, const std::string &name)
    {
        BelId site_bel = ctx->getBelByLocation(Loc(tile % ctx->chip_info->width, tile / ctx->chip_info->width, 0));
        if (site_bel == BelId())
            return WireId();
        IdStringList bel_name = ctx->getBelName(site_bel);
        const std::string &bn = bel_name[1].str(ctx);
        std::string site = bn.substr(0, bn.find('.'));
        return ctx->getWireByName(IdStringList::concat(bel_name[0], ctx->idf("%s.%s", site.c_str(), name.c_str())));
    }

    void write_luts_config_usp(int tile)
    {
        std::string tname = uarch->tile_name(tile);
        // CLEM / CLEM_R hold a SLICEM; CLEL_L / CLEL_R a SLICEL.
        bool is_slicem = boost::starts_with(tname, "CLEM");
        const auto &lts = uarch->tile_status.at(tile).lts;
        const char *letters = "ABCDEFGH";

        push(tname);
        bool wa7_used = false, wa8_used = false, wa9_used = false;
        bool any_lut = false;
        for (int i = 0; i < 8; i++) {
            CellInfo *lut6 = lts ? lts->cells[(i << 4) | BEL_6LUT] : nullptr;
            CellInfo *lut5 = lts ? lts->cells[(i << 4) | BEL_5LUT] : nullptr;
            auto found_rt = lut_route_throughs.find(std::make_pair(tile, i));
            if (lut6 == nullptr && lut5 == nullptr && found_rt != lut_route_throughs.end()) {
                // A LUT used as a route-through passes one input to O6.
                push(stringf("%cLUT", letters[i]));
                std::vector<bool> rt_init(64, false);
                for (unsigned b = 0; b < 64; b++)
                    rt_init[b] = (b & (1U << found_rt->second)) != 0;
                write_vector("INIT[63:0]", rt_init);
                pop();
                any_lut = true;
            }
            if (lut6 != nullptr || lut5 != nullptr) {
                any_lut = true;
                push(stringf("%cLUT", letters[i]));
                write_vector("INIT[63:0]", get_lut_init(lut6, lut5));
                bool is_small = false, is_ram = false, is_srl = false;
                for (int j = 0; j < 2; j++) {
                    CellInfo *lut = (j == 1) ? lut5 : lut6;
                    if (lut == nullptr)
                        continue;
                    std::string type = str_or_default(lut->attrs, id_X_ORIG_TYPE);
                    if (type == "RAMD64E" || type == "RAMS64E") {
                        is_ram = true;
                    } else if (type == "RAMD32" || type == "RAMS32") {
                        is_ram = true;
                        is_small = true;
                    } else if (type == "SRL16E") {
                        is_srl = true;
                        is_small = true;
                    } else if (type == "SRLC32E") {
                        is_srl = true;
                    }
                    wa7_used |= (lut->getPort(id_WA7) != nullptr);
                    wa8_used |= (lut->getPort(id_WA8) != nullptr);
                    wa9_used |= (lut->getPort(id_WA9) != nullptr);
                }
                // MODE and SIZE exist only where the LUT can be memory; prjuray
                // has no CLEL entry for either.
                if (is_slicem) {
                    write_choice("MODE", is_ram ? "RAM" : is_srl ? "SRL" : "LUT");
                    write_choice("SIZE", is_small ? "SMALL" : "LARGE");
                } else if (is_ram || is_srl) {
                    log_error("LUT memory placed on a SLICEL in tile %s\n", tname.c_str());
                }
                pop();
            }
            // OUTMUX<c>: the chipdb's site pip names are prjuray's.
            WireId outmux = usp_slice_wire(tile, stringf("%cMUX", letters[i]));
            if (outmux != WireId())
                write_routing_bel(outmux);
        }
        if (any_lut && is_slicem) {
            write_bool_usp("WA7USED", wa7_used);
            write_bool_usp("WA8USED", wa8_used);
            write_bool_usp("WA9USED", wa9_used);
        }
        pop();
    }

    void write_ffs_config_usp(int tile)
    {
        const auto &lts = uarch->tile_status.at(tile).lts;
        if (!lts)
            return;
        push(uarch->tile_name(tile));

        // Per-half control state: ABCD and EFGH each share a clock, set/reset
        // and two clock enables. Only rank 1 carries CLKINV/SRINV/MODE/SYNC.
        struct CtrlGroup
        {
            bool used = false;
            bool latch = false, sync = false, clkinv = false, srinv = false;
            bool srused[2] = {false, false};
            bool ceused[2] = {false, false};
            bool rank_used[2] = {false, false};
        } group[2];

        for (int i = 0; i < 8; i++) {
            int half = i / 4;
            for (int j = 0; j < 2; j++) {
                CellInfo *ff = lts->cells[(i << 4) | (j == 1 ? BEL_FF2 : BEL_FF)];
                if (ff == nullptr)
                    continue;
                bool zrst = false;
                bool zinit = (int_or_default(ff->params, id_INIT, 0) != 1);
                bool is_latch = false, is_sync = false;
                std::string type = str_or_default(ff->attrs, id_X_ORIG_TYPE, "");
                if (type == "FDRE" || type == "FDRE_1") {
                    zrst = true;
                    is_sync = true;
                } else if (type == "FDSE" || type == "FDSE_1") {
                    is_sync = true;
                } else if (type == "FDCE" || type == "FDCE_1") {
                    zrst = true;
                } else if (type == "FDPE" || type == "FDPE_1") {
                    // async set
                } else if (type == "LDCE" || type == "LDPE") {
                    zrst = (type == "LDCE");
                    is_latch = true;
                } else {
                    log_error("unsupported FF type: '%s'\n", type.c_str());
                }
                bool negedge_ff = boost::ends_with(type, "_1");

                // Both values carry bits, so always write one.
                push(get_bel_name(ff->bel));
                write_choice("INIT", zinit ? "V0" : "V1");
                write_choice("SRVAL", zrst ? "V0" : "V1");
                pop();

                NetInfo *sr = ff->getPort(id_SR), *ce = ff->getPort(id_CE);
                auto &g = group[half];
                g.used = true;
                g.rank_used[j] = true;
                g.latch = g.latch || is_latch;
                g.sync = g.sync || is_sync;
                g.clkinv = g.clkinv || negedge_ff || int_or_default(ff->params, id_IS_C_INVERTED) == 1;
                // yosys spells "no reset" as SR tied to VCC with IS_R_INVERTED=1:
                // either constant is an unused reset.
                bool sr_is_const = sr == nullptr || sr->name == ctx->id("$PACKER_GND_NET") ||
                                   sr->name == ctx->id("$PACKER_VCC_NET");
                g.srused[j] = g.srused[j] || !sr_is_const;
                if (!sr_is_const)
                    g.srinv = g.srinv || int_or_default(ff->params, id_IS_R_INVERTED) == 1 ||
                              int_or_default(ff->params, id_IS_S_INVERTED) == 1;
                g.ceused[j] = g.ceused[j] || (ce != nullptr && ce->name != ctx->id("$PACKER_VCC_NET"));

                // FFMUX<c><1|2> input select.
                write_routing_bel(ctx->getBelPinWire(ff->bel, id_D));
            }
        }

        for (int half = 0; half < 2; half++) {
            auto &g = group[half];
            if (!g.used)
                continue;
            std::string base = half ? "EFGH" : "ABCD";
            push(base + "FF");
            write_choice("MODE", g.latch ? "LATCH" : "FF");
            write_choice("SYNC", g.sync ? "SYNC" : "ASYNC");
            write_bool_usp("CLKINV", g.clkinv);
            write_bool_usp("SRINV", g.srinv);
            write_bool_usp("CEUSED", g.ceused[0]);
            write_bool_usp("SRUSED", g.srused[0]);
            pop();
            if (g.rank_used[1]) {
                push(base + "FF2");
                write_bool_usp("CEUSED", g.ceused[1]);
                write_bool_usp("SRUSED", g.srused[1]);
                pop();
            }
        }
        pop();
    }

    // CARRY8 mode and input selects, names from prjuray's segbits_cle*.db.
    //
    // There is no CI state for a chained CARRY8. Vivado's own chains decode as
    // CARRY8.CI.V0 plus the tile pip CLE_CLE_*_SITE_0_CIN_PIN.CLE_CLE_*_SITE_0_CIN,
    // and the router writes that pip when it routes the chain net to CIN.
    void write_carry_config_usp(int tile)
    {
        const auto &lts = uarch->tile_status.at(tile).lts;
        if (!lts)
            return;
        CellInfo *carry = lts->cells[BEL_CARRY8];
        if (carry == nullptr)
            return;
        const char *letters = "ABCDEFGH";

        push(uarch->tile_name(tile));
        push("CARRY8");
        write_choice("CARRY_TYPE", str_or_default(carry->params, id_CARRY_TYPE, "SINGLE_CY8"));
        auto ci_const = carry->attrs.find(ctx->id("X_CARRY8_CI_CONST"));
        if (ci_const != carry->attrs.end())
            write_choice("CI", ci_const->second.as_int64() ? "V1" : "V0");
        else if (carry->getPort(id_AX) != nullptr)
            write_choice("CI", "AX");
        else
            write_choice("CI", "V0");
        write_choice("CI_TOP", carry->getPort(id_EX) != nullptr ? "EX" : "V0");

        // DI<n> picks the <c>5LUT's O5 or the <c>X slice input.
        for (int i = 0; i < 8; i++) {
            NetInfo *di = carry->getPort(ctx->idf("DI%d", i));
            if (di == nullptr || di->driver.cell == nullptr)
                continue;
            BelId drv_bel = di->driver.cell->bel;
            bool from_lut = drv_bel != BelId() && drv_bel.tile == carry->bel.tile &&
                            ctx->getBelLocation(drv_bel).z == ((i << 4) | BEL_5LUT);
            write_choice(stringf("DI%d", i), from_lut ? stringf("DI%d", i) : stringf("%cX", letters[i]));
        }
        pop(2);
    }

    pool<int> usp_hv_banks, usp_lv_banks;

    void write_io_config_usp(CellInfo *pad)
    {
        NetInfo *pad_net = pad->getPort(id_PAD);
        NPNR_ASSERT(pad_net != nullptr);
        std::string iostandard = str_or_default(pad->attrs, id_IOSTANDARD, "LVCMOS33");
        std::string pulltype = str_or_default(pad->attrs, id_PULLTYPE, "NONE");
        std::string slew = str_or_default(pad->attrs, id_SLEW, "SLOW");
        int drive = int_or_default(pad->attrs, id_DRIVE, 12);

        bool is_output = pad_net->driver.cell != nullptr;
        bool is_input = false;
        for (auto &usr : pad_net->users)
            if (usr.cell->type == id_IOB_INBUF)
                is_input = true;

        std::string tile = uarch->tile_name(pad->bel.tile);
        push(tile);
        push(usp_site_name(uarch->get_bel_site(pad->bel)));
        if (is_input)
            write_choice("IOSTANDARD_IN", iostandard);
        if (is_output) {
            // 031-iob-spec folds drive and slew into the name:
            // LVCMOS33_IDRIVE_I12_SLEW_SLEW_SLOW.
            write_choice("IOSTANDARD_OUT", iostandard + "_IDRIVE_I" + std::to_string(drive) + "_SLEW_SLEW_" + slew);
            if (is_input)
                write_choice("IOSTANDARD_IN_OUT", iostandard);
        }
        write_choice("PULLTYPE", pulltype);
        pop(2);

        // Bank voltage is a per-bank bit in the HDIO_TOP_RIGHT tile.
        if (boost::starts_with(tile, "HDIO_TOP_RIGHT")) {
            bool low = iostandard == "LVCMOS12" || iostandard == "LVCMOS15" || iostandard == "LVCMOS18";
            (low ? usp_lv_banks : usp_hv_banks).insert(pad->bel.tile);
        }
    }

    void write_io_usp()
    {
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            if (ci->type == id_IOB_PAD && ci->bel != BelId()) {
                write_io_config_usp(ci);
                blank();
            }
        }
        for (int tile : usp_hv_banks) {
            push(uarch->tile_name(tile));
            write_choice("BANK_VOLTAGE", "HIGH_VOLTAGE");
            pop();
        }
        for (int tile : usp_lv_banks) {
            if (usp_hv_banks.count(tile))
                log_error("tile %s has both a low- and a high-voltage IO standard\n", uarch->tile_name(tile).c_str());
            push(uarch->tile_name(tile));
            write_choice("BANK_VOLTAGE", "LOW_VOLTAGE");
            pop();
        }
        blank();
    }

    // Clock distribution and route tracks carry a per-tile "in use" bit.
    // dump_features.tcl applies it to two wire sets depending on tile type.
    static bool usp_wire_used_feature(const std::string &tt, const std::string &wire)
    {
        static const pool<std::string> group_a = {
                "RCLK_BRAM_INTF_L", "RCLK_BRAM_INTF_TD_L", "RCLK_BRAM_INTF_TD_R", "RCLK_CLEL_L_L",
                "RCLK_CLEL_L",      "RCLK_CLEL_L_R",       "RCLK_CLEM_CLKBUF_L",  "RCLK_CLEM_L",
                "RCLK_DSP_INTF_L",  "RCLK_DSP_INTF_R",     "RCLK_RCLK_URAM_INTF_L_FT", "RCLK_CLEM_R",
                "RCLK_CLEL_R",      "GTH_QUAD_RIGHT"};
        static const pool<std::string> group_b = {
                "RCLK_DSP_INTF_CLKBUF_L", "RCLK_CLEM_CLKBUF_L", "RCLK_AMS_CFGIO",
                "RCLK_INTF_LEFT_TERM_ALTO", "RCLK_XIPHY_OUTER_RIGHT", "RCLK_RCLK_XIPHY_INNER_FT",
                "RCLK_HDIO", "GTH_QUAD_RIGHT"};
        if (group_a.count(tt))
            if (boost::starts_with(wire, "CLK_TEST_BUF_SITE_") || boost::starts_with(wire, "CLK_HROUTE_CORE_OPT") ||
                boost::starts_with(wire, "CLK_VDISTR_BOT") || boost::starts_with(wire, "CLK_VDISTR_TOP") ||
                boost::starts_with(wire, "CLK_VROUTE_BOT") || boost::starts_with(wire, "CLK_VROUTE_TOP"))
                return true;
        if (group_b.count(tt))
            if (boost::starts_with(wire, "CLK_HROUTE") || boost::starts_with(wire, "CLK_HDISTR"))
                return true;
        if (tt == "CMT_L" || tt == "CMT_RIGHT")
            if (boost::starts_with(wire, "CLK_HROUTE_") || boost::starts_with(wire, "CLK_HDISTR_") ||
                boost::starts_with(wire, "CLK_VDISTR_") || boost::starts_with(wire, "CLK_TEST_BUF_SITE_"))
                return true;
        return false;
    }

    void write_clocking_usp()
    {
        // A distribution wire is a node spanning every tile in the row, so
        // walk all of its tile wires: the track is driven along its length.
        std::set<std::pair<int, std::string>> emitted;
        for (auto &net : ctx->nets) {
            NetInfo *ni = net.second.get();
            for (auto &w : ni->wires) {
                for (WireId tw : ctx->get_tile_wire_range(w.first)) {
                    const auto &ti = chip_tile_info(ctx->chip_info, tw.tile);
                    std::string tts = IdString(ti.type_name).str(ctx);
                    std::string wname = IdString(ti.wires[tw.index].name).str(ctx);
                    if (!usp_wire_used_feature(tts, wname))
                        continue;
                    if (!emitted.insert({tw.tile, wname}).second)
                        continue;
                    push(uarch->tile_name(tw.tile));
                    push("WIRE");
                    write_choice(wname + ".USED", "V1");
                    pop(2);
                }
            }
        }
        blank();
    }

    // The PS interface tile's one enable; without it every fabric output on
    // the PS side is dead (segbits_int_intf_left_term_pss.db holds nothing
    // else).
    // CONFIG_SITE (ICAPE3, STARTUPE3, BSCANE2, ...). A placed primitive is
    // <tile>.<BEL>.IN_USE (plus STARTUP.PROG_USR), as characterised in
    // 0-ultrascale-bits/designs/kv260_cfg_char; the bits are
    // db-corrections/zynqusp/segbits_cfg_config.corrections.
    void write_config_usp()
    {
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            // FRAME_ECC, EFUSE_USR and USR_ACCESS set no site bits in Vivado's
            // bitstreams, nor does ICAP_AUTO_SWITCH: only their routing is
            // configured.
            if (!ci->type.in(id_ICAP_TOP, id_ICAP_BOT, id_STARTUP, id_BSCAN, id_DNA_PORT) ||
                ci->bel == BelId())
                continue;
            std::string bel = ctx->getBelName(ci->bel)[1].str(ctx);
            bel = bel.substr(bel.rfind('.') + 1);
            push(uarch->tile_name(ci->bel.tile));
            push(bel);
            write_bit("IN_USE");
            if (ci->type == id_STARTUP)
                write_bit("PROG_USR", str_or_default(ci->params, id_PROG_USR, "FALSE") == "TRUE");
            pop(2);
        }
        blank();
    }

    // DSP48E2 (0-ultrascale-bits/designs/kv260_dsp_char; segbits_dsp.corrections
    // from emit_dsp.py, which predicts every bit of Vivado's DSP tiles). One
    // DSP_ALU cell stands for each primitive; it carries the parameters. The
    // features are <tile>.DSP48E2_X0Y<0|1>.<f>, plus <tile>.IN_USE once a
    // tile has a DSP. Vivado normalises before writing, and so does this:
    // MASK reads as all ones and SEL_MASK as MASK without pattern detect,
    // ADREG/DREG count only with the pre-adder in use (ZADREG/ZDREG set when
    // they do not). An invertible pin writes ZINV_<pin> = !IS_INVERTED, or
    // for a pin tied by the packer (X_TIE_<pin>) the constant itself; a tied
    // CARRYINSEL bit has its own CARRYINSEL_TIE0/TIE1 bits, and so has a
    // tied D bit (D_TIE0/TIE1; a tied A, B or C bit is routed).
    void write_dsp_usp()
    {
        // A used output turns on its INT row's interface (OUT_ROW<r>, a tile
        // feature): usp_dsp_rows.h. No other bit marks a DSP in use.
        dict<std::pair<int, IdString>, int> out_row;
        for (const auto &r : usp_dsp_rows)
            out_row[{r.site, ctx->id(r.pin)}] = r.row;
        dict<int, std::set<int>> tile_rows;
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            if (ci->type != id_DSP_OUTPUT || ci->bel == BelId())
                continue;
            auto site = uarch->get_bel_site(ci->bel);
            int pos = uarch->tile_extra_data(site.tile)->sites[site.site].rel_y;
            for (auto &port : ci->ports) {
                if (port.second.type != PORT_OUT || port.second.net == nullptr || port.second.net->users.empty())
                    continue;
                auto found = out_row.find({pos, port.first});
                if (found != out_row.end())
                    tile_rows[ci->bel.tile].insert(found->second);
            }
        }
        for (auto &t : tile_rows) {
            push(uarch->tile_name(t.first));
            for (int r : t.second)
                write_bit(stringf("OUT_ROW%d", r));
            pop();
        }
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            if (ci->type != id_DSP_ALU || ci->bel == BelId())
                continue;
            std::string tile = uarch->tile_name(ci->bel.tile);
            if (!boost::starts_with(tile, "DSP_"))
                log_error("DSP48E2 '%s' is on %s; only DSP tiles are characterised\n", ctx->nameOf(ci),
                          ctx->nameOfBel(ci->bel));
            auto str = [&](const char *k, const char *def) { return str_or_default(ci->params, ctx->id(k), def); };
            auto num = [&](const char *k, int64_t def) -> int64_t {
                auto found = ci->params.find(ctx->id(k));
                if (found == ci->params.end())
                    return def;
                return found->second.is_string ? std::stoll(found->second.as_string()) : found->second.as_int64();
            };
            push(tile);
            push(usp_site_name(uarch->get_bel_site(ci->bel)));
            int64_t areg = num("AREG", 1), breg = num("BREG", 1);
            write_bit("AREG_0", areg == 0);
            write_bit("AREG_2", areg == 2);
            write_bit("ACASCREG_EQ_AREG", num("ACASCREG", 1) == areg);
            write_bit("BREG_0", breg == 0);
            write_bit("BREG_2", breg == 2);
            write_bit("BCASCREG_EQ_BREG", num("BCASCREG", 1) == breg);
            for (auto r : {"ALUMODEREG", "CARRYINREG", "CARRYINSELREG", "CREG", "INMODEREG", "MREG", "OPMODEREG",
                           "PREG"})
                write_bit(std::string(r) + "_0", num(r, 1) == 0);
            std::string use_mult = str("USE_MULT", "MULTIPLY"), amultsel = str("AMULTSEL", "A"),
                        bmultsel = str("BMULTSEL", "B");
            bool preadd = use_mult != "NONE" && (amultsel == "AD" || bmultsel == "AD");
            write_bit("ZADREG", !(preadd && num("ADREG", 1) == 1));
            write_bit("ZDREG", !(preadd && num("DREG", 1) == 1));
            write_bit("AMULTSEL.AD", amultsel == "AD");
            write_bit("BMULTSEL.AD", bmultsel == "AD");
            write_bit("PREADDINSEL.B", str("PREADDINSEL", "A") == "B");
            write_bit("A_INPUT.CASCADE", str("A_INPUT", "DIRECT") == "CASCADE");
            write_bit("B_INPUT.CASCADE", str("B_INPUT", "DIRECT") == "CASCADE");
            std::string autoreset = str("AUTORESET_PATDET", "NO_RESET");
            if (autoreset != "NO_RESET")
                write_bit("AUTORESET_PATDET." + autoreset);
            write_bit("AUTORESET_PRIORITY.CEP", str("AUTORESET_PRIORITY", "RESET") == "CEP");
            write_bit("SEL_PATTERN.C", str("SEL_PATTERN", "PATTERN") == "C");
            bool patdet = str("USE_PATTERN_DETECT", "NO_PATDET") == "PATDET";
            std::string sel_mask = patdet ? str("SEL_MASK", "MASK") : "MASK";
            if (sel_mask != "MASK")
                write_bit("SEL_MASK." + sel_mask);
            write_bit("USE_MULT.NONE", use_mult == "NONE");
            std::string simd = str("USE_SIMD", "ONE48");
            if (simd != "ONE48")
                write_bit("USE_SIMD." + simd);
            write_bit("USE_WIDEXOR.TRUE", str("USE_WIDEXOR", "FALSE") == "TRUE");
            write_bit("XORSIMD.XOR12", str("XORSIMD", "XOR24_48_96") == "XOR12");
            uint64_t mask = num("MASK", 0x3FFFFFFFFFFFLL), pattern = num("PATTERN", 0), rnd = num("RND", 0);
            for (int k = 0; k < 48; k++) {
                write_bit(stringf("MASK[%d]", k), !patdet || (mask >> k & 1));
                write_bit(stringf("PATTERN[%d]", k), pattern >> k & 1);
                write_bit(stringf("RND[%d]", k), rnd >> k & 1);
            }
            static const std::vector<std::pair<std::string, int>> inv_pins = {
                    {"ALUMODE", 4}, {"CARRYIN", 1},       {"INMODE", 5},     {"OPMODE", 9},
                    {"RSTA", 1},    {"RSTALLCARRYIN", 1}, {"RSTALUMODE", 1}, {"RSTB", 1},
                    {"RSTC", 1},    {"RSTCTRL", 1},       {"RSTD", 1},       {"RSTINMODE", 1},
                    {"RSTM", 1},    {"RSTP", 1}};
            for (auto &ip : inv_pins) {
                int64_t inv = num(("IS_" + ip.first + "_INVERTED").c_str(), 0);
                for (int b = 0; b < ip.second; b++) {
                    std::string pin = ip.second > 1 ? stringf("%s[%d]", ip.first.c_str(), b) : ip.first;
                    bool i = inv >> b & 1;
                    auto tie = ci->attrs.find(ctx->id("X_TIE_" + pin));
                    bool zinv = tie == ci->attrs.end() ? !i : ((tie->second.as_int64() != 0) != i);
                    write_bit("ZINV_" + pin, zinv);
                }
            }
            for (int b = 0; b < 3; b++) {
                auto tie = ci->attrs.find(ctx->id(stringf("X_TIE_CARRYINSEL[%d]", b)));
                if (tie != ci->attrs.end())
                    write_bit(stringf("CARRYINSEL_TIE%d[%d]", tie->second.as_int64() != 0 ? 1 : 0, b));
            }
            // D[3], D[13] and D[23] tied to 1 set no bit (kv260_dsp_char,
            // over 200 ties each): there is no D_TIE1 feature for them.
            for (int b = 0; b < 27; b++) {
                auto tie = ci->attrs.find(ctx->id(stringf("X_TIE_D[%d]", b)));
                if (tie == ci->attrs.end())
                    continue;
                bool one = tie->second.as_int64() != 0;
                if (!(one && (b == 3 || b == 13 || b == 23)))
                    write_bit(stringf("D_TIE%d[%d]", one ? 1 : 0, b));
            }
            write_bit("ZINV_CLK", !num("IS_CLK_INVERTED", 0));
            pop(2);
        }
        blank();
    }

    // The CMT_L clock buffers' site attributes, as characterised in
    // 0-ultrascale-bits/designs/kv260_cmt_char (segbits_cmt_l.corrections),
    // each <tile>.<SITE_XnYm>.<ATTR>:
    //   BUFGCE      CE_TYPE.ASYNC, IS_CE_INVERTED
    //   BUFGCE_DIV  BUFGCE_DIVIDE.D<n> (DIVIDE-1 in three bits), IS_CE_INVERTED,
    //               IS_CLR_INVERTED
    //   BUFGCTRL    INIT_OUT, PRESELECT_I1, IS_{CE,S,IGNORE}{0,1}_INVERTED
    // A site in use, CE_TYPE SYNC and PRESELECT_I0 set no bits. The clock
    // inputs' IS_I*_INVERTED set none either (Vivado folds them into the
    // driver), and HARDSYNC is refused by Vivado's own timing library, so all
    // of those are refused here rather than dropped.
    // UltraScale+ MMCM/PLL (0-ultrascale-bits/designs/kv260_mmcm_char):
    // IN_USE, the pin polarities, STARTUP_WAIT, a location bit, the DRP
    // registers (usp_drp.h), and the ties Vivado makes inside the CMT for
    // internal feedback and an unused CLKIN2. Feature names are
    // segbits_cmt_l.corrections'.
    void write_pll_usp(CellInfo *ci)
    {
        bool mmcm = ci->type == id_MMCM_MMCM_TOP;
        std::string tile = uarch->tile_name(ci->bel.tile);
        std::string site = usp_site_name(uarch->get_bel_site(ci->bel));
        if (!boost::starts_with(tile, "CMT_L") || (mmcm ? site != "MMCM_X0Y0" : (site != "PLL_X0Y0" && site != "PLL_X0Y1")))
            log_error("'%s' is on %s; only the CMT_L MMCM/PLL sites are characterised\n", ctx->nameOf(ci),
                      ctx->nameOfBel(ci->bel));
        // The CMT's row, counted from the bottom by the tile name's Y. Vivado
        // sets ROW_MID[0] (DRP 0x00 bit 0) from the routed CLKIN delay, in
        // bands of about 270 ps (kv260_mmcm_char/delay.tcl, the root sweep),
        // which needs a delay model nextpnr has no US+ data for. The two
        // middle rows set it with the usual clock root. It is not inert:
        // kv260_mmcm_phase measures CLKOUT0 5.0 ns away with it cleared
        // (half the 10 ns CLKIN period in that test, M 10, O 10). Lock and
        // frequency do not change, and UG572 promises no CLKIN/CLKOUT phase
        // relationship with internal feedback.
        auto name_y = [](const std::string &name) { return std::stoi(name.substr(name.rfind('Y') + 1)); };
        IdString cmt_l = IdString(chip_tile_info(ctx->chip_info, ci->bel.tile).type_name);
        int row = 0;
        for (int t = 0; t < ctx->chip_info->tile_insts.ssize(); t++)
            if (IdString(chip_tile_info(ctx->chip_info, t).type_name) == cmt_l &&
                name_y(uarch->tile_name(t)) < name_y(tile))
                row++;
        auto inv = [&](const std::string &pin) {
            return bool_or_default(ci->params, ctx->id("IS_" + pin + "_INVERTED"), false);
        };
        // ZINV_<pin> is set when the pin, left unrouted (tied), reads 0.
        auto zinv = [&](const std::string &pin) {
            return !((int_or_default(ci->params, ctx->id("X_TIE_" + pin), 0) != 0) ^ inv(pin));
        };
        UspDrp::Cell c;
        c.mmcm = mmcm;
        c.num = [&](const std::string &k, double d) { return float_or_default(ci, k, d); };
        c.str = [&](const std::string &k, const std::string &d) { return str_or_default(ci->params, ctx->id(k), d); };
        c.connected = [&](const std::string &o) {
            NetInfo *n = ci->getPort(ctx->id(o));
            return n != nullptr && n->users.entries() > 0;
        };
        auto regs = UspDrp::registers(c);
        if (regs.empty())
            log_error("'%s': CLKFBOUT_MULT outside the lock/filter tables\n", ctx->nameOf(ci));

        push(tile);
        push(site);
        write_bit("IN_USE");
        // A tied RST is unrouted, like PWRDWN: the bit then says it reads 0.
        write_bit("IS_RST_INVERTED", ci->params.count(ctx->id("X_TIE_RST")) ? zinv("RST") : inv("RST"));
        write_bit("ZINV_PWRDWN", zinv("PWRDWN"));
        if (mmcm) {
            write_bit("IS_CLKINSEL_INVERTED", inv("CLKINSEL"));
            write_bit("ZINV_PSEN", zinv("PSEN"));
            write_bit("ZINV_PSINCDEC", zinv("PSINCDEC"));
        }
        write_bit("STARTUP_WAIT", str_or_default(ci->params, ctx->id("STARTUP_WAIT"), "FALSE") == "TRUE");
        bool mid = row == 1 || row == 2;
        write_bit("ROW_MID[0]", mid);
        if (mmcm)
            write_bit("ROW_MID[1]"); // set in mid rows, and whenever CLKIN2 is unused (always, here)
        for (auto &r : regs)
            write_int_vector(r.first + "[15:0]", r.second, 16);
        pop();
        if (mmcm) {
            write_bit("PIP.CMT_MMCM_WITH_MEM_1_CLKFBIN.VCC_WIRE52");
            write_bit("PIP.CMT_MMCM_WITH_MEM_1_CLKIN2.VCC_WIRE");
        } else {
            bool odd = site == "PLL_X0Y1";
            std::string mux = odd ? "CLK_CMT_MUX_24_ENC_59_CLK_OUT" : "CLK_CMT_MUX_24_ENC_57_CLK_OUT";
            write_bit(std::string("PIP.CMT_TXPLL_WITH_MEM_") + (odd ? "87" : "86") + "_CLKFBIN." + mux);
            write_bit("PIP." + mux + ".VCC_WIRE");
        }
        pop();
    }

    // URAM288 (0-ultrascale-bits designs/kv260_uram_char: segbits from 30
    // random specimens, every tile predicted exactly). Features are per site,
    // URAM288_X0Y0..3 within the tile. SELF_MASK is stored inverted
    // (ZSELF_MASK); IS_<pin>_INVERTED is the inversion in effect, a tied pin
    // having been fed VCC by pack_constants. AVG_CONS_INACTIVE_CYCLES sets
    // no bits.
    void write_uram_usp()
    {
        // A used output turns on its INT row's interface on its side
        // (OUT_<A|B>_ROW<r>, a tile feature): usp_uram_rows.h.
        dict<std::pair<int, IdString>, std::pair<char, int>> out_row;
        for (const auto &r : usp_uram_rows)
            out_row[{r.site, ctx->id(r.pin)}] = {r.side, r.row};
        dict<int, std::set<std::pair<char, int>>> tile_rows;
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            if (ci->type != id_BEL_URAM288 || ci->bel == BelId())
                continue;
            auto str = [&](const char *k, const char *def) { return str_or_default(ci->params, ctx->id(k), def); };
            auto num = [&](const std::string &k, int64_t def) -> int64_t {
                auto found = ci->params.find(ctx->id(k));
                if (found == ci->params.end())
                    return def;
                return found->second.is_string ? std::stoll(found->second.as_string()) : found->second.as_int64();
            };
            auto site = uarch->get_bel_site(ci->bel);
            int pos = uarch->tile_extra_data(site.tile)->sites[site.site].rel_y;
            for (auto &port : ci->ports) {
                if (port.second.type != PORT_OUT || port.second.net == nullptr || port.second.net->users.empty())
                    continue;
                auto found = out_row.find({pos, port.first});
                if (found != out_row.end())
                    tile_rows[ci->bel.tile].insert(found->second);
            }
            push(uarch->tile_name(ci->bel.tile));
            push(usp_site_name(site));
            write_bit("IN_USE");
            for (std::string p : {"A", "B"}) {
                write_bit("BWE_MODE_" + p + ".PARITY_INDEPENDENT",
                          str(("BWE_MODE_" + p).c_str(), "PARITY_INTERLEAVED") == "PARITY_INDEPENDENT");
                for (std::string b : {"EN_ECC_RD_", "EN_ECC_WR_", "IREG_PRE_", "OREG_", "OREG_ECC_", "USE_EXT_CE_"})
                    write_bit(b + p + ".TRUE", str((b + p).c_str(), "FALSE") == "TRUE");
                write_bit("RST_MODE_" + p + ".ASYNC", str(("RST_MODE_" + p).c_str(), "SYNC") == "ASYNC");
                int64_t addr = num("SELF_ADDR_" + p, 0), mask = num("SELF_MASK_" + p, 0x7ff);
                for (int i = 0; i < 11; i++) {
                    write_bit(stringf("SELF_ADDR_%s[%d]", p.c_str(), i), (addr >> i) & 1);
                    write_bit(stringf("ZSELF_MASK_%s[%d]", p.c_str(), i), !((mask >> i) & 1));
                }
            }
            write_bit("EN_AUTO_SLEEP_MODE.TRUE", str("EN_AUTO_SLEEP_MODE", "FALSE") == "TRUE");
            int64_t lat = num("AUTO_SLEEP_LATENCY", 8);
            for (int i = 0; i < 4; i++)
                write_bit(stringf("AUTO_SLEEP_LATENCY[%d]", i), (lat >> i) & 1);
            for (std::string pin : {"CLK", "EN_A", "EN_B", "RDB_WR_A", "RDB_WR_B", "RST_A", "RST_B"})
                write_bit("IS_" + pin + "_INVERTED", num("IS_" + pin + "_INVERTED", 0) != 0);
            pop(2);
            blank();
        }
        for (auto &t : tile_rows) {
            push(uarch->tile_name(t.first));
            for (auto &r : t.second)
                write_bit(stringf("OUT_%c_ROW%d", r.first, r.second));
            pop();
            blank();
        }
    }

    void write_cmt_usp()
    {
        auto refuse_if = [&](CellInfo *ci, bool cond, const char *what) {
            if (cond)
                log_error("%s '%s': %s is not supported on UltraScale+\n", ci->type.c_str(ctx), ctx->nameOf(ci),
                          what);
        };
        auto param_bool = [&](CellInfo *ci, const std::string &name) {
            return bool_or_default(ci->params, ctx->id(name), false);
        };
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            if (ci->type.in(id_MMCM_MMCM_TOP, id_PLL_PLL_TOP) && ci->bel != BelId())
                write_pll_usp(ci);
            if (!ci->type.in(id_BUFCE_BUFCE, id_BUFGCE_DIV_BUFGCE_DIV, id_BUFGCTRL) || ci->bel == BelId())
                continue;
            std::string tile = uarch->tile_name(ci->bel.tile);
            if (!boost::starts_with(tile, "CMT_L"))
                log_error("'%s' is on %s; only the CMT_L clock buffer sites are characterised\n", ctx->nameOf(ci),
                          ctx->nameOfBel(ci->bel));
            push(tile);
            push(usp_site_name(uarch->get_bel_site(ci->bel)));
            if (ci->type == id_BUFCE_BUFCE) {
                std::string ce_type = str_or_default(ci->params, ctx->id("CE_TYPE"), "SYNC");
                refuse_if(ci, ce_type != "SYNC" && ce_type != "ASYNC", "CE_TYPE other than SYNC/ASYNC");
                refuse_if(ci, param_bool(ci, "IS_I_INVERTED"), "IS_I_INVERTED");
                if (ce_type == "ASYNC")
                    write_choice("CE_TYPE", "ASYNC");
                write_bit("IS_CE_INVERTED", param_bool(ci, "IS_CE_INVERTED"));
            } else if (ci->type == id_BUFGCE_DIV_BUFGCE_DIV) {
                int divide = int_or_default(ci->params, ctx->id("BUFGCE_DIVIDE"), 1);
                refuse_if(ci, divide < 1 || divide > 8, "BUFGCE_DIVIDE outside 1-8");
                refuse_if(ci, str_or_default(ci->params, ctx->id("CE_TYPE"), "SYNC") != "SYNC",
                          "CE_TYPE other than SYNC");
                refuse_if(ci, param_bool(ci, "IS_I_INVERTED"), "IS_I_INVERTED");
                if (divide > 1)
                    write_choice("BUFGCE_DIVIDE", "D" + std::to_string(divide));
                write_bit("IS_CE_INVERTED", param_bool(ci, "IS_CE_INVERTED"));
                write_bit("IS_CLR_INVERTED", param_bool(ci, "IS_CLR_INVERTED"));
            } else {
                for (auto ce : {"CE_TYPE_CE0", "CE_TYPE_CE1"})
                    refuse_if(ci, str_or_default(ci->params, ctx->id(ce), "SYNC") != "SYNC",
                              "CE_TYPE_CE0/CE1 other than SYNC");
                refuse_if(ci, param_bool(ci, "IS_I0_INVERTED") || param_bool(ci, "IS_I1_INVERTED"),
                          "IS_I0_INVERTED/IS_I1_INVERTED");
                bool pre0 = str_or_default(ci->params, ctx->id("PRESELECT_I0"), "FALSE") == "TRUE";
                bool pre1 = str_or_default(ci->params, ctx->id("PRESELECT_I1"), "FALSE") == "TRUE";
                refuse_if(ci, pre0 && pre1, "PRESELECT_I0 and PRESELECT_I1 both TRUE");
                write_bit("INIT_OUT", int_or_default(ci->params, ctx->id("INIT_OUT"), 0) != 0);
                write_bit("PRESELECT_I1", pre1);
                for (auto pin : {"CE0", "CE1", "S0", "S1", "IGNORE0", "IGNORE1"})
                    write_bit(std::string("IS_") + pin + "_INVERTED", param_bool(ci, std::string("IS_") + pin + "_INVERTED"));
            }
            pop(2);
        }
        blank();
    }

    void write_pss_intf_usp()
    {
        for (int tile : usp_pss_outputs_enabled) {
            push(uarch->tile_name(tile));
            write_bit("OUTPUTS_ENABLED");
            pop();
        }
        blank();
    }

    // UltraScale+ block RAM. Feature names are prjuray's segbits_bram*.db
    // plus 0-ultrascale-bits/db-corrections/zynqusp/segbits_bram.corrections
    // (IN_USE, port widths, RSTREGBINV), which prjuray never solved.
    // Characterised against Vivado in 0-ultrascale-bits/designs/kv260_bram_char.

    // INIT_A/INIT_B/SRVAL_A/SRVAL_B of a RAMB18 as stored: {P1,P0,D15..D0},
    // except that a 9-bit port's value arrives as {P1,D15..8,P0,D7..0}.
    static uint64_t bram18_vec_usp(uint64_t v, int width)
    {
        if (width != 9)
            return v;
        return (((v >> 17) & 1) << 17) | (((v >> 8) & 1) << 16) | (((v >> 9) & 0xFF) << 8) | (v & 0xFF);
    }

    // The same for a RAMB36, split over the two halves: normalise to
    // {P3..P0,D31..D0}, then data bit d goes to half d%2 bit d/2 and parity
    // bit k to half k%2 bit 16+k/2.
    static std::pair<uint64_t, uint64_t> bram36_vec_usp(uint64_t v, int width)
    {
        uint64_t d = 0, p = 0;
        if (width == 9) {
            for (int u = 0; u < 4; u++) {
                d |= ((v >> (9 * u)) & 0xFF) << (8 * u);
                p |= ((v >> (9 * u + 8)) & 1) << u;
            }
        } else if (width == 18) {
            d = (v & 0xFFFF) | (((v >> 18) & 0xFFFF) << 16);
            p = ((v >> 16) & 3) | (((v >> 34) & 3) << 2);
        } else {
            d = v & 0xFFFFFFFFULL;
            p = (v >> 32) & 0xF;
        }
        uint64_t l = 0, u = 0;
        for (int i = 0; i < 32; i++)
            if ((d >> i) & 1)
                ((i % 2) ? u : l) |= 1ULL << (i / 2);
        for (int k = 0; k < 4; k++)
            if ((p >> k) & 1)
                ((k % 2) ? u : l) |= 1ULL << (16 + k / 2);
        return {l, u};
    }

    void write_bram_width_usp(CellInfo *ci, const std::string &name, bool is_36, bool sdp_ram)
    {
        int width = int_or_default(ci->params, ctx->id(name), 0);
        int full = is_36 ? 36 : 18;
        // Simple dual port, as measured on the KV260 (0-ultrascale-bits
        // designs/kv260_sdp_bram_test, a 512x32 RAMB18E2 filled and read back):
        //   read:  READ_WIDTH_A.SDP alone. With W18 on READ_WIDTH_A/B as well,
        //          the RAM reads back garbage.
        //   write: WRITE_WIDTH_A.W<full> and WRITE_WIDTH_B.W<full>, plus
        //          WRITE_WIDTH_B.SDP. With no write width bits, each 18-bit half
        //          stores only its bit 0 (the all-clear code is width 1).
        // The decoded Vivado reference (kv260_bram_char v18L) shows only the two
        // SDP bits, which looked like the whole story and is not: trust the board.
        // prjuray calls the SDP bit <port>.36 (.72 on a RAMB36), which a FASM
        // parser cannot read, so it is written <port>.SDP
        // (db-corrections/zynqusp/segbits_bram.corrections). The RAMB36E2 (SDP72)
        // follows the same pattern with W36; that case is not yet board-tested.
        if (sdp_ram) {
            if (name == "READ_WIDTH_A")
                write_bit(name + ".SDP");
            else if (name == "WRITE_WIDTH_A")
                write_bit(name + ".W" + std::to_string(full));
            else if (name == "WRITE_WIDTH_B") {
                write_bit(name + ".W" + std::to_string(full));
                write_bit(name + ".SDP");
            }
            return;
        }
        if (width == 0)
            width = full;
        static const std::set<int> w18 = {1, 2, 4, 9, 18}, w36 = {1, 2, 4, 9, 18, 36};
        if (!(is_36 ? w36 : w18).count(width))
            log_error("%s '%s': unsupported %s %d\n", ci->type.c_str(ctx), ctx->nameOf(ci), name.c_str(), width);
        // The all-clear code has no feature: width 1 on a RAMB18, 2 on a RAMB36.
        if (width != (is_36 ? 2 : 1))
            write_bit(name + ".W" + std::to_string(width));
    }

    void write_bram_cell_usp(int tile, CellInfo *ci, int half)
    {
        bool is_36 = ci->type == id_RAMB36E2_RAMB36E2;
        IdString orig = ctx->id(ci->attrs.at(id_X_ORIG_TYPE).as_string());
        // Settings with a RAMB36E2 feature covering both halves.
        std::string site = is_36 ? "RAMB36E2" : (half == 0 ? "RAMB18E2_L" : "RAMB18E2_U");
        auto str_param = [&](const char *name, const char *def) {
            return str_or_default(ci->params, ctx->id(name), def);
        };
        int rw_a = int_or_default(ci->params, ctx->id("READ_WIDTH_A"), 0);
        int ww_b = int_or_default(ci->params, id_WRITE_WIDTH_B, 0);
        bool sdp = is_36 ? (rw_a == 72 || ww_b == 72) : (rw_a == 36 || ww_b == 36);

        push(uarch->tile_name(tile));
        push(site);
        write_bit("IN_USE");
        for (auto w : {"READ_WIDTH_A", "WRITE_WIDTH_A", "READ_WIDTH_B", "WRITE_WIDTH_B"})
            write_bram_width_usp(ci, w, is_36, sdp);
        // The block RAM's clock arrives inverted from the slices' when both hang
        // off the same leaf polarity: Vivado puts BRAM columns on leaves of the
        // opposite IINV (kv260_bram_char's reference), which write_pip_as_site_config_usp
        // does not. Inverting at the RAM's own clock pins instead works whichever
        // leaf the router picks. Measured on a KV260 (0-ultrascale-bits
        // designs/kv260_bram_hold_test): a read address that changes on the edge
        // that reads it -- a FIFO pointer, a program counter -- read address + 1 on
        // every clock in two clock regions of opposite leaf IINV; with the pins
        // inverted, 0 mismatches in 84M clocks. Writes, and reads whose address
        // was stable for a cycle, worked either way, which is how it hid.
        write_bool_usp("CLKARDCLKINV", !bool_or_default(ci->params, ctx->id("IS_CLKARDCLK_INVERTED"), false));
        write_bool_usp("CLKBWRCLKINV", !bool_or_default(ci->params, ctx->id("IS_CLKBWRCLK_INVERTED"), false));
        for (auto &invpin : invertible_pins[orig]) {
            bool inv = bool_or_default(ci->params, ctx->idf("IS_%s_INVERTED", invpin.c_str(ctx)), false);
            if (invpin == id_RSTREGB)
                write_bit("RSTREGBINV.V1", inv);
            else
                write_bool_usp(invpin.str(ctx) + "INV", inv);
        }
        // RSTREGB is in invertible_pins, so this was written above unless
        // the RAM is SDP, where Vivado sets it too (port B has no output
        // register to reset). There is no V0 feature.
        if (sdp && !bool_or_default(ci->params, ctx->id("IS_RSTREGB_INVERTED"), false))
            write_choice("RSTREGBINV", "V1");
        // Vivado writes COMMON only when both clocks are on one net. On a
        // RAMB36 the packer splits each clock into L and U bel pins.
        auto clk_net = [&](const char *name) {
            NetInfo *n = ci->getPort(ctx->id(name));
            return n != nullptr ? n : ci->getPort(ctx->id(std::string(name) + "L"));
        };
        NetInfo *clka = clk_net("CLKARDCLK"), *clkb = clk_net("CLKBWRCLK");
        write_choice("CLOCK_DOMAINS", (clka != nullptr && clka == clkb) ? "COMMON" : "INDEPENDENT");
        write_bool_usp("DOA_REG", bool_or_default(ci->params, id_DOA_REG, false));
        if (!is_36)
            write_bool_usp("DOB_REG", bool_or_default(ci->params, id_DOB_REG, false));
        write_choice("ENADDRENA", str_param("ENADDRENA", "FALSE"));
        write_choice("ENADDRENB", str_param("ENADDRENB", "FALSE"));
        write_choice("RDADDRCHANGEA", str_param("RDADDRCHANGEA", "FALSE"));
        write_choice("RDADDRCHANGEB", str_param("RDADDRCHANGEB", "FALSE"));
        write_choice("RSTREG_PRIORITY_A", str_param("RSTREG_PRIORITY_A", "RSTREG"));
        write_choice("RSTREG_PRIORITY_B", str_param("RSTREG_PRIORITY_B", "RSTREG"));
        write_choice("SLEEP_ASYNC", str_param("SLEEP_ASYNC", "FALSE"));
        write_choice("WRITE_MODE_A", str_param("WRITE_MODE_A", "NO_CHANGE"));
        write_choice("WRITE_MODE_B", str_param("WRITE_MODE_B", "NO_CHANGE"));
        if (is_36)
            write_choice("EN_ECC_WRITE", str_param("EN_ECC_WRITE", "FALSE"));
        pop();

        // Per-half: output register B, INIT_A/B, SRVAL_A/B and the contents.
        for (int h = is_36 ? 0 : half; h <= (is_36 ? 1 : half); h++) {
            push(h == 0 ? "RAMB18E2_L" : "RAMB18E2_U");
            if (is_36)
                write_bool_usp("DOB_REG", bool_or_default(ci->params, id_DOB_REG, false));
            for (auto vec : {"INIT_A", "INIT_B", "SRVAL_A", "SRVAL_B"}) {
                int width = int_or_default(ci->params, ctx->id(std::string("READ_WIDTH_") + vec[std::strlen(vec) - 1]), 0);
                uint64_t v = 0;
                if (ci->params.count(ctx->id(vec)))
                    v = ci->params.at(ctx->id(vec)).as_int64();
                uint64_t enc;
                if (is_36) {
                    // The packing at widths 1, 2 and 4 is not characterised.
                    if (v != 0 && (width == 1 || width == 2 || width == 4))
                        log_error("%s '%s': nonzero %s at width %d is not supported on UltraScale+\n",
                                  ci->type.c_str(ctx), ctx->nameOf(ci), vec, width);
                    auto lu = bram36_vec_usp(v, width);
                    enc = h == 0 ? lu.first : lu.second;
                } else {
                    enc = bram18_vec_usp(v, width);
                }
                write_int_vector(std::string(vec) + "[17:0]", enc, 18);
            }
            write_bram_init(h, ci, is_36);
            pop();
        }
        pop();
    }

    // Runs after write_routing(), which records the output rows.
    void write_bram_usp()
    {
        for (int tile = 0; tile < ctx->chip_info->tile_insts.ssize(); tile++) {
            const auto &bts = uarch->tile_status[tile].bts;
            if (!bts)
                continue;
            // Row k of a BRAM tile is INT_INTF_L_X<x>Y<y+k>, checked for every
            // BRAM tile on the xck26.
            int x, y;
            if (usp_tile_xy(uarch->tile_name(tile), x, y)) {
                for (int k = 0; k < 5; k++) {
                    if (usp_bram_output_rows.count({x, y + k})) {
                        push(uarch->tile_name(tile));
                        write_bit(stringf("ROW%d.OUTPUTS_ENABLED", k));
                        pop();
                    }
                }
            }
            if (bts->cells[BEL_RAM36] != nullptr) {
                write_bram_cell_usp(tile, bts->cells[BEL_RAM36], 0);
            } else {
                if (bts->cells[BEL_RAM18_L] != nullptr)
                    write_bram_cell_usp(tile, bts->cells[BEL_RAM18_L], 0);
                if (bts->cells[BEL_RAM18_U] != nullptr)
                    write_bram_cell_usp(tile, bts->cells[BEL_RAM18_U], 1);
            }
            blank();
        }
    }

    void write_fasm()
    {
        get_invertible_pins(ctx, invertible_pins, uarch->usp);
        if (uarch->usp) {
            // The BRAM FIFOs and cascades of DSP, BRAM and URAM are not
            // ported yet (the packer rejects them).
            write_logic();
            write_io_usp();
            write_routing();
            write_bram_usp();
            write_clocking_usp();
            write_cmt_usp();
            write_dsp_usp();
            write_uram_usp();
            write_pss_intf_usp();
            write_config_usp();
            return;
        }
        write_logic();
        write_io();
        write_routing();
        write_bram();
        write_clocking();
        write_ip();
    }
};

} // namespace

void XilinxImpl::write_fasm(const std::string &filename)
{
    auto out = open_ofstream_and_log_error(filename, "FASM file");

    FasmBackend be(this->ctx, this, out);
    be.write_fasm();
}

NEXTPNR_NAMESPACE_END
