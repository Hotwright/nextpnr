# Mapping rules from prjxray to nextpnr

# Set by xilinx_gen.py for an UltraScale+ device. The z encodings below must
# match legacy nextpnr-xilinx's bbaexport.getBelZoverride() bit for bit: the
# placer validity check and FASM writers ported from it index bels by z.
USP = False

def get_bel_z_override_usp(bel, default_z):
    s = bel.site
    t = s.tile
    bt = bel.bel_type()
    bn = bel.name()
    if t.tile_type() == "BRAM":
        return {"RAMBFIFO36E2_RAMBFIFO36E2": 0, "RAMB36E2_RAMB36E2": 1, "FIFO36E2_FIFO36E2": 2,
                "RAMB18E2_U_RAMB18E2": 5, "RAMBFIFO18E2_RAMBFIFO18E2": 8, "RAMB18E2_L_RAMB18E2": 9,
                "FIFO18E2_FIFO18E2": 10}.get(bt, default_z)
    if t.tile_type() == "DSP":
        is_upper_site = any(o.grid_xy[1] < s.grid_xy[1] for o in t.sites())
        zs = (1 << 5) if is_upper_site else 0
        order = ["DSP_PREADD_DATA", "DSP_PREADD", "DSP_A_B_DATA", "DSP_MULTIPLIER",
                 "DSP_C_DATA", "DSP_M_DATA", "DSP_ALU", "DSP_OUTPUT"]
        return (zs | order.index(bt)) if bt in order else -1
    if s.site_type() in ("SLICEL", "SLICEM"):
        # z[6..4] = eighth (A-H), z[3..0] = type, as LogicBelTypeZ
        subslices = "ABCDEFGH"
        for i, pf in enumerate(["6LUT", "5LUT", "FF", "FF2"]):
            if len(bn) == len(pf) + 1 and bn[1:] == pf:
                return (subslices.index(bn[0]) << 4) | i
        if bn.startswith("FFMUX"):
            return (subslices.index(bn[5]) << 4) | (5 if bn[6] == "2" else 4)
        if bn.startswith("OUTMUX"):
            return (subslices.index(bn[6]) << 4) | 6
        return {"F7MUX_AB": 0x07, "F7MUX_CD": 0x27, "F7MUX_EF": 0x47, "F7MUX_GH": 0x67,
                "F8MUX_BOT": 0x08, "F8MUX_TOP": 0x48, "F9MUX": 0x09, "CARRY8": 0x0A,
                "CLK1INV": 0x0B, "CLK2INV": 0x4B, "LCLKINV": 0x1B,
                "RST_ABCDINV": 0x0C, "RST_EFGHINV": 0x4C, "HARD0": 0x0D}.get(bn, -1)
    return default_z

def get_bel_z_override(bel, default_z):
    if USP:
        return get_bel_z_override_usp(bel, default_z)
    s = bel.site
    t = s.tile
    bt = bel.bel_type()
    bn = bel.name()
    if t.tile_type() == "BRAM_L" or t.tile_type() == "BRAM_R":
        is_top18 = (s.primary.site_type() == "RAMB18E1")
        if bt == "RAMBFIFO36E1_RAMBFIFO36E1":
            return 0
        elif bt == "RAMB36E1_RAMB36E1":
            return 1
        elif bt == "FIFO36E1_FIFO36E1":
            return 2
        elif bt == "RAMB18E1_RAMB18E1":
            return (5 if is_top18 else 9)
        elif bt == "FIFO18E1_FIFO18E1":
            return 10
    if s.site_type() == "SLICEL" or s.site_type() == "SLICEM":
        is_upper_site = (s.rel_xy()[0] == 1)
        subslices = "ABCD"
        postfixes = ["6LUT", "5LUT", "FF", "5FF"]
        for i, pf in enumerate(postfixes):
            if len(bn) == len(pf) + 1 and bn[1:] == pf:
                return (64 if is_upper_site else 0) | (subslices.index(bn[0]) << 4) | i
        if bn == "F7AMUX":
            return (0x47 if is_upper_site else 0x07)
        elif bn == "F7BMUX":
            return (0x67 if is_upper_site else 0x27)
        elif bn == "F8MUX":
            return (0x48 if is_upper_site else 0x08)
        elif bn == "CARRY4":
            return (0x4F if is_upper_site else 0x0F)
        # Other bels (e.g. extra xc7 routing bels) can be ignored for nextpnr porpoises
        return -1
    return default_z

def get_bel_type_override(bt):
    if bt.endswith("6LUT") or bt == "LUT_OR_MEM6" or bt == "LUT6":
        return "SLICE_LUTX"
    elif bt.endswith("5LUT") or bt == "LUT_OR_MEM5" or bt == "LUT5":
        return "SLICE_LUTX"
    elif len(bt) == 4 and bt.endswith("FF2"):
        return "SLICE_FFX"
    elif len(bt) == 3 and bt.endswith("FF"):
        return "SLICE_FFX"
    elif bt == "FF_INIT" or bt == "REG_INIT":
        return "SLICE_FFX"

    if USP:
        for p in ("_PAD_", "_PULL_", "_IBUFCTRL_", "_INBUF_", "_OUTBUF_"):
            if p in bt:
                return "IOB_" + p.replace("_", "")
    iolParts = ["COMBUF_", "IDDR_", "IPFF_", "OPFF_", "OPTFF_", "TFF_"]
    for p in iolParts:
        if bt.startswith(p):
            return "IOL_" + p.replace("_", "")
    if bt.endswith("_VREF"):
        return "IOB_VREF"
    elif bt.endswith("_DIFFINBUF"):
        return "IOB_DIFFINBUF"
    elif bt.startswith("PSS_ALTO_CORE_PAD_"):
        return "PSS_PAD"
    elif bt.startswith("LAGUNA_RX_REG") or bt.startswith("LAGUNA_TX_REG"):
        return "LAGUNA_REGX"
    elif bt.startswith("BSCAN"):
        return "BSCAN"
    elif bt == "BUFGCTRL_BUFGCTRL":
        return "BUFGCTRL"
    elif bt == "RAMB18E2_U_RAMB18E2" or bt == "RAMB18E2_L_RAMB18E2":
        return "RAMB18E2_RAMB18E2"
    else:
        return bt

def include_pip(tile_type, p):
    if USP:
        # CLE tile-level route-thrus (A1 -> AMUX etc.) carry no LUT
        # occupancy check and could run straight through a used LUT; the
        # generator's own LUT route-thru pseudo-pips are the checked path.
        if p.is_route_thru() and tile_type in ("CLEL_L", "CLEL_R", "CLEM", "CLEM_R"):
            return False
        # The RCLK_INT GND_WIRE<n> sources: prjuray-db has no segbits for
        # INT_RCLK_TO_CLK_*.GND_WIRE<n>, so a constant routed this way would
        # assemble to nothing. Constants come from the INT tiles instead.
        if tile_type.startswith("RCLK_INT_") and p.src_wire().name().startswith(("GND_WIRE", "VCC_WIRE")):
            return False
        if tile_type.startswith("HDIO_"):
            # The differential-pair crossover: OP -> OUTINV (inverting) ->
            # OUTB_B -> the partner's OUTB_B_IN -> its OUTMUX. Only a true
            # differential output may use it; a single-ended one routed that
            # way drives the wrong pad, inverted. Same blacklist as
            # patches/nextpnr-xilinx-usp-router2-pip-avail.patch.
            if p.dst_wire().name().endswith("_OUTB_B_IN_PIN"):
                return False
            if p.is_route_thru() and p.dst_wire().name().endswith("_OUTB_B"):
                return False
    is_xc7_logic = (tile_type in ("CLBLL_L", "CLBLL_R", "CLBLM_L", "CLBLM_R"))
    if p.is_route_thru() and p.src_wire().name().endswith("_CE_INT"):
        return False
    if p.is_route_thru() and is_xc7_logic:
        return False
    if p.is_route_thru() and "TFB" in p.dst_wire().name():
        return False
    if p.src_wire().name().startswith("CLK_BUFG_R_FBG_OUT"):
        return False
    if "CLK_HROW_CK_INT" in p.src_wire().name():
        return False
    # if tile_type.startswith("HCLK_CMT") and "FREQ_REF" in p.dst_wire().name():
    #     return False
    if tile_type.startswith("CLK_HROW_TOP"):
        if "CK_BUFG_CASCO" in p.dst_wire().name() and "CK_BUFG_CASCIN" in p.src_wire().name():
            return False
    if tile_type.startswith("HCLK_IOI"):
        if "RCLK_BEFORE_DIV" in p.dst_wire().name() and "IMUX" in p.src_wire().name():
            return False
        if  p.dst_wire().name().endswith("_DMUX") and "I2IOCLK_TOP" in p.src_wire().name():
            return False
    if "IOI" in tile_type:
        if "CLKB" in p.dst_wire().name() and "IMUX22" in p.src_wire().name():
            return False
        if "OCLKB" in p.dst_wire().name() and "IOI_OCLK_" in p.src_wire().name():
            return False
        if "OCLKM" in p.dst_wire().name() and "IMUX31" in p.src_wire().name():
            return False
    if "CMT_TOP_R" in tile_type:
        if "PLLOUT_CLK_FREQ_BB_REBUFOUT" in p.dst_wire().name():
            return False
        if "MMCM_CLK_FREQ_BB" in p.dst_wire().name():
            return False
    return True

def is_global_bel(bel):
    bt = bel.bel_type()
    return bt in ("BUFGCTRL_BUFGCTRL", "BUFG_BUFG")
