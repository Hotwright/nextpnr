"""
UltraScale+ slice logic timing from RapidWright's delay model
(timing/ultrascaleplus/intrasite_delay_terms.txt, copied into the dump by
run_usp_dump.sh). It is the model RWRoute's timing-driven mode uses: one corner,
in ps, calibrated by its authors against Vivado. The routing half of the same
model is in nodes_delay.bin.

Only the "bel" sections are read here; the "site" sections are intra-site wire
delays, which the chipdb does not model separately (every site pip is the
SITE_NULL class).
"""

import re

# RapidWright's model is for faster UltraScale+ parts than the K26 -2LV
# (0.72 V). Vivado 2024.2's timing of kv260_ps_axi_test on xck26-sfvc784-2LV-c
# puts the slice cells at about 1.5x the model: FF clock-to-Q 112-118 ps
# (model 77), the slowest LUT input 220-228 ps (A1, 150), the fastest
# 56-60 ps (A6, 35). Every cell arc below is scaled by CELL_SCALE, and
# xilinx_gen.py scales the node delays by ROUTE_SCALE.
#
# ROUTE_SCALE: the 880 single-load nets of the same Vivado build, each priced
# node by node with the model, fit Vivado = 1.34 x model (+107 ps when the
# sink is an FF D pin: the wire inside the slice, see below), RMS error 76 ps.
CELL_SCALE = 1.5
ROUTE_SCALE = 1.35
# Site pin -> FF D inside the slice: model 100 ps, fitted from Vivado 107.
FF_D_SITE_WIRE = 107


def parse_logic_arcs(filename, config=None):
    """
    {bel name: {(from pin, to pin): delay ps}} for every "bel" section.

    An arc listing configurations applies only when each listed configuration
    that `config` names includes config's value; configurations config does not
    name match any value, and the largest delay of all matching arcs is kept.
    """
    config = config or {}
    arcs = {}
    bels = None
    with open(filename) as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            words = line.split()
            if words[0] == "site":
                bels = None
                continue
            if words[0] == "bel":
                # bel A,B,C [CONFIG: v v ...]* in SITE,SITE
                bels = words[1].split(",")
                for b in bels:
                    arcs.setdefault(b, {})
                continue
            if bels is None:
                continue
            frm, to, delay = words[0], words[1], int(float(words[2]))
            rest = words[3:]
            # A substitution ("i: 0-31", "A: A B C") comes first, in lower case
            # or as a single letter; configurations are upper-case words.
            subst = [("", "")]
            if rest and re.fullmatch(r"([a-z]+|[A-Z]):", rest[0]):
                key, rest, vals = rest[0][:-1], rest[1:], []
                while rest and not rest[0].endswith(":"):
                    v, rest = rest[0], rest[1:]
                    m = re.fullmatch(r"(\d+)-(\d+)", v)
                    vals += [str(i) for i in range(int(m[1]), int(m[2]) + 1)] if m else [v]
                subst = [(key, v) for v in vals]
            # For each configuration config names, its value must be listed.
            listed = {}
            key = None
            for w in rest:
                if w.endswith(":"):
                    key = w[:-1]
                    listed[key] = set()
                else:
                    listed[key].add(w)
            if any(k in config and config[k] not in v for k, v in listed.items()):
                continue
            for key, v in subst:
                f_sub = frm.replace(key, v) if key else frm
                t_sub = to.replace(key, v) if key else to
                for b in bels:
                    for fp in f_sub.split(","):
                        for tp in t_sub.split(","):
                            k = (fp, tp)
                            arcs[b][k] = max(arcs[b].get(k, delay), delay)
    return arcs


def import_usp_cell_timings(ch, filename, TimingValue, ClockEdge):
    arcs = parse_logic_arcs(filename, {"CARRY_TYPE": "SINGLE_CY8"})
    tv = lambda d: TimingValue(round(d * CELL_SCALE), round(d * CELL_SCALE))

    lut = ch.timing.add_cell_variant("DEFAULT", "SLICE_LUTX")
    lut6, lut5 = arcs["A6LUT"], arcs["A5LUT"]
    for i in range(1, 6+1):
        lut.add_comb_arc(f"A{i}", "O6", tv(lut6[(f"A{i}", "O6")]))
        if i <= 5:
            lut.add_comb_arc(f"A{i}", "O5", tv(lut5[(f"A{i}", "O5")]))
    # LUT RAM: write port setup, and the write-through clock-to-output.
    for i in range(1, 6+1):
        lut.add_setup_hold("CLK", f"WA{i}", ClockEdge.RISING, tv(lut6[("CLK", f"WA{i}")]), tv(0))
    for i in (7, 8): # WA7/WA8 address the F7/F8 muxes of wide RAMs; the model has none
        lut.add_setup_hold("CLK", f"WA{i}", ClockEdge.RISING, tv(lut6[("CLK", "WA6")]), tv(0))
    lut.add_setup_hold("CLK", "WE", ClockEdge.RISING, tv(lut6[("CLK", "WE")]), tv(0))
    lut.add_setup_hold("CLK", "DI1", ClockEdge.RISING, tv(lut6[("CLK", "DI1")]), tv(0))
    lut.add_setup_hold("CLK", "DI2", ClockEdge.RISING, tv(lut6[("CLK", "DI2")]), tv(0))
    lut.add_clock_out("CLK", "O6", ClockEdge.RISING, tv(lut6[("CLK", "O6")]))
    lut.add_clock_out("CLK", "O5", ClockEdge.RISING, tv(lut5[("CLK", "O5")]))

    ff = ch.timing.add_cell_variant("DEFAULT", "SLICE_FFX")
    ffa = arcs["AFF"]
    # The model's setup is relative to the FF's D bel pin (-25 ps); the site
    # pin to D wire inside the slice (FF_D_SITE_WIRE) is not carried by the
    # chipdb's site pips. Fold it in here: pessimistic by ~50 ps for a LUT
    # feeding its own FF, exact from outside the slice. (Vivado's own D setup
    # is -44 ps, which with the wire gives 63.)
    setup = round(ffa[("CLK", "D")] * CELL_SCALE) + FF_D_SITE_WIRE
    ff.add_setup_hold("CLK", "D", ClockEdge.RISING, TimingValue(setup, setup), tv(0))
    # CE and SR: the model gives only their 5 ps site pin delay. Vivado's CE
    # setup on the K26 is 78-81 ps; use it, unscaled, for SR too.
    ff.add_setup_hold("CLK", "CE", ClockEdge.RISING, TimingValue(80, 80), tv(0))
    ff.add_setup_hold("CLK", "SR", ClockEdge.RISING, TimingValue(80, 80), tv(0))
    ff.add_clock_out("CLK", "Q", ClockEdge.RISING, tv(ffa[("CLK", "Q")]))

    # F8MUX and F9MUX are not in the model; F7MUX's arcs stand in.
    f7 = arcs["F7MUX_AB"]
    for mux in ("F7MUX", "F8MUX", "F9MUX"):
        m = ch.timing.add_cell_variant("DEFAULT", mux)
        for i in ("0", "1", "S0"):
            m.add_comb_arc(i, "OUT", tv(f7[(i, "OUT")]))

    c8 = ch.timing.add_cell_variant("DEFAULT", "CARRY8")
    for (frm, to), d in sorted(arcs["CARRY8"].items()):
        c8.add_comb_arc(frm, to, tv(d))
