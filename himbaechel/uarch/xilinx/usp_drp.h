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

// UltraScale+ MMCME4/PLLE4 DRP register values from the cell parameters, as
// the bitstream holds them. A port of XAPP888's calculation (AMD's
// mmcm_pll_drp_func_us_plus_{mmcm,pll}.vh) with the differences Vivado shows
// on the K26 (0-ultrascale-bits/designs/kv260_mmcm_char, drp.py; every
// register of 480 random cells matched):
//   - a fractional multiplier indexes the lock/filter tables rounded half up
//     (XAPP888 truncates);
//   - CLKOUT5/CLKOUT6 REG2[15:12] hold the falling-edge fraction of
//     CLKOUT0/CLKFBOUT (FRAC_WF_F[12], PHASE_MUX_F[15:13]); for an integer
//     counter Vivado copies its PHASE_MUX there;
//   - REG1[12] is the output enable: a connected output, CLKFBOUT, and
//     CLKOUT6 when CLKOUT4_CASCADE (which also sets CLKOUT4 REG2[9]);
//   - FILT1 bit 3 and MMCM LOCK3 bit 15 are reserved ones.
// Header-only and free of nextpnr types so that it can be checked against
// drp.py outside nextpnr.

#ifndef XILINX_USP_DRP_H
#define XILINX_USP_DRP_H

#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "usp_drp_tables.h"

namespace UspDrp {

// The headers' fixed-point values carry 10 fraction bits in 32.
static constexpr int FRAC_PRECISION = 10;
static constexpr int64_t MASK32 = 0xFFFFFFFFLL;

inline int64_t floordiv(int64_t a, int64_t b)
{
    int64_t q = a / b;
    if ((a % b != 0) && ((a < 0) != (b < 0)))
        --q;
    return q;
}

inline int64_t round_frac(int64_t decimal, int precision)
{
    if ((decimal >> (FRAC_PRECISION - precision - 1)) & 1)
        decimal += int64_t(1) << (FRAC_PRECISION - precision);
    return decimal & MASK32;
}

struct Divider
{
    int edge, no_count, high, low;
};

// duty is the duty cycle times 100,000
inline Divider divider(int64_t divide, int64_t duty)
{
    int64_t duty_fix = floordiv(duty << FRAC_PRECISION, 100000) & MASK32;
    if (divide == 1)
        return {0, 1, 1, 1};
    int64_t temp = round_frac((duty_fix * divide) & MASK32, 1);
    int64_t high = (temp >> FRAC_PRECISION) & 0x7F;
    int edge = (temp >> (FRAC_PRECISION - 1)) & 1;
    if (high == 0) {
        high = 1;
        edge = 0;
    }
    if (high == divide) {
        high = divide - 1;
        edge = 1;
    }
    int64_t low = divide - high;
    return {edge, 0, int(high & 0x3F), int(low & 0x3F)};
}

struct Phase
{
    int mx, phase_mux, delay_time;
};

// ph is the phase in millidegrees
inline Phase phase(int64_t divide, int64_t ph)
{
    int64_t fixed;
    if (ph < 0)
        fixed = floordiv((ph + 360000) << FRAC_PRECISION, 1000) & MASK32;
    else
        fixed = floordiv(ph << FRAC_PRECISION, 1000) & MASK32;
    int64_t cycles = floordiv(fixed * divide, 360) & MASK32;
    int64_t temp = round_frac(cycles, 3);
    return {0, int((temp >> (FRAC_PRECISION - 3)) & 7), int((temp >> FRAC_PRECISION) & 0x3F)};
}

struct Counter
{
    uint16_t reg1, reg2;
    int transfer; // the falling-edge fraction field for CLKOUT5/6 REG2[15:12]
};

inline Counter count_calc(int64_t divide, int64_t ph, int64_t duty)
{
    Divider d = divider(divide, duty);
    Phase p = phase(divide, ph);
    Counter c;
    c.reg1 = uint16_t(d.low | d.high << 6 | p.phase_mux << 13);
    c.reg2 = uint16_t(p.delay_time | d.no_count << 6 | d.edge << 7 | p.mx << 8);
    c.transfer = p.phase_mux << 1;
    return c;
}

// divide: the integer part; frac: the fraction times 1000 (a multiple of 125)
inline Counter frac_count_calc(int64_t divide, int64_t ph, int64_t frac)
{
    int64_t f = frac / 125;
    int64_t even_high = divide >> 1, even_low = even_high;
    int64_t odd = divide - even_high - even_low;
    int64_t odd_and_frac = 8 * odd + f;
    int64_t lt = (even_high - (odd_and_frac <= 9 ? 1 : 0)) & 0xFF;
    int64_t ht = (even_low - (odd_and_frac <= 8 ? 1 : 0)) & 0xFF;
    int64_t pm_fall = ((odd & 0x7F) << 2) + (f >> 1);
    int wf_fall = ((2 <= odd_and_frac && odd_and_frac <= 9) || (divide == 2 && f == 1)) ? 1 : 0;
    int wf_rise = (1 <= odd_and_frac && odd_and_frac <= 8) ? 1 : 0;
    int64_t per_oct = 8 * divide + f;
    int64_t ph_cyc = floordiv((ph + 10) * per_oct, 360000) & MASK32;
    int64_t pm_rise = (ph_cyc & 0xFF) == 0 ? 0 : ((ph_cyc & 0xFF) - (ph_cyc & 0xF8)) & 0xFF;
    int64_t dt = floordiv(floordiv((ph + 10) * per_oct, 8), 360000) & 0xFF;
    int64_t pm_rise_filt = pm_rise >= 8 ? pm_rise - 8 : pm_rise;
    int64_t pm_fall_frac = (pm_fall + pm_rise) & 0xFF;
    int64_t pm_fall_filt = (pm_fall + pm_rise - (pm_fall_frac & 0xF8)) & 0xFF;
    Phase p = phase(divide, ph);
    Counter c;
    c.reg1 = uint16_t((lt & 0x3F) | (ht & 0x3F) << 6 | (pm_rise_filt & 7) << 13);
    c.reg2 = uint16_t((dt & 0x3F) | p.mx << 8 | wf_rise << 10 | 1 << 11 | f << 12);
    c.transfer = int(wf_fall | (pm_fall_filt & 7) << 1);
    return c;
}

// The cell, as the calculation needs it.
struct Cell
{
    bool mmcm;
    // numeric parameter by name, or the default
    std::function<double(const std::string &, double)> num;
    // string parameter by name, or the default
    std::function<std::string(const std::string &, const std::string &)> str;
    // is this output (CLKOUT0..6) connected
    std::function<bool(const std::string &)> connected;
};

// The lock/filter table index for a multiplier, or -1 when out of range.
inline int table_index(const Cell &c, double mult)
{
    int m = int(mult + 0.5);
    int n = c.mmcm ? int(sizeof(mmcm_lock) / sizeof(mmcm_lock[0])) : int(sizeof(pll_lock) / sizeof(pll_lock[0]));
    return (m >= 1 && m <= n) ? m - 1 : -1;
}

// Every register, named as segbits_cmt_l.corrections names them
// (CLKOUT0_REG1, ..., DIVCLK, LOCK1-3, FILT1-2). Empty if the multiplier is
// outside the tables.
inline std::vector<std::pair<std::string, uint16_t>> registers(const Cell &c)
{
    std::vector<std::pair<std::string, uint16_t>> out;
    double mult = c.num(c.mmcm ? "CLKFBOUT_MULT_F" : "CLKFBOUT_MULT", 5.0);
    int ti = table_index(c, mult);
    if (ti < 0)
        return out;

    std::vector<std::pair<std::string, std::string>> outs; // counter, divide parameter
    if (c.mmcm) {
        outs.emplace_back("CLKOUT0", "CLKOUT0_DIVIDE_F");
        for (int o = 1; o <= 6; o++)
            outs.emplace_back("CLKOUT" + std::to_string(o), "CLKOUT" + std::to_string(o) + "_DIVIDE");
        outs.emplace_back("CLKFBOUT", "CLKFBOUT_MULT_F");
    } else {
        outs = {{"CLKOUT0", "CLKOUT0_DIVIDE"}, {"CLKOUT1", "CLKOUT1_DIVIDE"}, {"CLKFBOUT", "CLKFBOUT_MULT"}};
    }
    std::vector<Counter> regs;
    for (auto &o : outs) {
        double d = c.num(o.second, 1.0);
        int64_t ph = std::llround(c.num(o.first + "_PHASE", 0.0) * 1000);
        int64_t duty = std::llround(c.num(o.first + "_DUTY_CYCLE", 0.5) * 100000);
        int64_t di = int64_t(d);
        int64_t frac = std::llround((d - di) * 1000);
        regs.push_back(frac ? frac_count_calc(di, ph, frac) : count_calc(di, ph, duty));
    }
    auto idx = [&](const std::string &name) {
        for (size_t i = 0; i < outs.size(); i++)
            if (outs[i].first == name)
                return i;
        return outs.size();
    };
    if (c.mmcm) {
        // CLKOUT0's falling-edge fraction lives in CLKOUT5's REG2, CLKFBOUT's
        // in CLKOUT6's.
        for (auto &p : {std::make_pair("CLKOUT0", "CLKOUT5"), std::make_pair("CLKFBOUT", "CLKOUT6")}) {
            Counter &host = regs[idx(p.second)];
            host.reg2 = uint16_t((host.reg2 & 0xFFF) | regs[idx(p.first)].transfer << 12);
        }
    }
    bool cascade = c.mmcm && c.str("CLKOUT4_CASCADE", "FALSE") == "TRUE";
    for (size_t i = 0; i < outs.size(); i++) {
        const std::string &name = outs[i].first;
        bool oe = name == "CLKFBOUT" || c.connected(name) || (name == "CLKOUT6" && cascade);
        regs[i].reg1 |= uint16_t(oe) << 12;
        if (name == "CLKOUT4" && cascade)
            regs[i].reg2 |= 1 << 9;
        out.emplace_back(name + "_REG1", regs[i].reg1);
        out.emplace_back(name + "_REG2", regs[i].reg2);
    }
    Divider dv = divider(int64_t(c.num("DIVCLK_DIVIDE", 1.0)), 50000);
    out.emplace_back("DIVCLK", uint16_t(dv.low | dv.high << 6 | dv.no_count << 12 | dv.edge << 13));

    uint64_t lk = c.mmcm ? mmcm_lock[ti] : pll_lock[ti];
    auto lkb = [&](int lo, int hi) { return uint16_t((lk >> lo) & ((1u << (hi - lo + 1)) - 1)); };
    out.emplace_back("LOCK1", lkb(20, 29));
    out.emplace_back("LOCK2", uint16_t(lkb(30, 34) << 10 | lkb(0, 9)));
    out.emplace_back("LOCK3", uint16_t(int(c.mmcm) << 15 | lkb(35, 39) << 10 | lkb(10, 19)));
    uint16_t ft;
    if (c.mmcm)
        ft = c.str("BANDWIDTH", "OPTIMIZED") == "LOW" ? mmcm_filter_low[ti] : mmcm_filter_high[ti];
    else
        ft = pll_filter[ti];
    auto tb = [&](int i) { return (ft >> i) & 1; };
    out.emplace_back("FILT1", uint16_t(1 << 3 | tb(6) << 8 | tb(7) << 11 | tb(8) << 12 | tb(9) << 15));
    out.emplace_back("FILT2",
                     uint16_t(tb(0) << 4 | tb(1) << 7 | tb(2) << 8 | tb(3) << 11 | tb(4) << 12 | tb(5) << 15));
    return out;
}

} // namespace UspDrp

#endif
