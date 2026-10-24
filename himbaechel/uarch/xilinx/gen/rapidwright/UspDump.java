package dev.fpga.rapidwright;

import com.xilinx.rapidwright.design.Design;
import com.xilinx.rapidwright.design.SiteInst;
import com.xilinx.rapidwright.device.*;

import java.io.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.*;

/**
 * Dump an UltraScale+ device from RapidWright in the prjxray-shaped files
 * that xilinx_device.import_device() reads, so xilinx_gen.py can build a
 * himbaechel chipdb for a part prjxray/prjuray cannot describe:
 *
 *   OUT/tile_type_<T>.json            wires, pips, site pins (prjxray shape)
 *   OUT/meta/site_type_<S>.json       bels / pins / site pips, every variant
 *   OUT/meta/wire_intents.json
 *   OUT/<fabric>/tilegrid.json
 *   OUT/<fabric>/nodes.bin            exact nodes (replaces tileconn.json)
 *   OUT/<fabric><package>-<speed>/package_pins.csv
 *
 * Only USED tile wires are written: pip endpoints and site-pin wires. Every
 * other wire is a pass-through that xilinx_gen.py would prune from its node
 * anyway, and keeping them puts PCIE4_PCIE4_FT (25,297 tile wires, 74 pips)
 * over himbaechel's int16 per-tile-type wire index. nodes.bin is written in
 * the remapped (used-only) indices, so the two always agree.
 *
 * nodes.bin, little-endian: u32 node count, then per node
 *   u32 n, then n x (u16 tile_x, u16 tile_y, u32 wire_index).
 * Nodes with fewer than two used wires are omitted.
 */
public class UspDump {
    static String q(String s) {
        StringBuilder b = new StringBuilder("\"");
        for (char c : s.toCharArray()) {
            if (c == '"' || c == '\\') b.append('\\');
            b.append(c);
        }
        return b.append('"').toString();
    }

    static String prefixOf(String siteName) {
        return siteName.substring(0, siteName.lastIndexOf('_'));
    }

    // Site position relative to the lowest site with the same name prefix in
    // the tile -- exactly what xilinx_device.Site.rel_xy() computes, which the
    // generator uses to key site pins.
    static int[] relXY(Tile t, Site s) {
        String p = prefixOf(s.getName());
        int bx = Integer.MAX_VALUE, by = Integer.MAX_VALUE;
        for (Site o : t.getSites()) {
            if (!prefixOf(o.getName()).equals(p)) continue;
            bx = Math.min(bx, o.getInstanceX());
            by = Math.min(by, o.getInstanceY());
        }
        return new int[]{s.getInstanceX() - bx, s.getInstanceY() - by};
    }

    static String dirName(BELPin bp) {
        return bp.isBidir() ? "BIDIR" : (bp.isOutput() ? "OUTPUT" : "INPUT");
    }

    static List<SiteTypeEnum> variantsOf(Site s) {
        List<SiteTypeEnum> v = new ArrayList<>();
        v.add(s.getSiteTypeEnum());
        v.addAll(Arrays.asList(s.getAlternateSiteTypeEnums()));
        return v;
    }

    static int autoidx = 0;

    // Site pins of one variant: name -> {site wire, primary pin name}. A site
    // pin shows up as the connected site pin of some BEL pin.
    static Map<String, String[]> variantPins(Site s, SiteInst si) {
        Map<String, String[]> pins = new TreeMap<>();
        for (BEL b : si.getBELs()) {
            for (BELPin bp : b.getPins()) {
                String sp = bp.getConnectedSitePinName();
                if (sp == null || bp.getSiteWireName() == null || pins.containsKey(sp)) continue;
                String prim = (s.getSiteTypeEnum() == si.getSiteTypeEnum()) ? sp : si.getPrimarySitePinName(sp);
                if (prim == null) continue;
                String dir = s.isOutputPin(prim) ? "OUTPUT" : "INPUT";
                pins.put(sp, new String[]{bp.getSiteWireName(), prim, dir});
            }
        }
        return pins;
    }

    public static void main(String[] args) throws IOException {
        if (args.length < 3) {
            System.err.println("usage: UspDump <part> <fabric> <outdir>   e.g. UspDump xck26-sfvc784-2LV-c xck26 out");
            System.exit(1);
        }
        String part = args[0], fabric = args[1];
        File out = new File(args[2]);
        File meta = new File(out, "meta"), fab = new File(out, fabric);
        meta.mkdirs();
        fab.mkdirs();

        Design des = new Design("top", part);
        Device d = des.getDevice();

        // ---- tile types: used wires, pips, site pins; site types alongside
        Map<TileTypeEnum, int[]> usedRemap = new HashMap<>(); // orig wire -> used index, -1 if pruned
        Map<String, Integer> intentIdx = new LinkedHashMap<>();
        StringBuilder intentTiles = new StringBuilder();
        Set<SiteTypeEnum> seenSiteTypes = new HashSet<>();
        int tileTypes = 0;
        for (Tile t : d.getAllTiles()) {
            TileTypeEnum tt = t.getTileTypeEnum();
            if (usedRemap.containsKey(tt)) continue;
            tileTypes++;

            // Collect every variant's site pins first: their tile wires are
            // part of the used set.
            BitSet used = new BitSet(t.getWireCount());
            for (PIP p : t.getPIPs()) {
                used.set(p.getStartWireIndex());
                used.set(p.getEndWireIndex());
            }
            List<String> siteJson = new ArrayList<>();
            for (Site s : t.getSites()) {
                Map<String, Integer> sitePinWire = new TreeMap<>();
                StringBuilder sty = seenSiteTypes.add(s.getSiteTypeEnum()) ? new StringBuilder("{") : null;
                boolean firstVar = true;
                for (SiteTypeEnum v : variantsOf(s)) {
                    SiteInst si = new SiteInst(t.getName() + "_" + (autoidx++), des, v, s);
                    Map<String, String[]> pins = variantPins(s, si);
                    for (String[] pv : pins.values()) {
                        int w = s.getTileWireIndexFromPinName(pv[1]);
                        if (w >= 0) {
                            used.set(w);
                            sitePinWire.put(pv[1], w);
                        }
                    }
                    if (sty != null) {
                        if (!firstVar) sty.append(",");
                        firstVar = false;
                        sty.append(q(v.name())).append(":{\"bels\":{");
                        boolean fb = true;
                        Set<String> belNames = new HashSet<>();
                        StringBuilder pips = new StringBuilder();
                        Set<String> seenPip = new HashSet<>();
                        for (BEL b : si.getBELs()) {
                            if (b.getBELClass() == BELClass.PORT) continue;
                            belNames.add(b.getName());
                            if (!fb) sty.append(",");
                            fb = false;
                            sty.append(q(b.getName())).append(":{\"type\":").append(q(b.getBELType()))
                               .append(",\"class\":").append(q(b.getBELClass().name())).append(",\"pins\":{");
                            boolean fp = true;
                            for (BELPin bp : b.getPins()) {
                                if (bp.getSiteWireName() == null) continue;
                                if (!fp) sty.append(",");
                                fp = false;
                                sty.append(q(bp.getName())).append(":{\"dir\":").append(q(dirName(bp)))
                                   .append(",\"wire\":").append(q(bp.getSiteWireName())).append("}");
                            }
                            sty.append("}}");
                        }
                        sty.append("},\"pips\":[");
                        boolean fsp = true;
                        for (BEL b : si.getBELs()) {
                            for (BELPin bp : b.getPins()) {
                                for (SitePIP sp : bp.getSitePIPs()) {
                                    if (!belNames.contains(sp.getBELName())) continue;
                                    if (sp.getInputPin().getSiteWireName() == null || sp.getOutputPin().getSiteWireName() == null) continue;
                                    String key = sp.getBELName() + "/" + sp.getInputPinName() + "/" + sp.getOutputPinName();
                                    if (!seenPip.add(key)) continue;
                                    if (!fsp) sty.append(",");
                                    fsp = false;
                                    sty.append("{\"bel\":").append(q(sp.getBELName())).append(",\"from_pin\":")
                                       .append(q(sp.getInputPinName())).append(",\"to_pin\":").append(q(sp.getOutputPinName())).append("}");
                                }
                            }
                        }
                        sty.append("],\"pins\":{");
                        boolean fpn = true;
                        for (Map.Entry<String, String[]> e : pins.entrySet()) {
                            if (!fpn) sty.append(",");
                            fpn = false;
                            sty.append(q(e.getKey())).append(":{\"primary\":").append(q(e.getValue()[1]))
                               .append(",\"wire\":").append(q(e.getValue()[0])).append(",\"dir\":").append(q(e.getValue()[2])).append("}");
                        }
                        sty.append("}}");
                    }
                    si.unPlace();
                }
                if (sty != null) {
                    sty.append("}");
                    try (Writer w = new BufferedWriter(new FileWriter(new File(meta, "site_type_" + s.getSiteTypeEnum().name() + ".json")))) {
                        w.write(sty.toString());
                    }
                }
                int[] rel = relXY(t, s);
                StringBuilder sj = new StringBuilder();
                sj.append("{\"name\":").append(q("X" + rel[0] + "Y" + rel[1]))
                  .append(",\"prefix\":").append(q(prefixOf(s.getName())))
                  .append(",\"type\":").append(q(s.getSiteTypeEnum().name())).append(",\"site_pins\":{");
                boolean f = true;
                for (Map.Entry<String, Integer> e : sitePinWire.entrySet()) {
                    if (!f) sj.append(",");
                    f = false;
                    sj.append(q(e.getKey())).append(":{\"wire\":").append(q(t.getWireName(e.getValue()))).append("}");
                }
                sj.append("}}");
                siteJson.add(sj.toString());
            }

            int[] remap = new int[t.getWireCount()];
            Arrays.fill(remap, -1);
            StringBuilder tj = new StringBuilder("{\"tile_type\":").append(q(tt.name())).append(",\"wires\":{");
            StringBuilder ij = new StringBuilder();
            int n = 0;
            for (int w = used.nextSetBit(0); w >= 0; w = used.nextSetBit(w + 1)) {
                remap[w] = n++;
                String wn = t.getWireName(w);
                if (n > 1) tj.append(",");
                tj.append(q(wn)).append(":null");
                String intent = t.getWireIntentCode(w).name();
                Integer ii = intentIdx.get(intent);
                if (ii == null) {
                    ii = intentIdx.size();
                    intentIdx.put(intent, ii);
                }
                if (ij.length() > 0) ij.append(",");
                ij.append(q(wn)).append(":").append(ii);
            }
            usedRemap.put(tt, remap);
            if (intentTiles.length() > 0) intentTiles.append(",");
            intentTiles.append(q(tt.name())).append(":{").append(ij).append("}");

            tj.append("},\"pips\":{");
            boolean fp = true;
            Set<String> seenPips = new HashSet<>();
            for (PIP p : t.getPIPs()) {
                String sw = t.getWireName(p.getStartWireIndex()), dw = t.getWireName(p.getEndWireIndex());
                String key = tt.name() + "." + sw + "->>" + dw;
                if (!seenPips.add(key)) continue;
                if (!fp) tj.append(",");
                fp = false;
                tj.append(q(key)).append(":{\"src_wire\":").append(q(sw)).append(",\"dst_wire\":").append(q(dw))
                  .append(",\"is_directional\":\"").append(p.isBidirectional() ? 0 : 1)
                  .append("\",\"is_pseudo\":\"").append(p.isRouteThru() ? 1 : 0).append("\"}");
            }
            tj.append("},\"sites\":[").append(String.join(",", siteJson)).append("]}");
            try (Writer w = new BufferedWriter(new FileWriter(new File(out, "tile_type_" + tt.name() + ".json")))) {
                w.write(tj.toString());
            }
            if (n > 32767)
                System.out.println("WARNING: " + tt.name() + " still has " + n + " used wires (int16 limit)");
        }
        System.out.println("tile types: " + tileTypes);

        try (Writer w = new BufferedWriter(new FileWriter(new File(meta, "wire_intents.json")))) {
            w.write("{\"intents\":{");
            boolean f = true;
            for (Map.Entry<String, Integer> e : intentIdx.entrySet()) {
                if (!f) w.write(",");
                f = false;
                w.write(q(String.valueOf(e.getValue())) + ":" + q(e.getKey()));
            }
            w.write("},\"tiles\":{" + intentTiles + "}}");
        }

        // ---- tilegrid
        try (Writer w = new BufferedWriter(new FileWriter(new File(fab, "tilegrid.json")))) {
            w.write("{");
            boolean f = true;
            for (Tile t : d.getAllTiles()) {
                if (!f) w.write(",");
                f = false;
                w.write(q(t.getName()) + ":{\"grid_x\":" + t.getColumn() + ",\"grid_y\":" + t.getRow()
                        + ",\"type\":" + q(t.getTileTypeEnum().name()) + ",\"sites\":{");
                boolean fs = true;
                for (Site s : t.getSites()) {
                    if (!fs) w.write(",");
                    fs = false;
                    w.write(q(s.getName()) + ":" + q(s.getSiteTypeEnum().name()));
                }
                w.write("}}");
            }
            w.write("}");
        }

        // ---- nodes, in used-wire indices
        long nodes = 0, refs = 0;
        File nodesFile = new File(fab, "nodes.bin");
        try (DataOutputStream os = new DataOutputStream(new BufferedOutputStream(new FileOutputStream(nodesFile), 1 << 20))) {
            ByteBuffer bb = ByteBuffer.allocate(8).order(ByteOrder.LITTLE_ENDIAN);
            os.writeInt(0); // patched below
            int[] xs = new int[4096], ys = new int[4096], ws = new int[4096];
            for (Tile t : d.getAllTiles()) {
                for (int w = 0; w < t.getWireCount(); w++) {
                    Node node = Node.getNode(t, w);
                    if (node == null || node.getTile() != t || node.getWireIndex() != w) continue;
                    int k = 0;
                    for (Wire nw : node.getAllWiresInNode()) {
                        int r = usedRemap.get(nw.getTile().getTileTypeEnum())[nw.getWireIndex()];
                        if (r < 0) continue;
                        if (k == xs.length) {
                            xs = Arrays.copyOf(xs, k * 2); ys = Arrays.copyOf(ys, k * 2); ws = Arrays.copyOf(ws, k * 2);
                        }
                        xs[k] = nw.getTile().getColumn(); ys[k] = nw.getTile().getRow(); ws[k] = r;
                        k++;
                    }
                    if (k < 2) continue;
                    bb.clear(); bb.putInt(k); os.write(bb.array(), 0, 4);
                    for (int i = 0; i < k; i++) {
                        bb.clear(); bb.putShort((short) xs[i]); bb.putShort((short) ys[i]); bb.putInt(ws[i]);
                        os.write(bb.array(), 0, 8);
                    }
                    nodes++;
                    refs += k;
                }
            }
        }
        try (RandomAccessFile raf = new RandomAccessFile(nodesFile, "rw")) {
            ByteBuffer bb = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN);
            bb.putInt((int) nodes);
            raf.write(bb.array());
        }
        System.out.println("nodes: " + nodes + " (" + refs + " used-wire refs)");

        // ---- package pins
        com.xilinx.rapidwright.device.Package pkg = d.getActivePackage();
        String speed = part.substring(part.indexOf(pkg.getName()) + pkg.getName().length());
        File pkgDir = new File(out, fabric + pkg.getName() + speed);
        pkgDir.mkdirs();
        try (Writer w = new BufferedWriter(new FileWriter(new File(pkgDir, "package_pins.csv")))) {
            w.write("pin,bank,site,tile,pin_function\n");
            for (PackagePin pp : pkg.getPackagePinMap().values()) {
                if (pp == null || pp.getSite() == null) continue;
                IOBank bank = pp.getIOBank();
                w.write(pp.getName() + "," + (bank == null ? "" : String.valueOf(bank.getId())) + "," + pp.getSite().getName() + ","
                        + pp.getSite().getTile().getName() + "," + pp.getName() + "\n");
            }
        }
        System.out.println("package " + pkg.getName() + " -> " + pkgDir);
    }
}
