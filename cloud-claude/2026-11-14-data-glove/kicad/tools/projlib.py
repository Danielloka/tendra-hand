"""Write the project library (symbols + footprints) and project files for GW-POD."""
import copy, os, shutil
from sexp import Q, dump
import gw_pod  # noqa: builds the schematic and defines the custom symbols

OUT = gw_pod.OUT
# --- symbols ---
syms = []
for s in (gw_pod.MAX17048, gw_pod.TPS63802):
    s = copy.deepcopy(s); s[1] = Q(s[1].split(":")[1]); syms.append(s)
with open(os.path.join(OUT, "Tendra_Glove.kicad_sym"), "w") as f:
    f.write(dump(["kicad_symbol_lib", ["version", 20220914], ["generator", "tendra_schgen"]] + syms) + "\n")

# --- TPS63802 DLA0010A footprint (TI SLVSEU9D land pattern example, exact pad sizes) ---
pads = [(str(i + 1), -0.9, -1.0 + 0.5 * i, 0.6, 0.25) for i in range(5)]
pads += [("10", 0.75, -1.0, 0.9, 0.25), ("9", 0.75, -0.5, 0.9, 0.25), ("8", 0.55, 0.0, 1.3, 0.25),
         ("7", 0.75, 0.5, 0.9, 0.25), ("6", 0.75, 1.0, 0.9, 0.25)]
def line(x1, y1, x2, y2, layer, w):
    return ["fp_line", ["start", x1, y1], ["end", x2, y2], ["stroke", ["width", w], ["type", "solid"]], ["layer", Q(layer)]]
fp = ["footprint", Q("TI_DLA0010A_VSON-HR-10_2x3mm"), ["version", 20221018], ["generator", "tendra_schgen"], ["layer", Q("F.Cu")],
      ["descr", Q("TI DLA0010A VSON-HR 10-pin 2x3 mm (TPS63802). Pads from SLVSEU9D land pattern example (non-solder-mask-defined).")],
      ["tags", Q("TPS63802 DLA VSON-HR HotRod")], ["attr", "smd"],
      ["fp_text", "reference", Q("REF**"), ["at", 0, -2.5], ["layer", Q("F.SilkS")], ["effects", ["font", ["size", 1, 1], ["thickness", 0.15]]]],
      ["fp_text", "value", Q("TI_DLA0010A"), ["at", 0, 2.5], ["layer", Q("F.Fab")], ["effects", ["font", ["size", 1, 1], ["thickness", 0.15]]]],
      ]
for x1, y1, x2, y2 in [(-1.0, -1.5, 1.0, -1.5), (1.0, -1.5, 1.0, 1.5), (1.0, 1.5, -1.0, 1.5), (-1.0, 1.5, -1.0, -1.5)]:
    fp.append(line(x1, y1, x2, y2, "F.Fab", 0.1))
for x1, y1, x2, y2 in [(-1.55, -1.85, 1.55, -1.85), (1.55, -1.85, 1.55, 1.85), (1.55, 1.85, -1.55, 1.85), (-1.55, 1.85, -1.55, -1.85)]:
    fp.append(line(x1, y1, x2, y2, "F.CrtYd", 0.05))
fp += [line(-0.4, -1.62, 0.4, -1.62, "F.SilkS", 0.12), line(-0.4, 1.62, 0.4, 1.62, "F.SilkS", 0.12),
       ["fp_circle", ["center", -1.35, -1.55], ["end", -1.25, -1.55], ["stroke", ["width", 0.12], ["type", "solid"]], ["fill", "solid"], ["layer", Q("F.SilkS")]]]
for n, x, y, w, h in pads:
    fp.append(["pad", Q(n), "smd", "roundrect", ["at", x, y], ["size", w, h], ["layers", Q("F.Cu"), Q("F.Paste"), Q("F.Mask")], ["roundrect_rratio", 0.2]])
os.makedirs(os.path.join(OUT, "Tendra_Glove.pretty"), exist_ok=True)
with open(os.path.join(OUT, "Tendra_Glove.pretty", "TI_DLA0010A_VSON-HR-10_2x3mm.kicad_mod"), "w") as f:
    f.write(dump(fp) + "\n")

# --- project + library tables ---
with open(os.path.join(OUT, "sym-lib-table"), "w") as f:
    f.write('(sym_lib_table\n  (version 7)\n  (lib (name "Tendra_Glove")(type "KiCad")(uri "${KIPRJMOD}/Tendra_Glove.kicad_sym")(options "")(descr "Tendra glove custom symbols"))\n)\n')
with open(os.path.join(OUT, "fp-lib-table"), "w") as f:
    f.write('(fp_lib_table\n  (version 7)\n  (lib (name "Tendra_Glove")(type "KiCad")(uri "${KIPRJMOD}/Tendra_Glove.pretty")(options "")(descr "Tendra glove custom footprints"))\n)\n')
pro = os.path.join(OUT, "glove_pod.kicad_pro")
if not os.path.exists(pro):
    shutil.copy("/usr/share/kicad/template/kicad.kicad_pro", pro)
print("library + project written")
