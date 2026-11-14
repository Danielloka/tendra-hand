"""Generate a KiCad 7 schematic from a Python description.

Every symbol pin must be assigned a net (or None = no-connect). Each pin gets a
short wire stub ending in a net label (or a GND / +3V3 power symbol), so the
schematic is "label style": connectivity is explicit and checkable.
"""
import copy
import os
import uuid as _uuid

from sexp import Q, dump, find, first, parse

KICAD_SYM = os.environ.get("KICAD9_SYMBOL_DIR", "/usr/share/kicad/symbols")
POWER_NETS = {"GND": "power:GND", "+3V3": "power:+3V3"}
STUB = 2.54
_NS = _uuid.UUID("6f1d2c3e-0000-4000-8000-746e64726100")


def uid(*parts):
    return Q(str(_uuid.uuid5(_NS, "/".join(map(str, parts)))))


class Library:
    def __init__(self, extra=None):
        self.libs = {}
        self.extra = extra or {}  # "Lib:Name" -> symbol s-expr (custom)

    def _lib(self, lib):
        if lib not in self.libs:
            tree = parse(open(f"{KICAD_SYM}/{lib}.kicad_sym").read())[0]
            self.libs[lib] = {s[1]: s for s in find(tree, "symbol")}
        return self.libs[lib]

    def get(self, lib_id):
        """Return a flattened symbol named lib_id (for lib_symbols)."""
        if lib_id in self.extra:
            sym = copy.deepcopy(self.extra[lib_id])
            sym[1] = Q(lib_id)
            return sym
        lib, name = lib_id.split(":")
        syms = self._lib(lib)
        sym = copy.deepcopy(syms[name])
        ext = first(sym, "extends")
        if ext:
            parent = copy.deepcopy(syms[ext[1]])
            pname = ext[1]
            body = [c for c in parent[2:] if not (isinstance(c, list) and c[0] == "property")]
            for c in body:
                if isinstance(c, list) and c[0] == "symbol":
                    c[1] = Q(c[1].replace(pname, name, 1))
            props = [c for c in sym[2:] if isinstance(c, list) and c[0] == "property"]
            flags = [c for c in parent[2:] if isinstance(c, list) and c[0] in ("power", "pin_names", "pin_numbers", "in_bom", "on_board", "exclude_from_sim")]
            body = [c for c in body if not (isinstance(c, list) and c[0] in ("power", "pin_names", "pin_numbers", "in_bom", "on_board", "exclude_from_sim"))]
            sym = ["symbol", Q(name)] + flags + props + body
        sym[1] = Q(lib_id)
        return sym

    @staticmethod
    def pins(sym):
        out = []

        def walk(n):
            for c in n:
                if isinstance(c, list):
                    if c and c[0] == "pin":
                        at = first(c, "at")
                        out.append({
                            "num": str(first(c, "number")[1]),
                            "name": str(first(c, "name")[1]),
                            "type": c[1],
                            "x": float(at[1]), "y": float(at[2]), "a": int(float(at[3])),
                        })
                    else:
                        walk(c)

        walk(sym)
        return out


def _r(v):
    return round(v / 1.27) * 1.27 if abs(v / 1.27 - round(v / 1.27)) < 1e-6 else round(v, 4)


class Schematic:
    def __init__(self, name, lib, paper="A2", title="", rev="", date="", company="", comments=()):
        self.name, self.lib, self.paper = name, lib, paper
        self.root = uid(name, "root")
        self.items, self.lib_syms, self.power_n = [], {}, 0
        self.title = dict(title=title, rev=rev, date=date, company=company, comments=comments)
        self.netmap = {}  # (ref, pin) -> net (for verification)
        self.refs = set()

    def _use(self, lib_id):
        if lib_id not in self.lib_syms:
            self.lib_syms[lib_id] = self.lib.get(lib_id)
        return self.lib_syms[lib_id]

    def _instance(self, lib_id, ref, value, x, y, rot=0, footprint="", fields=None, hide_ref=False, hide_val=False):
        sym = self._use(lib_id)
        pins = Library.pins(sym)
        def libpos(name, dy):
            lp = [c for c in sym if isinstance(c, list) and c[:2] == ["property", name]]
            if lp:
                at = first(lp[0], "at")
                eff = copy.deepcopy(first(lp[0], "effects")) or ["effects", ["font", ["size", 1.27, 1.27]]]
                eff = [e for e in eff if e != "hide" and not (isinstance(e, list) and e[:1] == ["hide"])]
                ang = int(float(at[3])) if len(at) > 3 else 0
                if rot:  # rotated power symbols: keep text horizontal near the point
                    return ["at", x, y + dy, 0], ["effects", ["font", ["size", 1.27, 1.27]]]
                return ["at", round(x + float(at[1]), 4), round(y - float(at[2]), 4), ang], eff
            return ["at", x, y + dy, 0], ["effects", ["font", ["size", 1.27, 1.27]]]

        rat, reff = libpos("Reference", -3.81)
        vat, veff = libpos("Value", 3.81)
        props = [
            ["property", Q("Reference"), Q(ref), rat, reff + (["hide"] if hide_ref else [])],
            ["property", Q("Value"), Q(value), vat, veff + (["hide"] if hide_val else [])],
            ["property", Q("Footprint"), Q(footprint), ["at", x, y, 0], ["effects", ["font", ["size", 1.27, 1.27]], "hide"]],
            ["property", Q("Datasheet"), Q(""), ["at", x, y, 0], ["effects", ["font", ["size", 1.27, 1.27]], "hide"]],
        ]
        for k, v in (fields or {}).items():
            props.append(["property", Q(k), Q(v), ["at", x, y, 0], ["effects", ["font", ["size", 1.27, 1.27]], "hide"]])
        node = ["symbol", ["lib_id", Q(lib_id)], ["at", x, y, rot], ["unit", 1],
                ["in_bom", "yes"], ["on_board", "yes"], ["dnp", "no"], ["fields_autoplaced"],
                ["uuid", uid(self.name, "sym", ref)]] + props
        for p in sorted({p["num"] for p in pins}):
            node.append(["pin", Q(p), ["uuid", uid(self.name, "pin", ref, p)]])
        node.append(["instances", ["project", Q(self.name), ["path", Q("/" + self.root), ["reference", Q(ref)], ["unit", 1]]]])
        self.items.append(node)
        return pins

    @staticmethod
    def _xf(px, py, x, y):
        return x + px, y - py

    @staticmethod
    def _out(a):
        # pin angle = direction from the connection point into the body
        return {0: (-1, 0), 180: (1, 0), 90: (0, 1), 270: (0, -1)}[a]

    def _endpoint(self, ref, net, ex, ey, dx, dy):
        sx, sy = ex + dx * STUB, ey + dy * STUB
        self.items.append(["wire", ["pts", ["xy", ex, ey], ["xy", sx, sy]],
                           ["stroke", ["width", 0], ["type", "default"]], ["uuid", uid(self.name, "w", ref, ex, ey)]])
        if net in POWER_NETS:
            self.power_n += 1
            # keep power symbols upright; flip only when the stub points against them
            horiz = dy == 0
            if net == "+3V3":
                rot = {(0, 1): 180, (0, -1): 0, (-1, 0): 90, (1, 0): 270}[(dx, dy)]
            else:
                rot = {(0, 1): 0, (0, -1): 180, (-1, 0): 270, (1, 0): 90}[(dx, dy)]
            pref = f"#PWR{self.power_n:03d}"
            self._instance(POWER_NETS[net], pref, net, sx, sy, rot, hide_ref=True, hide_val=horiz)
        else:
            ang = {(1, 0): 0, (-1, 0): 180, (0, -1): 90, (0, 1): 270}[(dx, dy)]
            just = "left" if ang in (0, 90) else "right"
            self.items.append(["label", Q(net), ["at", sx, sy, ang], ["fields_autoplaced"],
                               ["effects", ["font", ["size", 1.27, 1.27]], ["justify", just, "bottom"]],
                               ["uuid", uid(self.name, "lbl", ref, sx, sy)]])

    def part(self, ref, lib_id, value, x, y, conns, footprint="", fields=None):
        assert ref not in self.refs, ref
        self.refs.add(ref)
        pins = self._instance(lib_id, ref, value, x, y, 0, footprint, fields)
        nums = {p["num"] for p in pins}
        missing = nums - set(conns)
        extra = set(conns) - nums
        assert not missing and not extra, f"{ref}: missing {sorted(missing)} extra {sorted(extra)}"
        done = {}
        for p in pins:
            net = conns[p["num"]]
            self.netmap[(ref, p["num"])] = net
            ex, ey = self._xf(p["x"], p["y"], x, y)
            key = (round(ex, 3), round(ey, 3))
            if key in done:
                assert done[key] == net, f"{ref} pin {p['num']} shares a point with another net"
                continue
            done[key] = net
            if net is None:
                self.items.append(["no_connect", ["at", ex, ey], ["uuid", uid(self.name, "nc", ref, p["num"])]])
            else:
                dx, dy = self._out(p["a"])
                self._endpoint(ref, net, ex, ey, dx, dy)

    def flag(self, ref, net, x, y):
        """PWR_FLAG whose pin is at (x, y), with a stub down to a label."""
        self._instance("power:PWR_FLAG", ref, "PWR_FLAG", x, y, 0, hide_ref=True)
        self._endpoint(ref, net, x, y, 0, 1)

    def text(self, s, x, y, size=2.54, bold=True):
        eff = ["effects", ["font", ["size", size, size]] + (["bold"] if bold else []), ["justify", "left", "bottom"]]
        self.items.append(["text", Q(s), ["at", x, y, 0], eff, ["uuid", uid(self.name, "txt", s[:40], x, y)]])

    def box(self, x1, y1, x2, y2):
        self.items.append(["rectangle", ["start", x1, y1], ["end", x2, y2],
                           ["stroke", ["width", 0.2], ["type", "dash"]], ["fill", ["type", "none"]],
                           ["uuid", uid(self.name, "box", x1, y1)]])

    def write(self, path):
        t = self.title
        tb = ["title_block", ["title", Q(t["title"])], ["date", Q(t["date"])], ["rev", Q(t["rev"])], ["company", Q(t["company"])]]
        for i, c in enumerate(t["comments"], 1):
            tb.append(["comment", i, Q(c)])
        doc = ["kicad_sch", ["version", 20230121], ["generator", "eeschema"], ["uuid", self.root],
               ["paper", Q(self.paper)], tb, ["lib_symbols"] + list(self.lib_syms.values())]
        doc += self.items
        doc.append(["sheet_instances", ["path", Q("/"), ["page", Q("1")]]])
        with open(path, "w") as f:
            f.write(dump(doc) + "\n")


def custom_symbol(name, ref, value, footprint, datasheet, left, right, bottom, width=15.24, desc=""):
    """Rectangle symbol. left/right: list of (num, name, type) or None (gap). bottom: list of (num, name, type)."""
    n = max(len(left), len(right))
    h = (n + 3) * 2.54
    top = h / 2
    top = round(top / 2.54) * 2.54
    eff = ["effects", ["font", ["size", 1.27, 1.27]]]
    sym = ["symbol", Q(name), ["in_bom", "yes"], ["on_board", "yes"],
           ["property", Q("Reference"), Q(ref), ["at", 0, top + 5.08, 0], eff],
           ["property", Q("Value"), Q(value), ["at", 0, top + 2.54, 0], eff],
           ["property", Q("Footprint"), Q(footprint), ["at", 0, -top - 8.89, 0], eff + ["hide"]],
           ["property", Q("Datasheet"), Q(datasheet), ["at", 0, -top - 11.43, 0], eff + ["hide"]],
           ["property", Q("Description"), Q(desc), ["at", 0, 0, 0], eff + ["hide"]]]
    base = name.split(":")[-1]
    hw = width / 2
    body = ["symbol", Q(f"{base}_0_1"), ["rectangle", ["start", -hw, top], ["end", hw, -top - 2.54],
                                          ["stroke", ["width", 0.254], ["type", "default"]], ["fill", ["type", "background"]]]]
    pins = ["symbol", Q(f"{base}_1_1")]

    def pin(num, pname, typ, x, y, a):
        pins.append(["pin", typ, "line", ["at", x, y, a], ["length", 2.54],
                     ["name", Q(pname), eff], ["number", Q(num), eff]])

    for i, p in enumerate(left):
        if p:
            pin(p[0], p[1], p[2], -hw - 2.54, top - 2.54 * (i + 1), 0)
    for i, p in enumerate(right):
        if p:
            pin(p[0], p[1], p[2], hw + 2.54, top - 2.54 * (i + 1), 180)
    k = len(bottom)
    for i, p in enumerate(bottom):
        bx = (i - (k - 1) / 2) * 5.08
        pin(p[0], p[1], p[2], round(bx / 1.27) * 1.27, -top - 5.08, 90)
    sym += [body, pins]
    return sym
