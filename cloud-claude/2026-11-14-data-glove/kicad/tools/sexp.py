"""Minimal S-expression reader/writer for KiCad files."""
import re

_TOK = re.compile(r'\(|\)|"(?:\\.|[^"\\])*"|[^\s()]+')


class Q(str):
    """A string that must be written quoted."""


def parse(text):
    toks = _TOK.findall(text)
    pos = 0

    def node():
        nonlocal pos
        t = toks[pos]
        pos += 1
        if t == "(":
            out = []
            while toks[pos] != ")":
                out.append(node())
            pos += 1
            return out
        if t.startswith('"'):
            return Q(t[1:-1].replace('\\"', '"').replace("\\\\", "\\"))
        return t

    res = []
    while pos < len(toks):
        res.append(node())
    return res


def dump(x, indent=0):
    if isinstance(x, list):
        if not x:
            return "()"
        simple = all(not isinstance(e, list) for e in x)
        if simple or len(x) <= 2 and not any(isinstance(e, list) and len(e) > 3 for e in x):
            return "(" + " ".join(dump(e) for e in x) + ")"
        head = []
        rest = list(x)
        while rest and not isinstance(rest[0], list):
            head.append(dump(rest.pop(0)))
        pad = "  " * (indent + 1)
        inner = "\n".join(pad + dump(e, indent + 1) for e in rest)
        return "(" + " ".join(head) + "\n" + inner + "\n" + "  " * indent + ")"
    if isinstance(x, Q):
        return '"' + x.replace("\\", "\\\\").replace('"', '\\"') + '"'
    if isinstance(x, float):
        s = f"{x:.4f}".rstrip("0").rstrip(".")
        return "0" if s in ("-0", "") else s
    return str(x)


def find(node, key):
    return [c for c in node if isinstance(c, list) and c and c[0] == key]


def first(node, key):
    r = find(node, key)
    return r[0] if r else None
