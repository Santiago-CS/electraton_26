"""Minimal KiCad s-expression parser / writer.

Atoms are kept as Python str. Quoted strings become `QStr` (a str subclass) so the
writer can re-quote them; bare symbols stay plain `str`. Numbers are left as bare
strings (keeps exact formatting).
"""


class QStr(str):
    """A string that was quoted in the source (or must be quoted on output)."""


def parse(text):
    tokens = _tokenize(text)
    pos = 0

    def read():
        nonlocal pos
        tok = tokens[pos]
        if tok == "(":
            pos += 1
            lst = []
            while tokens[pos] != ")":
                lst.append(read())
            pos += 1
            return lst
        pos += 1
        return tok

    out = []
    while pos < len(tokens):
        out.append(read())
    return out[0] if len(out) == 1 else out


def _tokenize(text):
    toks = []
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if c in " \t\r\n":
            i += 1
        elif c in "()":
            toks.append(c)
            i += 1
        elif c == '"':
            j = i + 1
            buf = []
            while text[j] != '"':
                if text[j] == "\\":
                    nxt = text[j + 1]
                    buf.append({"n": "\n", "t": "\t", '"': '"', "\\": "\\"}.get(nxt, nxt))
                    j += 2
                else:
                    buf.append(text[j])
                    j += 1
            toks.append(QStr("".join(buf)))
            i = j + 1
        else:
            j = i
            while j < n and text[j] not in ' \t\r\n()"':
                j += 1
            toks.append(text[i:j])
            i = j
    return toks


def _atom(a):
    if isinstance(a, QStr):
        return '"' + a.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n") + '"'
    if isinstance(a, bool):
        return "yes" if a else "no"
    if isinstance(a, float):
        s = f"{a:.4f}".rstrip("0").rstrip(".")
        return "0" if s in ("-0", "") else s
    return str(a)


# Lists whose children are all atoms and short are written on one line.
_INLINE_HEADS = {"at", "size", "xy", "start", "end", "mid", "center", "radius", "length",
                 "width", "type", "color", "offset", "version", "generator",
                 "generator_version", "uuid", "paper", "unit", "diameter", "hide",
                 "in_bom", "on_board", "exclude_from_sim", "dnp", "lib_id", "reference",
                 "page", "justify", "thickness", "bold", "italic", "shape", "fields_autoplaced",
                 "embedded_fonts", "in_pos_files", "mirror", "number", "name"}


def dumps(node, indent=0):
    t = "\t" * indent
    if not isinstance(node, list):
        return t + _atom(node)
    if not node:
        return t + "()"
    head = node[0]
    all_atoms = all(not isinstance(x, list) for x in node)
    if all_atoms:
        return t + "(" + " ".join(_atom(x) for x in node) + ")"
    # pts list: keep xy on a single line
    if head == "pts":
        return t + "(pts\n" + t + "\t" + " ".join(dumps(x, 0) for x in node[1:]) + "\n" + t + ")"
    parts = [t + "(" + _atom(head)]
    # leading atoms stay on the head line
    k = 1
    while k < len(node) and not isinstance(node[k], list):
        parts[0] += " " + _atom(node[k])
        k += 1
    for child in node[k:]:
        parts.append(dumps(child, indent + 1))
    parts.append(t + ")")
    return "\n".join(parts)


def find(node, head):
    """First child list with the given head."""
    for c in node:
        if isinstance(c, list) and c and c[0] == head:
            return c
    return None


def find_all(node, head):
    return [c for c in node if isinstance(c, list) and c and c[0] == head]
