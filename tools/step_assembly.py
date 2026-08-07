#!/usr/bin/env python3
"""Extract per-part placement transforms from an AP242 STEP assembly.

Pure standard library.  No CAD kernel required.

Motivation
----------
The WLKATA MT4 model ships as a STEP assembly plus a pile of per-part STL
meshes.  The STLs are all exported in their *own* local coordinate frames, so
to build a URDF/USD robot you need to know where each part sits in the
assembly.  That information lives in the STEP file, spread across
NEXT_ASSEMBLY_USAGE_OCCURRENCE / CONTEXT_DEPENDENT_SHAPE_REPRESENTATION /
REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION / ITEM_DEFINED_TRANSFORMATION
entity chains.  This script walks those chains and emits a flat list of leaf
parts with their 4x4 pose in the assembly root frame.

Entity chain that carries a component placement
-----------------------------------------------
    NEXT_ASSEMBLY_USAGE_OCCURRENCE(id, name, desc, parent_pd, child_pd, ref)
        ^ the parent/child relationship (in *product definition* space)
        |
    PRODUCT_DEFINITION_SHAPE('', '', #nauo)
        ^ .represented_product_relation points back at the occurrence
        |
    CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#rep_rel, #pds)
        ^ ties the occurrence to the geometric placement
        |
    (REPRESENTATION_RELATIONSHIP('', '', #rep_1, #rep_2)
     REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#idt)
     SHAPE_REPRESENTATION_RELATIONSHIP())
        |
    ITEM_DEFINED_TRANSFORMATION('', '', #item_1, #item_2)
        ^ item_1 lives in rep_1's frame, item_2 in rep_2's frame
        |
    AXIS2_PLACEMENT_3D('', #origin, #z_dir, #x_dir)

Per ISO 10303 assembly recommended practice, the relationship maps rep_1 into
rep_2.  Which of the two is the child is *not* assumed here: we resolve each
product definition to its own SHAPE_REPRESENTATION (through
SHAPE_DEFINITION_REPRESENTATION) and check which side of the relationship the
child's representation is on.  The placement is then

    T_child_in_parent = A_parent_side @ inverse(A_child_side)

Usage
-----
    python step_assembly.py [STEP file] [-o out.json]
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys

# ---------------------------------------------------------------------------
# Tokenizer / entity parser
# ---------------------------------------------------------------------------


def _strip_comments_and_split(data: str):
    """Split a STEP DATA section into raw ``#id=...`` records.

    Records are terminated by a semicolon at paren-depth zero that is not
    inside a quoted string.  Entity definitions may span any number of lines,
    so we cannot work line-by-line.  ``/* ... */`` comments are dropped, but
    only when they occur outside a string literal.
    """
    records = []
    buf = []
    i = 0
    n = len(data)
    in_string = False
    while i < n:
        ch = data[i]
        if in_string:
            if ch == "'":
                # '' is an escaped single quote inside a STEP string
                if i + 1 < n and data[i + 1] == "'":
                    buf.append("''")
                    i += 2
                    continue
                in_string = False
            buf.append(ch)
            i += 1
            continue
        if ch == "'":
            in_string = True
            buf.append(ch)
            i += 1
            continue
        if ch == "/" and i + 1 < n and data[i + 1] == "*":
            end = data.find("*/", i + 2)
            i = n if end == -1 else end + 2
            continue
        if ch == ";":
            rec = "".join(buf).strip()
            if rec:
                records.append(rec)
            buf = []
            i += 1
            continue
        buf.append(ch)
        i += 1
    rec = "".join(buf).strip()
    if rec:
        records.append(rec)
    return records


def _split_args(s: str):
    """Split a top-level argument list on commas.

    Respects nested parentheses and quoted strings.
    """
    args = []
    depth = 0
    in_string = False
    start = 0
    i = 0
    n = len(s)
    while i < n:
        ch = s[i]
        if in_string:
            if ch == "'":
                if i + 1 < n and s[i + 1] == "'":
                    i += 2
                    continue
                in_string = False
            i += 1
            continue
        if ch == "'":
            in_string = True
        elif ch in "([":
            depth += 1
        elif ch in ")]":
            depth -= 1
        elif ch == "," and depth == 0:
            args.append(s[start:i].strip())
            start = i + 1
        i += 1
    tail = s[start:].strip()
    if tail or args:
        args.append(tail)
    return args


_SIMPLE_RE = re.compile(r"^#(\d+)\s*=\s*(.*)$", re.S)
# NB: deliberately unanchored.  Pattern.match(s, pos) already anchors at pos,
# whereas a leading "^" would only ever match at index 0 -- which would silently
# drop every group after the first when scanning a complex entity.
_TYPE_RE = re.compile(r"\s*([A-Za-z_][A-Za-z0-9_]*)\s*\(", re.S)


def _match_paren(s: str, open_idx: int) -> int:
    """Return the index of the ``)`` matching the ``(`` at ``open_idx``."""
    depth = 0
    in_string = False
    i = open_idx
    n = len(s)
    while i < n:
        ch = s[i]
        if in_string:
            if ch == "'":
                if i + 1 < n and s[i + 1] == "'":
                    i += 2
                    continue
                in_string = False
            i += 1
            continue
        if ch == "'":
            in_string = True
        elif ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
            if depth == 0:
                return i
        i += 1
    raise ValueError("unbalanced parentheses")


def parse_step(path: str):
    """Parse a STEP file.

    Returns ``(simple, complex_)`` where

    * ``simple[id] = (TYPE, [args])`` for plain ``#id=TYPE(...)`` entities, and
    * ``complex_[id] = {TYPE: [args], ...}`` for grouped/"complex" entities of
      the form ``#id=(TYPE_A(...)TYPE_B(...))`` used for AP242 multiple
      inheritance.  Complex entities are *also* registered in ``simple`` under
      each of their component type names so lookups can be uniform.
    """
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        text = fh.read()

    start = text.find("DATA;")
    if start == -1:
        raise ValueError("no DATA; section found")
    end = text.find("ENDSEC;", start)
    if end == -1:
        end = len(text)
    data = text[start + len("DATA;"):end]

    simple = {}
    complex_ = {}
    for rec in _strip_comments_and_split(data):
        m = _SIMPLE_RE.match(rec)
        if not m:
            continue
        eid = int(m.group(1))
        body = m.group(2).strip()
        if body.startswith("("):
            # Complex entity: a run of TYPE(...) groups inside outer parens.
            inner = body[1:_match_paren(body, 0)]
            parts = {}
            pos = 0
            while pos < len(inner):
                tm = _TYPE_RE.match(inner, pos)
                if not tm:
                    break
                tname = tm.group(1).upper()
                op = tm.end() - 1
                cp = _match_paren(inner, op)
                parts[tname] = _split_args(inner[op + 1:cp])
                pos = cp + 1
            complex_[eid] = parts
            for tname, args in parts.items():
                simple.setdefault(eid, (tname, args))
        else:
            tm = _TYPE_RE.match(body)
            if not tm:
                continue
            tname = tm.group(1).upper()
            op = tm.end() - 1
            cp = _match_paren(body, op)
            simple[eid] = (tname, _split_args(body[op + 1:cp]))
    return simple, complex_


def entity_args(simple, complex_, eid, typename):
    """Fetch the args of ``typename`` from entity ``eid`` (simple or complex)."""
    typename = typename.upper()
    if eid in complex_ and typename in complex_[eid]:
        return complex_[eid][typename]
    ent = simple.get(eid)
    if ent and ent[0] == typename:
        return ent[1]
    return None


# ---------------------------------------------------------------------------
# Value helpers
# ---------------------------------------------------------------------------


def ref(arg):
    """``'#123'`` -> ``123``; anything else -> ``None``."""
    if isinstance(arg, str):
        a = arg.strip()
        if a.startswith("#"):
            try:
                return int(a[1:])
            except ValueError:
                return None
    return None


def reflist(arg):
    a = arg.strip()
    if a.startswith("(") and a.endswith(")"):
        a = a[1:-1]
    return [r for r in (ref(x) for x in _split_args(a)) if r is not None]


def numlist(arg):
    a = arg.strip()
    if a.startswith("(") and a.endswith(")"):
        a = a[1:-1]
    out = []
    for tok in _split_args(a):
        tok = tok.strip()
        if not tok or tok == "$":
            continue
        out.append(float(tok))
    return out


def decode_step_string(raw: str) -> str:
    """Decode an ISO 10303-21 string literal into Python text.

    Handles the ``\\X2\\<utf16be hex>\\X0\\`` and ``\\X4\\<utf32be hex>\\X0\\``
    extended-character escapes (used here for Chinese part names), the
    single-byte ``\\X\\hh`` escape, ``\\S\\c`` (ISO 8859 upper half), the
    ``\\P?\\`` page directive, and doubled quotes / backslashes.
    """
    s = raw.strip()
    if s.startswith("'") and s.endswith("'") and len(s) >= 2:
        s = s[1:-1]
    s = s.replace("''", "'")

    out = []
    i = 0
    n = len(s)
    while i < n:
        ch = s[i]
        if ch != "\\":
            out.append(ch)
            i += 1
            continue
        rest = s[i:]
        # \X2\<hex>\X0\  (UTF-16BE) and \X4\<hex>\X0\ (UTF-32BE)
        m = re.match(r"\\X([24])\\([0-9A-Fa-f]*?)\\X0\\", rest)
        if m:
            width = 4 if m.group(1) == "2" else 8
            hexs = m.group(2)
            enc = "utf-16-be" if width == 4 else "utf-32-be"
            try:
                out.append(bytes.fromhex(hexs).decode(enc, errors="replace"))
            except ValueError:
                out.append(hexs)
            i += m.end()
            continue
        # \X\hh  single byte
        m = re.match(r"\\X\\([0-9A-Fa-f]{2})", rest)
        if m:
            out.append(bytes([int(m.group(1), 16)]).decode("latin-1"))
            i += m.end()
            continue
        # \S\c  -> c + 0x80
        m = re.match(r"\\S\\(.)", rest)
        if m:
            out.append(bytes([ord(m.group(1)) + 0x80]).decode("latin-1"))
            i += m.end()
            continue
        # \P?\  code page directive: drop
        m = re.match(r"\\P.\\", rest)
        if m:
            i += m.end()
            continue
        # \\ -> literal backslash
        if rest.startswith("\\\\"):
            out.append("\\")
            i += 2
            continue
        out.append(ch)
        i += 1
    return "".join(out)


# ---------------------------------------------------------------------------
# Minimal 4x4 rigid transform math (row-major list of lists)
# ---------------------------------------------------------------------------


def identity4():
    return [[1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]]


def matmul4(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def invert_rigid4(m):
    """Inverse of a rigid transform: ``R^T`` with ``-R^T t``."""
    out = identity4()
    for i in range(3):
        for j in range(3):
            out[i][j] = m[j][i]
    for i in range(3):
        out[i][3] = -sum(m[k][i] * m[k][3] for k in range(3))
    return out


def _norm(v):
    return math.sqrt(sum(c * c for c in v))


def _unit(v, fallback):
    n = _norm(v)
    if n < 1e-12:
        return list(fallback)
    return [c / n for c in v]


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


class Model:
    """Thin accessor layer over the parsed entity tables."""

    def __init__(self, simple, complex_):
        self.simple = simple
        self.complex = complex_

    def args(self, eid, typename):
        return entity_args(self.simple, self.complex, eid, typename)

    def typeof(self, eid):
        ent = self.simple.get(eid)
        return ent[0] if ent else None

    def cartesian_point(self, eid):
        a = self.args(eid, "CARTESIAN_POINT")
        if a is None:
            return [0.0, 0.0, 0.0]
        c = numlist(a[1])
        while len(c) < 3:
            c.append(0.0)
        return c[:3]

    def direction(self, eid, fallback):
        if eid is None:
            return list(fallback)
        a = self.args(eid, "DIRECTION")
        if a is None:
            return list(fallback)
        c = numlist(a[1])
        while len(c) < 3:
            c.append(0.0)
        return _unit(c[:3], fallback)

    def axis2_placement_3d(self, eid):
        """Build a 4x4 matrix from an AXIS2_PLACEMENT_3D.

        ``AXIS2_PLACEMENT_3D(name, location, axis /*z*/, ref_direction /*x*/)``.
        The x axis is Gram-Schmidt orthogonalised against z, exactly as the
        STEP geometric model prescribes.
        """
        a = self.args(eid, "AXIS2_PLACEMENT_3D")
        if a is None:
            return identity4()
        loc = self.cartesian_point(ref(a[1])) if len(a) > 1 else [0.0] * 3
        z = self.direction(ref(a[2]) if len(a) > 2 else None, (0.0, 0.0, 1.0))
        xr = self.direction(ref(a[3]) if len(a) > 3 else None, (1.0, 0.0, 0.0))
        z = _unit(z, (0.0, 0.0, 1.0))
        x = [xr[i] - _dot(xr, z) * z[i] for i in range(3)]
        if _norm(x) < 1e-9:
            # degenerate ref_direction: pick any vector not parallel to z
            alt = (1.0, 0.0, 0.0) if abs(z[0]) < 0.9 else (0.0, 1.0, 0.0)
            x = [alt[i] - _dot(alt, z) * z[i] for i in range(3)]
        x = _unit(x, (1.0, 0.0, 0.0))
        y = _cross(z, x)
        m = identity4()
        for i in range(3):
            m[i][0], m[i][1], m[i][2] = x[i], y[i], z[i]
            m[i][3] = loc[i]
        return m


# ---------------------------------------------------------------------------
# Assembly resolution
# ---------------------------------------------------------------------------


class Occurrence:
    __slots__ = ("eid", "occ_id", "name", "parent_pd", "child_pd",
                 "transform", "transform_source", "children")

    def __init__(self, eid, occ_id, name, parent_pd, child_pd):
        self.eid = eid
        self.occ_id = occ_id
        self.name = name
        self.parent_pd = parent_pd
        self.child_pd = child_pd
        self.transform = identity4()
        self.transform_source = None
        self.children = []


def build_assembly(model, compose="parent_x_inv_child"):
    """Resolve the assembly graph.

    ``compose`` selects the composition order used to turn the two axis
    placements of an ITEM_DEFINED_TRANSFORMATION into a relative transform:

    * ``parent_x_inv_child`` -> ``A_parent @ inv(A_child)``  (correct per spec)
    * ``child_x_inv_parent`` -> ``A_child @ inv(A_parent)``  (for comparison)
    """
    simple, complex_ = model.simple, model.complex

    # -- product names -----------------------------------------------------
    # PRODUCT_DEFINITION(id, desc, formation, frame)
    #   -> PRODUCT_DEFINITION_FORMATION(...)(id, desc, of_product)
    #   -> PRODUCT(id, name, desc, frame_of_reference)
    pd_name = {}
    for eid, (tname, args) in simple.items():
        if tname != "PRODUCT_DEFINITION":
            continue
        name = None
        form = ref(args[2]) if len(args) > 2 else None
        if form is not None:
            fargs = (entity_args(simple, complex_, form,
                                 "PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE")
                     or entity_args(simple, complex_, form,
                                    "PRODUCT_DEFINITION_FORMATION"))
            if fargs and len(fargs) > 2:
                prod = ref(fargs[2])
                pargs = entity_args(simple, complex_, prod, "PRODUCT")
                if pargs and len(pargs) > 1:
                    name = decode_step_string(pargs[1]) or decode_step_string(pargs[0])
        pd_name[eid] = name or "PD#%d" % eid

    # -- product definition -> its own shape representation -----------------
    # SHAPE_DEFINITION_REPRESENTATION(definition /*PRODUCT_DEFINITION_SHAPE*/,
    #                                 used_representation /*SHAPE_REPRESENTATION*/)
    pd_shape_rep = {}
    for eid, (tname, args) in simple.items():
        if tname != "SHAPE_DEFINITION_REPRESENTATION":
            continue
        pds, rep = ref(args[0]), ref(args[1])
        pds_args = entity_args(simple, complex_, pds, "PRODUCT_DEFINITION_SHAPE")
        if not pds_args or len(pds_args) < 3 or rep is None:
            continue
        target = ref(pds_args[2])
        if target is not None and model.typeof(target) == "PRODUCT_DEFINITION":
            pd_shape_rep[target] = rep

    # -- occurrences --------------------------------------------------------
    occurrences = {}
    for eid, (tname, args) in simple.items():
        if tname not in ("NEXT_ASSEMBLY_USAGE_OCCURRENCE",
                         "ASSEMBLY_COMPONENT_USAGE",
                         "SPECIFIED_HIGHER_USAGE_OCCURRENCE"):
            continue
        occurrences[eid] = Occurrence(
            eid,
            decode_step_string(args[0]) if len(args) > 0 else "",
            decode_step_string(args[1]) if len(args) > 1 else "",
            ref(args[3]) if len(args) > 3 else None,
            ref(args[4]) if len(args) > 4 else None,
        )

    # -- occurrence -> placement -------------------------------------------
    # PRODUCT_DEFINITION_SHAPE whose represented_product_relation is a NAUO
    pds_for_occ = {}
    for eid, (tname, args) in simple.items():
        if tname != "PRODUCT_DEFINITION_SHAPE" or len(args) < 3:
            continue
        target = ref(args[2])
        if target in occurrences:
            pds_for_occ[eid] = target

    stats = {"placed": 0, "missing": 0, "flipped_side": 0}
    for eid, (tname, args) in simple.items():
        if tname != "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION" or len(args) < 2:
            continue
        rep_rel, pds = ref(args[0]), ref(args[1])
        occ_eid = pds_for_occ.get(pds)
        if occ_eid is None:
            continue
        occ = occurrences[occ_eid]

        rr = entity_args(simple, complex_, rep_rel, "REPRESENTATION_RELATIONSHIP")
        idt_holder = entity_args(simple, complex_, rep_rel,
                                 "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION")
        if not rr or not idt_holder:
            continue
        rep_1 = ref(rr[2]) if len(rr) > 2 else None
        rep_2 = ref(rr[3]) if len(rr) > 3 else None
        idt = ref(idt_holder[0])
        it = entity_args(simple, complex_, idt, "ITEM_DEFINED_TRANSFORMATION")
        if not it or len(it) < 4:
            continue
        item_1, item_2 = ref(it[2]), ref(it[3])

        # Decide which side of the relationship is the component.  item_1 is
        # expressed in rep_1's frame and item_2 in rep_2's frame, so once we
        # know which representation belongs to the child product definition we
        # know which axis placement is the child's.
        child_rep = pd_shape_rep.get(occ.child_pd)
        parent_rep = pd_shape_rep.get(occ.parent_pd)
        if child_rep is not None and child_rep == rep_1:
            child_axis, parent_axis = item_1, item_2
        elif child_rep is not None and child_rep == rep_2:
            child_axis, parent_axis = item_2, item_1
            stats["flipped_side"] += 1
        elif parent_rep is not None and parent_rep == rep_2:
            child_axis, parent_axis = item_1, item_2
        elif parent_rep is not None and parent_rep == rep_1:
            child_axis, parent_axis = item_2, item_1
            stats["flipped_side"] += 1
        else:
            # Fall back on the spec's rep_1 = component convention.
            child_axis, parent_axis = item_1, item_2

        a_child = model.axis2_placement_3d(child_axis)
        a_parent = model.axis2_placement_3d(parent_axis)
        if compose == "child_x_inv_parent":
            occ.transform = matmul4(a_child, invert_rigid4(a_parent))
        else:
            occ.transform = matmul4(a_parent, invert_rigid4(a_child))
        occ.transform_source = {"rep_relationship": rep_rel,
                                "item_defined_transformation": idt,
                                "child_axis": child_axis,
                                "parent_axis": parent_axis}
        stats["placed"] += 1

    stats["missing"] = len(occurrences) - stats["placed"]

    # -- tree --------------------------------------------------------------
    by_parent = {}
    for occ in occurrences.values():
        by_parent.setdefault(occ.parent_pd, []).append(occ)
    child_pds = {occ.child_pd for occ in occurrences.values()}
    roots = sorted(pd for pd in by_parent if pd not in child_pds and pd is not None)

    return {
        "model": model,
        "occurrences": occurrences,
        "by_parent": by_parent,
        "pd_name": pd_name,
        "pd_shape_rep": pd_shape_rep,
        "roots": roots,
        "child_pds": child_pds,
        "stats": stats,
    }


def flatten(asm):
    """Walk the tree, accumulate transforms, and return every leaf occurrence.

    Returns a list of dicts with the leaf's name path, product name,
    root-frame 4x4, and the chain of occurrence entity ids.
    """
    occurrences = asm["occurrences"]
    by_parent = asm["by_parent"]
    pd_name = asm["pd_name"]

    leaves = []
    nodes = []

    def walk(pd, prefix_path, prefix_ids, xf, depth):
        kids = by_parent.get(pd, [])
        for occ in sorted(kids, key=lambda o: o.eid):
            world = matmul4(xf, occ.transform)
            name = pd_name.get(occ.child_pd, "?")
            path = prefix_path + [name]
            ids = prefix_ids + [occ.eid]
            nodes.append({"path": path, "occ_eid": occ.eid, "matrix": world,
                          "depth": depth, "is_leaf": not by_parent.get(occ.child_pd)})
            if by_parent.get(occ.child_pd):
                walk(occ.child_pd, path, ids, world, depth + 1)
            else:
                leaves.append({
                    "path": path,
                    "product_name": name,
                    "occurrence_name": occ.name,
                    "occurrence_id": occ.occ_id,
                    "occ_eid": occ.eid,
                    "occ_chain": ids,
                    "matrix": world,
                })

    for root in asm["roots"]:
        walk(root, [pd_name.get(root, "ROOT")], [], identity4(), 0)
    return leaves, nodes


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------
#
# This particular export (Shapr3D / HOOPS Exchange) writes every
# ITEM_DEFINED_TRANSFORMATION as identity: the assembly is *flattened*, i.e.
# each part's B-rep is already expressed in the assembly root frame.  The
# per-occurrence transforms therefore carry no information and the only way to
# recover where a part sits is to look at its geometry.  The helpers below pull
# out, per leaf part, the global bounding box and the cylindrical bores -- which
# is what actually locates the joint pivots.


class Geometry:
    """Locates and measures the B-rep geometry attached to each part."""

    def __init__(self, model):
        self.model = model
        s = model.simple
        # forward reference index, for transitive reachability
        self.fwd = {}
        for eid, (_t, args) in s.items():
            refs = []
            for arg in args:
                refs.extend(int(x) for x in re.findall(r"#(\d+)", arg))
            self.fwd[eid] = refs

        # PRODUCT_DEFINITION -> its SHAPE_REPRESENTATION
        self.pd_rep = {}
        for eid, (tname, args) in s.items():
            if tname != "SHAPE_DEFINITION_REPRESENTATION" or len(args) < 2:
                continue
            pds, rep = ref(args[0]), ref(args[1])
            pargs = model.args(pds, "PRODUCT_DEFINITION_SHAPE")
            if pargs and len(pargs) > 2 and rep is not None:
                target = ref(pargs[2])
                if model.typeof(target) == "PRODUCT_DEFINITION":
                    self.pd_rep[target] = rep

        # hierarchy SHAPE_REPRESENTATION -> the B-rep that carries its solids.
        # Linked by a plain SHAPE_REPRESENTATION_RELATIONSHIP (no transform).
        self.rep_brep = {}
        for eid, (tname, args) in s.items():
            if tname != "SHAPE_REPRESENTATION_RELATIONSHIP" or len(args) < 4:
                continue
            r1, r2 = ref(args[2]), ref(args[3])
            for a, b in ((r1, r2), (r2, r1)):
                if model.typeof(a) in BREP_TYPES:
                    self.rep_brep[b] = a
        self._cache = {}

    def brep_of(self, pd):
        return self.rep_brep.get(self.pd_rep.get(pd))

    def reach(self, root):
        """Every entity transitively referenced from ``root`` (inclusive)."""
        if root in self._cache:
            return self._cache[root]
        seen = set()
        stack = [root]
        s = self.model.simple
        while stack:
            eid = stack.pop()
            if eid in seen:
                continue
            seen.add(eid)
            if eid in s:
                stack.extend(self.fwd.get(eid, ()))
        self._cache[root] = seen
        return seen

    def measure(self, pd):
        """Return a dict of global-frame measurements for one product definition.

        ``vertex_bbox`` is tight (B-rep topological vertices only) but can miss
        the extreme of a full circular arc, whose extremum is not a vertex.
        ``point_bbox`` covers every CARTESIAN_POINT and so never understates the
        solid, but is inflated by the origins of large-radius surfaces.  Both are
        reported so the caller can see the bracket.
        """
        brep = self.brep_of(pd)
        if brep is None:
            return None
        ids = self.reach(brep)
        s = self.model.simple
        vlo = [1e18] * 3
        vhi = [-1e18] * 3
        plo = [1e18] * 3
        phi = [-1e18] * 3
        nv = 0
        npt = 0
        bores = []
        for eid in ids:
            ent = s.get(eid)
            if ent is None:
                continue
            tname = ent[0]
            if tname == "CARTESIAN_POINT":
                p = self.model.cartesian_point(eid)
                for i in range(3):
                    plo[i] = min(plo[i], p[i])
                    phi[i] = max(phi[i], p[i])
                npt += 1
            elif tname == "VERTEX_POINT":
                p = self.model.cartesian_point(ref(ent[1][1]))
                for i in range(3):
                    vlo[i] = min(vlo[i], p[i])
                    vhi[i] = max(vhi[i], p[i])
                nv += 1
            elif tname == "CYLINDRICAL_SURFACE":
                axis = ref(ent[1][1])
                radius = float(ent[1][2])
                a = self.model.args(axis, "AXIS2_PLACEMENT_3D")
                origin = self.model.cartesian_point(ref(a[1]))
                direction = self.model.direction(ref(a[2]), (0.0, 0.0, 1.0))
                bores.append({"radius": radius, "origin": origin,
                              "axis": direction})
        if npt == 0:
            return None
        out = {"brep_entity": brep, "n_vertices": nv, "n_points": npt,
               "point_bbox": [plo, phi], "bores": bores}
        if nv:
            out["vertex_bbox"] = [vlo, vhi]
            out["centre"] = [(vlo[i] + vhi[i]) / 2.0 for i in range(3)]
        else:
            out["vertex_bbox"] = None
            out["centre"] = [(plo[i] + phi[i]) / 2.0 for i in range(3)]
        return out


BREP_TYPES = ("ADVANCED_BREP_SHAPE_REPRESENTATION",
              "MANIFOLD_SURFACE_SHAPE_REPRESENTATION",
              "FACETED_BREP_SHAPE_REPRESENTATION",
              "GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION")


def attach_geometry(asm, geo, leaves):
    """Attach a ``geometry`` measurement to every leaf that has a B-rep."""
    occ = asm["occurrences"]
    for leaf in leaves:
        pd = occ[leaf["occ_eid"]].child_pd
        leaf["geometry"] = geo.measure(pd)
    return leaves


def _axis_index(axis):
    best = 0
    for i in range(3):
        if abs(axis[i]) > abs(axis[best]):
            best = i
    return best


def find_pivots(leaves, min_radius=4.0, max_radius=25.0, tol=0.02):
    """Group co-axial bores that are shared by two or more distinct parts.

    A hole that appears at the same position and along the same direction in
    more than one part is, in a real assembly, a joint: a shaft, pin or bearing
    passes through it.  This is what recovers the kinematic skeleton from a
    flattened assembly.
    """
    groups = {}
    for leaf in leaves:
        geom = leaf.get("geometry")
        if not geom:
            continue
        for bore in geom["bores"]:
            r = bore["radius"]
            if not (min_radius <= r <= max_radius):
                continue
            axis = bore["axis"]
            ai = _axis_index(axis)
            if abs(abs(axis[ai]) - 1.0) > 1e-6:
                continue        # skip bores not along a principal direction
            perp = [i for i in range(3) if i != ai]
            key = ("XYZ"[ai],
                   round(bore["origin"][perp[0]] / tol) * tol,
                   round(bore["origin"][perp[1]] / tol) * tol)
            g = groups.setdefault(key, {"axis": "XYZ"[ai], "perp": perp,
                                        "pos": [bore["origin"][perp[0]],
                                                bore["origin"][perp[1]]],
                                        "radii": set(), "parts": set()})
            g["radii"].add(round(r, 3))
            g["parts"].add(leaf["product_name"])
    shared = [g for g in groups.values() if len(g["parts"]) >= 2]
    shared.sort(key=lambda g: (-len(g["parts"]), -max(g["radii"])))
    return shared


# Known MT4 kinematics, in millimetres, taken from the arm's firmware.
MT4_EXPECTED = {
    "upper arm  (J2 pivot -> J3 pivot)": 130.0,
    "forearm    (J3 pivot -> wrist)": 150.0,
    "link rod L130 (hole centres)": 130.0,
    "link rod L150 (hole centres)": 150.0,
    "J1 axis -> J2 pivot, horizontal": 45.0,
    "J2 pivot above base bottom": 140.0,
    "wrist -> TCP, horizontal": 35.0,
    "wrist -> TCP, vertical drop": 14.43,
}


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------


def fmt_vec(v, prec=3):
    return "[" + ", ".join("%*.*f" % (9, prec, c) for c in v) + "]"


def _is_identity(m, tol=1e-9):
    for i in range(3):
        for j in range(4):
            want = 1.0 if i == j else 0.0
            if abs(m[i][j] - want) > tol:
                return False
    return True


def summarize(leaves, asm, stream=None):
    if stream is None:
        stream = sys.stdout
        # Part names contain CJK characters; a cp1252 console would raise.
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):
            pass
    w = stream.write
    st = asm["stats"]
    w("STEP assembly summary\n")
    w("=" * 78 + "\n")
    w("occurrences         : %d\n" % len(asm["occurrences"]))
    w("placed (transform)  : %d\n" % st["placed"])
    w("without transform   : %d\n" % st["missing"])
    w("roots               : %s\n" % ", ".join(
        "%s (#%d)" % (asm["pd_name"].get(r, "?"), r) for r in asm["roots"]))
    w("leaf occurrences    : %d\n" % len(leaves))

    n_ident = sum(1 for lf in leaves if _is_identity(lf["matrix"]))
    with_geom = [lf for lf in leaves if lf.get("geometry")]
    w("identity poses       : %d of %d\n" % (n_ident, len(leaves)))
    w("leaves with B-rep    : %d\n" % len(with_geom))
    w("\n")

    if n_ident == len(leaves) and leaves:
        w("!! Every assembly transform in this file is the identity.  The export is\n"
          "!! FLATTENED: each part's B-rep is already written in the assembly root\n"
          "!! frame, so the NEXT_ASSEMBLY_USAGE_OCCURRENCE placements carry no\n"
          "!! information.  Part positions below therefore come from the geometry.\n\n")

    groups = {}
    for leaf in leaves:
        groups.setdefault(leaf["product_name"], []).append(leaf)

    w("Distinct leaf parts (%d).  translation = occurrence pose;\n"
      "centre / bbox = measured B-rep extent in the root frame (mm).\n" % len(groups))
    w("-" * 78 + "\n")
    for name in sorted(groups, key=lambda k: (-len(groups[k]), k)):
        items = groups[name]
        w("%-52s x%-3d\n" % (name[:52], len(items)))
        shown = items if len(items) <= 4 else items[:2]
        for leaf in shown:
            t = [leaf["matrix"][i][3] for i in range(3)]
            w("      T=%s  path=%s\n"
              % (fmt_vec(t), " / ".join(leaf["path"][1:])))
            g = leaf.get("geometry")
            if g:
                lo, hi = g["vertex_bbox"] or g["point_bbox"]
                w("      centre=%s  bbox=%s..%s  bores=%d\n"
                  % (fmt_vec(g["centre"]), fmt_vec(lo, 2), fmt_vec(hi, 2),
                     len(g["bores"])))
        if len(items) > len(shown):
            w("      ... %d more instances\n" % (len(items) - len(shown)))
    w("\n")

    if with_geom:
        xs = [c for lf in with_geom for c in
              ((lf["geometry"]["vertex_bbox"] or lf["geometry"]["point_bbox"])[0][0],
               (lf["geometry"]["vertex_bbox"] or lf["geometry"]["point_bbox"])[1][0])]
        ys = [c for lf in with_geom for c in
              ((lf["geometry"]["vertex_bbox"] or lf["geometry"]["point_bbox"])[0][1],
               (lf["geometry"]["vertex_bbox"] or lf["geometry"]["point_bbox"])[1][1])]
        zs = [c for lf in with_geom for c in
              ((lf["geometry"]["vertex_bbox"] or lf["geometry"]["point_bbox"])[0][2],
               (lf["geometry"]["vertex_bbox"] or lf["geometry"]["point_bbox"])[1][2])]
        w("Overall geometry extent (mm): x[%.2f, %.2f] y[%.2f, %.2f] z[%.2f, %.2f]\n"
          % (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)))
        w("(the vertical axis of this model is +Y)\n\n")

    pivots = find_pivots(leaves)
    if pivots:
        w("Shared bores = joints (a bore at the same place in >=2 parts)\n")
        w("-" * 78 + "\n")
        for g in pivots:
            a = g["axis"]
            pa, pb = "XYZ"[g["perp"][0]], "XYZ"[g["perp"][1]]
            w("  axis %s  %s=%9.3f %s=%9.3f  r=%s\n"
              % (a, pa, g["pos"][0], pb, g["pos"][1],
                 ",".join("%.1f" % r for r in sorted(g["radii"], reverse=True))))
            w("        %s\n" % " | ".join(sorted(p[:30] for p in g["parts"])))
        w("\n")


def to_json_records(leaves):
    out = []
    for leaf in leaves:
        m = leaf["matrix"]
        rec = {
            "path": leaf["path"],
            "product_name": leaf["product_name"],
            "occurrence_id": leaf["occurrence_id"],
            "occurrence_entity": leaf["occ_eid"],
            "translation": [round(m[i][3], 6) for i in range(3)],
            "rotation": [[round(m[i][j], 9) for j in range(3)] for i in range(3)],
            "transform_is_identity": _is_identity(m),
        }
        g = leaf.get("geometry")
        if g:
            lo, hi = g["vertex_bbox"] or g["point_bbox"]
            rec["geometry"] = {
                "brep_entity": g["brep_entity"],
                "n_vertices": g["n_vertices"],
                "bbox_min": [round(v, 4) for v in lo],
                "bbox_max": [round(v, 4) for v in hi],
                "centre": [round(v, 4) for v in g["centre"]],
                "bores": [
                    {"radius": round(b["radius"], 4),
                     "origin": [round(v, 4) for v in b["origin"]],
                     "axis": [round(v, 6) for v in b["axis"]]}
                    for b in g["bores"] if b["radius"] >= 3.0
                ],
            }
        else:
            rec["geometry"] = None
        out.append(rec)
    return out


DEFAULT_STEP = r"Z:\MT4-sim\vendor\MT4-STL\MT4 - For All Users.step"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("step", nargs="?", default=DEFAULT_STEP,
                    help="STEP (AP242) assembly file")
    ap.add_argument("-o", "--out", default=None,
                    help="output JSON path (default: step_assembly.json "
                         "next to this script)")
    ap.add_argument("--compose", default="parent_x_inv_child",
                    choices=["parent_x_inv_child", "child_x_inv_parent"],
                    help="axis composition order (default is the spec order)")
    ap.add_argument("--quiet", action="store_true", help="suppress the summary")
    ap.add_argument("--no-geometry", action="store_true",
                    help="skip B-rep measurement (much faster, but for a "
                         "flattened export it leaves you with no positions)")
    args = ap.parse_args(argv)

    out_path = args.out or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        "step_assembly.json")

    simple, complex_ = parse_step(args.step)
    model = Model(simple, complex_)
    asm = build_assembly(model, compose=args.compose)
    leaves, _nodes = flatten(asm)
    if not args.no_geometry:
        attach_geometry(asm, Geometry(model), leaves)
    else:
        for leaf in leaves:
            leaf["geometry"] = None

    with open(out_path, "w", encoding="utf-8") as fh:
        json.dump(to_json_records(leaves), fh, indent=1, ensure_ascii=False)

    if not args.quiet:
        summarize(leaves, asm)
        print("\nwrote %s (%d leaf records)" % (out_path, len(leaves)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
