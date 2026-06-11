#!/usr/bin/env python3
"""
Generate the Duet3D CANlib decode spec consumed by the Saleae High Level Analyzer.

This parses the relevant CANlib C++ headers and emits ``duet_can_spec.json``:

  * the CAN-ID bit layout (from CanId.h)
  * the CanMessageType enum (numeric type id -> name)
  * the packed memory layout of every ``CanMessageXxx`` struct that declares a
    ``static constexpr CanMessageType messageType = ...`` member
  * the generic M-code parameter tables (from CanMessageGenericTables.h) plus a
    heuristic mapping from generic message-type names to those tables

The parser is deliberately tailored to the regular style used throughout
CANlib (GCC ``__attribute__((packed))`` structs, little-endian ARM target).
It is NOT a general C++ parser. After computing layouts it self-validates the
struct sizes against the ``static_assert`` / ``SizeWith...`` constants that the
headers themselves contain, so a layout bug surfaces loudly at generate time.

Run ``python generator/generate_spec.py`` from the project root.
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "CANlib" / "src"
OUT = ROOT / "duet_can_spec.json"

# ---------------------------------------------------------------------------
# Scalar type model. Sizes are for the ARM target CANlib runs on; the layout is
# packed so alignment never matters, only sizes.
# ---------------------------------------------------------------------------

# logical kind, byte size, signed?
SCALARS = {
    "bool": ("u", 1, False),
    "char": ("char", 1, True),
    "int8_t": ("i", 1, True),
    "uint8_t": ("u", 1, False),
    "int16_t": ("i", 2, True),
    "uint16_t": ("u", 2, False),
    "float16_t": ("f16", 2, True),
    "__fp16": ("f16", 2, True),
    "int32_t": ("i", 4, True),
    "uint32_t": ("u", 4, False),
    "int64_t": ("i", 8, True),
    "uint64_t": ("u", 8, False),
    "float": ("f32", 4, True),
    "double": ("f64", 8, True),
    "size_t": ("u", 4, False),
    "unsigned": ("u", 4, False),
    "unsigned int": ("u", 4, False),
    "int": ("i", 4, True),
    "CanAddress": ("u", 1, False),
    "CanRequestId": ("u", 2, False),
}


class ParseError(Exception):
    pass


# ---------------------------------------------------------------------------
# Preprocessing helpers
# ---------------------------------------------------------------------------

def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    text = re.sub(r"//[^\n]*", "", text)
    return text


def eval_int(expr: str, consts: dict) -> int:
    """Safely evaluate a small integer constant expression."""
    expr = expr.strip()
    expr = expr.replace("'", "")  # C++14 digit separators: 48'000'000
    expr = re.sub(r"\b(\d+)[uUlL]+\b", r"\1", expr)  # 1u, 64ul -> 1, 64
    # substitute known identifiers
    def repl(m):
        name = m.group(0)
        if name in consts:
            return str(consts[name])
        raise ParseError(f"unknown identifier in const expr: {name!r}")
    safe = re.sub(r"[A-Za-z_]\w*", repl, expr)
    if not re.fullmatch(r"[\d\s+\-*/()]+", safe):
        raise ParseError(f"unsafe const expr: {expr!r}")
    return int(eval(safe))  # noqa: S307 - input constrained to digits/operators


def collect_constants(texts: list[str]) -> dict:
    consts: dict[str, int] = {}
    # type may be multiple tokens ("unsigned int"); the last word before '=' is
    # the constant name. Values containing '{' (aggregate inits) are skipped.
    pat = re.compile(
        r"(?:static\s+)?constexpr\s+(?:[\w:]+\s+)+(\w+)\s*=\s*([^;{]+);"
    )
    # iterate until fixed point so consts defined via earlier consts resolve
    pending = []
    for text in texts:
        for m in pat.finditer(text):
            pending.append((m.group(1), m.group(2)))
    changed = True
    while changed:
        changed = False
        for name, raw in pending:
            if name in consts:
                continue
            try:
                consts[name] = eval_int(raw, consts)
                changed = True
            except ParseError:
                continue
    return consts


# ---------------------------------------------------------------------------
# Body splitting: split a struct body into top-level statements, keeping inline
# nested struct/union/method braces intact.
# ---------------------------------------------------------------------------

def split_top_level(body: str) -> list[str]:
    """Split a struct body into top-level statements.

    A ';' at depth 0 ends a statement. A '}' that returns to depth 0 also ends
    the statement *if* the statement is a function definition (a '(' appears
    before its first '{'); this handles inline member methods that carry no
    trailing ';'. Type declarations like ``struct {...} name;`` have no '('
    before '{', so they keep accumulating until their ';'.
    """
    stmts, depth, cur = [], 0, []

    def looks_like_function(tokens) -> bool:
        s = "".join(tokens)
        b = s.find("{")
        p = s.find("(")
        return p != -1 and (b == -1 or p < b)

    for ch in body:
        if ch == "{":
            depth += 1
            cur.append(ch)
        elif ch == "}":
            depth -= 1
            cur.append(ch)
            if depth == 0 and looks_like_function(cur):
                stmts.append("".join(cur).strip())
                cur = []
        elif ch == ";" and depth == 0:
            stmts.append("".join(cur).strip())
            cur = []
        else:
            cur.append(ch)
    tail = "".join(cur).strip()
    if tail:
        stmts.append(tail)
    return [s for s in stmts if s]


# Find each top-level ``struct``/``union`` definition and its body. Returns list
# of (kind, name, body, instance_name).
STRUCT_HDR = re.compile(
    r"\b(struct|union)\b"
    r"(?:\s+__attribute__\s*\(\(\s*packed\s*\)\))?"
    r"(?:\s+(\w+))?"
    r"(?:\s+final)?"
    r"\s*\{"
)


def iter_struct_defs(text: str):
    i = 0
    n = len(text)
    while i < n:
        m = STRUCT_HDR.search(text, i)
        if not m:
            break
        kind = m.group(1)
        name = m.group(2)
        # find matching close brace for the '{' at m.end()-1
        depth = 1
        j = m.end()
        while j < n and depth:
            if text[j] == "{":
                depth += 1
            elif text[j] == "}":
                depth -= 1
            j += 1
        body = text[m.end():j - 1]
        # capture optional instance name up to ';'
        k = j
        while k < n and text[k] not in ";{":
            k += 1
        instance = text[j:k].strip()
        instance = instance.replace("final", "").strip()
        yield kind, name, body, instance or None, m.start(), j
        i = j


# ---------------------------------------------------------------------------
# Layout engine
# ---------------------------------------------------------------------------

class Layout:
    """Computes packed bit-level layout for a struct/union body."""

    def __init__(self, registry: dict, consts: dict):
        self.registry = registry          # name -> {"size":int, "fields":[...]}
        self.consts = consts

    def type_size(self, ctype: str) -> int:
        if ctype in SCALARS:
            return SCALARS[ctype][1]
        if ctype in self.registry:
            return self.registry[ctype]["size"]
        raise ParseError(f"unknown type size: {ctype!r}")

    def parse_body(self, body: str, is_union: bool):
        """Return (size_bytes, fields). Field offsets are bit offsets from start."""
        body = re.sub(r"\b(?:public|private|protected)\s*:", " ", body)
        stmts = split_top_level(body)
        nested_types: dict[str, dict] = {}
        fields: list[dict] = []
        bit = 0          # running bit cursor (for struct)
        max_bytes = 0    # for union

        def align_byte():
            nonlocal bit
            if bit % 8:
                bit += 8 - (bit % 8)

        for st in stmts:
            s = st.strip()
            if not s:
                continue

            # inline nested struct/union definition?
            mh = STRUCT_HDR.match(s)
            if mh:
                # parse the single nested def contained in this statement
                for nk, nname, nbody, ninst, _, _ in iter_struct_defs(s):
                    sub_size, sub_fields = self.parse_body(nbody, nk == "union")
                    sub = {"size": sub_size, "fields": sub_fields}
                    if nname:
                        nested_types[nname] = sub
                        self.registry.setdefault(nname, sub)
                    if ninst:
                        # an instance member of the nested type
                        self._place_struct_member(
                            fields, ninst, sub, bit if not is_union else 0)
                        if is_union:
                            max_bytes = max(max_bytes, sub_size)
                        else:
                            align_byte()
                            bit += sub_size * 8
                    elif not nname:
                        # anonymous union/struct used directly as a member
                        align_byte()
                        self._place_anonymous(fields, sub, bit, nk == "union")
                        if is_union:
                            max_bytes = max(max_bytes, sub_size)
                        else:
                            bit += sub_size * 8
                    break
                continue

            # skip methods / constructors / static members / typedefs / using
            if self._is_non_field(s):
                continue

            # field declaration(s)
            parsed = self._parse_field_stmt(s)
            if parsed is None:
                continue
            base_type, signed_override, members = parsed
            for mem in members:
                if mem["bitwidth"] is not None:
                    # bitfield member
                    if is_union:
                        # bitfields in a union start at 0
                        f = self._make_bitfield(base_type, mem, 0)
                        fields.append(f)
                        max_bytes = max(max_bytes, (mem["bitwidth"] + 7) // 8)
                    else:
                        f = self._make_bitfield(base_type, mem, bit)
                        fields.append(f)
                        bit += mem["bitwidth"]
                else:
                    # byte-aligned scalar / array / nested-struct member
                    if is_union:
                        self._place_named(fields, base_type, mem, 0)
                        max_bytes = max(max_bytes, self._member_bytes(base_type, mem))
                    else:
                        align_byte()
                        self._place_named(fields, base_type, mem, bit)
                        bit += self._member_bytes(base_type, mem) * 8

        if is_union:
            size = max_bytes
        else:
            align_byte()
            size = bit // 8
        return size, fields

    # -- field statement parsing --------------------------------------------

    def _is_non_field(self, s: str) -> bool:
        if s.startswith(("static", "typedef", "using", "friend", "template")):
            return True
        # a constructor or method: has '(' at top level before any '['
        # but not a function-pointer member (none in CANlib structs)
        depth = 0
        for ch in s:
            if ch == "(":
                return True
            if ch in "[]":
                break
        # bare declarations like "void Foo" w/o parens won't appear
        return False

    def _parse_field_stmt(self, s: str):
        """Parse 'TYPE a, b[3], c:12, d:4' -> (base_type, signed, [members])."""
        s = re.sub(r"\s+", " ", s).strip()
        # leading type: greedily match known multi-word/templated/qualified type
        m = re.match(r"((?:unsigned\s+int|unsigned|[\w:]+))\s+(.*)", s)
        if not m:
            return None
        base_type = m.group(1).strip()
        rest = m.group(2).strip()
        if base_type not in SCALARS and base_type not in self.registry:
            # unknown type (e.g. a forward-declared/templated thing we skip)
            raise ParseError(f"unknown field type {base_type!r} in: {s!r}")
        members = []
        for decl in self._split_commas(rest):
            decl = decl.strip()
            if not decl:
                continue
            mb = re.match(r"(\w+)\s*:\s*(\d+)$", decl)        # bitfield
            if mb:
                members.append({"name": mb.group(1), "array": None,
                                "bitwidth": int(mb.group(2))})
                continue
            ma = re.match(r"(\w+)\s*\[([^\]]+)\]$", decl)      # array
            if ma:
                count = eval_int(ma.group(2), self.consts)
                members.append({"name": ma.group(1), "array": count,
                                "bitwidth": None})
                continue
            ms = re.match(r"(\w+)$", decl)                     # plain scalar
            if ms:
                members.append({"name": ms.group(1), "array": None,
                                "bitwidth": None})
                continue
            raise ParseError(f"cannot parse member decl: {decl!r}")
        return base_type, None, members

    @staticmethod
    def _split_commas(rest: str) -> list[str]:
        out, depth, cur = [], 0, []
        for ch in rest:
            if ch in "[(":
                depth += 1
            elif ch in "])":
                depth -= 1
            if ch == "," and depth == 0:
                out.append("".join(cur))
                cur = []
            else:
                cur.append(ch)
        out.append("".join(cur))
        return out

    # -- placement helpers ---------------------------------------------------

    def _member_bytes(self, base_type: str, mem: dict) -> int:
        sz = self.type_size(base_type)
        if mem["array"] is not None:
            return sz * mem["array"]
        return sz

    def _make_bitfield(self, base_type: str, mem: dict, bit: int) -> dict:
        signed = SCALARS.get(base_type, ("u", 0, False))[2]
        return {
            "name": mem["name"], "kind": "bitfield",
            "bit_offset": bit, "bit_width": mem["bitwidth"],
            "signed": bool(signed),
        }

    def _place_named(self, fields, base_type, mem, bit):
        byte = bit // 8
        if base_type in SCALARS:
            kind, size, signed = SCALARS[base_type]
            if base_type == "char" and mem["array"] is not None:
                fields.append({"name": mem["name"], "kind": "string",
                               "byte_offset": byte, "max_len": mem["array"]})
                return
            if mem["array"] is not None:
                fields.append({"name": mem["name"], "kind": "array",
                               "byte_offset": byte, "scalar": kind,
                               "elem_bytes": size, "count": mem["array"]})
                return
            fields.append({"name": mem["name"], "kind": "scalar",
                           "byte_offset": byte, "scalar": kind,
                           "bytes": size, "signed": signed})
            return
        # nested struct/union type
        sub = self.registry[base_type]
        if mem["array"] is not None:
            fields.append({"name": mem["name"], "kind": "array_struct",
                           "byte_offset": byte, "elem_bytes": sub["size"],
                           "count": mem["array"], "type": base_type,
                           "fields": sub["fields"]})
        else:
            fields.append({"name": mem["name"], "kind": "struct",
                           "byte_offset": byte, "bytes": sub["size"],
                           "type": base_type, "fields": sub["fields"]})

    def _place_struct_member(self, fields, inst, sub, bit):
        byte = (bit + 7) // 8
        fields.append({"name": inst, "kind": "struct", "byte_offset": byte,
                       "bytes": sub["size"], "type": None,
                       "fields": sub["fields"]})

    def _place_anonymous(self, fields, sub, bit, is_union):
        byte = bit // 8
        # surface the anonymous union/struct's members at this offset
        for f in sub["fields"]:
            g = dict(f)
            if "byte_offset" in g:
                g["byte_offset"] = byte + g.get("byte_offset", 0)
            if "bit_offset" in g:
                g["bit_offset"] = bit + g.get("bit_offset", 0)
            fields.append(g)


# ---------------------------------------------------------------------------
# Top-level driver
# ---------------------------------------------------------------------------

def parse_enum_message_types(text: str) -> dict:
    m = re.search(r"enum\s+class\s+CanMessageType\s*:\s*\w+\s*\{(.*?)\}", text, re.S)
    if not m:
        raise ParseError("CanMessageType enum not found")
    body = m.group(1)
    result: dict[int, str] = {}
    cur = 0
    for line in body.split(","):
        line = line.strip()
        if not line:
            continue
        mm = re.match(r"(\w+)\s*=\s*(0x[0-9A-Fa-f]+|\d+)", line)
        if mm:
            name = mm.group(1)
            val = int(mm.group(2), 0)
            cur = val
        else:
            mm = re.match(r"(\w+)$", line)
            if not mm:
                continue
            name = mm.group(1)
            val = cur
        cur += 1
        if name == "unusedMessageType":
            continue
        result[val] = name
    return result


def parse_param_tables(text: str) -> dict:
    """Parse 'constexpr ParamDescriptor XxxParams[] = { MACRO('c'), ... };'."""
    tables: dict[str, list] = {}
    for m in re.finditer(
        r"constexpr\s+ParamDescriptor\s+(\w+)\s*\[\]\s*=\s*\{(.*?)\}\s*;",
        text, re.S,
    ):
        name, body = m.group(1), m.group(2)
        params = []
        for entry in re.finditer(r"(\w+)\s*\(\s*'((?:\\.|[^']))'\s*(?:,\s*([^)]+))?\)",
                                 body):
            macro, letter, extra = entry.group(1), entry.group(2), entry.group(3)
            if macro == "END_PARAMS":
                continue
            params.append({"macro": macro, "letter": letter,
                           "arg": extra.strip() if extra else None})
        if params:
            tables[name] = params
    return tables


def map_types_to_tables(generic_types: dict, tables: dict) -> dict:
    """Heuristically link a generic message-type name to a param table name."""
    def norm(s: str) -> str:
        s = s.lower()
        s = s.replace("point", "p").replace("params", "")
        s = re.sub(r"[^a-z0-9]", "", s)
        return s
    table_norm = {norm(t): t for t in tables}
    out = {}
    for name in generic_types.values():
        key = norm(name)
        if key in table_norm:
            out[name] = table_norm[key]
    return out


def main() -> int:
    if not SRC.exists():
        print(f"ERROR: CANlib source not found at {SRC}.", file=sys.stderr)
        print("Run: git submodule update --init  (or clone CANlib there)",
              file=sys.stderr)
        return 2

    files = ["CanId.h", "Duet3Common.h", "RRF3Common.h", "CanSettings.h",
             "HeaterModel.h", "RemoteInputHandle.h", "CanMessageFormats.h"]
    raw = {f: strip_comments((SRC / f).read_text(encoding="utf-8", errors="replace"))
           for f in files}

    consts = collect_constants(list(raw.values()))

    message_types = parse_enum_message_types(raw["CanId.h"])

    # registry of all struct/union layouts, resolved with retry passes
    registry: dict[str, dict] = {}
    layout = Layout(registry, consts)

    # gather every top-level struct/union across the dependency headers
    defs = []
    for f in ["RRF3Common.h", "CanSettings.h", "HeaterModel.h",
              "RemoteInputHandle.h", "CanMessageFormats.h"]:
        for kind, name, body, instance, _, _ in iter_struct_defs(raw[f]):
            if name:
                defs.append((kind, name, body))

    struct_msgtype: dict[str, int] = {}   # struct name -> numeric message type
    name_to_type = {v: k for k, v in message_types.items()}

    pending = list(defs)
    for _ in range(12):
        progress = False
        still = []
        for kind, name, body in pending:
            if name in registry:
                continue
            try:
                size, fields = layout.parse_body(body, kind == "union")
            except ParseError:
                still.append((kind, name, body))
                continue
            registry[name] = {"size": size, "fields": fields}
            progress = True
            mt = re.search(
                r"static\s+constexpr\s+CanMessageType\s+messageType\s*=\s*"
                r"CanMessageType::(\w+)\s*;", body)
            if mt and mt.group(1) in name_to_type:
                struct_msgtype[name] = name_to_type[mt.group(1)]
        pending = still
        if not progress:
            break

    if pending:
        names = ", ".join(n for _, n, _ in pending)
        print(f"WARNING: could not parse layouts for: {names}", file=sys.stderr)

    # build per-message-type struct spec
    structs_by_type = {}
    for sname, tid in struct_msgtype.items():
        entry = registry[sname]
        structs_by_type[str(tid)] = {
            "name": sname, "size": entry["size"], "fields": entry["fields"],
        }

    typed = set(struct_msgtype.values())
    generic_types = {k: v for k, v in message_types.items() if k not in typed}

    tables = parse_param_tables(raw.get("CanMessageGenericTables.h", "") or
                                strip_comments((SRC / "CanMessageGenericTables.h")
                                               .read_text(errors="replace")))
    type_to_table = map_types_to_tables(generic_types, tables)

    spec = {
        "_comment": "AUTO-GENERATED by generator/generate_spec.py. Do not edit.",
        "canId": {
            "messageTypeShift": 16, "messageTypeMask": 0x1FFF,
            "responseBit": 1 << 15, "srcAddressShift": 8,
            "dstAddressShift": 0, "boardAddressMask": 0x7F,
            "requestIdMask": 0x07FF,
        },
        "addresses": {
            "0": "main", "121": "tool", "122": "EXP1XD", "123": "EXP1HCL",
            "124": "SammyC21", "125": "ATE-main", "126": "FW-update",
            "127": "broadcast",
        },
        "messageTypes": {str(k): v for k, v in sorted(message_types.items())},
        "structsByType": structs_by_type,
        "genericTypes": {str(k): v for k, v in sorted(generic_types.items())},
        "paramTables": tables,
        "typeNameToParamTable": type_to_table,
        "generic": {
            "header": {"requestIdBits": 12, "paramMapBits": 20,
                       "dataOffset": 4, "dataMax": 60},
        },
    }

    OUT.write_text(json.dumps(spec, indent=1), encoding="utf-8")
    print(f"Wrote {OUT.relative_to(ROOT)}")
    print(f"  message types:        {len(message_types)}")
    print(f"  fully-decoded structs:{len(structs_by_type)}")
    print(f"  generic-format types: {len(generic_types)} "
          f"({len(type_to_table)} mapped to param tables)")

    validate(registry, consts)
    return 0


# ---------------------------------------------------------------------------
# Self-validation against the headers' own size assertions / constants
# ---------------------------------------------------------------------------

def validate(registry: dict, consts: dict) -> None:
    checks = [
        # (struct, expected size) -- from explicit constants/static_asserts
        ("CanMessageTimeSync", 20),     # SizeWithRealTimeAndMovementDelay
        ("CanTiming", 10),
        ("HeaterModel", 40),
        ("RemoteInputHandle", 2),
        ("MinCurMax", 12),
        ("ShortMinCurMax", 6),
        ("CanMessageFirmwareUpdateRequest", 64),
        ("CanMessageFirmwareUpdateResponse", 64),
        ("CanMessageStandardReply", 64),
        ("CanMessageRevertPosition", 4 + 4 + 4 * consts["MaxLinearDriversPerCanSlave"]),
        ("CanSensorReport", 5),
        ("CanHeaterReport", 6),
        ("ClosedLoopStatus", 12),
        ("OpenLoopStatus", 4),
    ]
    failures = []
    for name, expected in checks:
        got = registry.get(name, {}).get("size")
        if got != expected:
            failures.append(f"  {name}: expected {expected}, got {got}")
    if failures:
        print("LAYOUT VALIDATION FAILED:", file=sys.stderr)
        print("\n".join(failures), file=sys.stderr)
        raise SystemExit(1)
    print("Layout self-validation passed "
          f"({len(checks)} size assertions).")


if __name__ == "__main__":
    raise SystemExit(main())
