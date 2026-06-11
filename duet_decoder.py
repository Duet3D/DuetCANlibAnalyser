"""
Pure-Python decoder for Duet3D CANlib CAN-FD messages.

Given a 29-bit CAN identifier and the message payload bytes, this turns them
into a structured decode using ``duet_can_spec.json`` (produced by
``generator/generate_spec.py`` from the CANlib headers).

This module has no third-party dependencies and does not import the Saleae SDK,
so it can be unit-tested on its own (see ``tests/test_decoder.py``). The Saleae
High Level Analyzer in ``HighLevelAnalyzer.py`` reassembles CAN fields into
``(can_id, payload)`` and calls :class:`DuetDecoder` here.
"""

from __future__ import annotations

import json
import struct
from pathlib import Path

SPEC_PATH = Path(__file__).resolve().parent / "duet_can_spec.json"

# Parameter element sizes for the generic M-code parameter macros.
_PARAM_MACRO = {
    "UINT64_PARAM": ("u", 8), "UINT32_PARAM": ("u", 4),
    "UINT16_PARAM": ("u", 2), "UINT8_PARAM": ("u", 1),
    "INT32_PARAM": ("i", 4), "INT16_PARAM": ("i", 2), "INT8_PARAM": ("i", 1),
    "FLOAT_PARAM": ("f32", 4), "FLOAT16_PARAM": ("f16", 2),
    "PWM_FREQ_PARAM": ("u", 2), "CHAR_PARAM": ("char", 1),
    "LOCAL_DRIVER_PARAM": ("u", 1),
    "STRING_PARAM": ("string", 0), "REDUCED_STRING_PARAM": ("string", 0),
    "UINT8_ARRAY_PARAM": ("u", 1), "UINT16_ARRAY_PARAM": ("u", 2),
    "UINT32_ARRAY_PARAM": ("u", 4), "FLOAT_ARRAY_PARAM": ("f32", 4),
}
_ARRAY_MACROS = {"UINT8_ARRAY_PARAM", "UINT16_ARRAY_PARAM",
                 "UINT32_ARRAY_PARAM", "FLOAT_ARRAY_PARAM"}


def _decode_scalar(kind: str, raw: bytes):
    if kind == "f32":
        return struct.unpack("<f", raw)[0]
    if kind == "f64":
        return struct.unpack("<d", raw)[0]
    if kind == "f16":
        return struct.unpack("<e", raw)[0]
    if kind == "i":
        return int.from_bytes(raw, "little", signed=True)
    return int.from_bytes(raw, "little", signed=False)  # 'u'


def _extract_bits(payload: bytes, bit_offset: int, bit_width: int, signed: bool):
    """Extract a packed little-endian bitfield (bit 0 == LSB of byte 0)."""
    end = bit_offset + bit_width
    nbytes = (end + 7) // 8
    if nbytes > len(payload):
        return None
    acc = int.from_bytes(payload[:nbytes], "little")
    val = (acc >> bit_offset) & ((1 << bit_width) - 1)
    if signed and (val & (1 << (bit_width - 1))):
        val -= 1 << bit_width
    return val


class DuetDecoder:
    def __init__(self, spec: dict | None = None):
        if spec is None:
            spec = json.loads(SPEC_PATH.read_text(encoding="utf-8"))
        self.spec = spec
        self.canid = spec["canId"]
        self.addresses = spec["addresses"]
        self.message_types = spec["messageTypes"]
        self.structs_by_type = spec["structsByType"]
        self.generic_types = spec["genericTypes"]
        self.param_tables = spec["paramTables"]
        self.type_to_table = spec["typeNameToParamTable"]

    # -- CAN identifier ------------------------------------------------------

    def decode_id(self, can_id: int) -> dict:
        c = self.canid
        type_id = (can_id >> c["messageTypeShift"]) & c["messageTypeMask"]
        is_response = bool(can_id & c["responseBit"])
        src = (can_id >> c["srcAddressShift"]) & c["boardAddressMask"]
        dst = (can_id >> c["dstAddressShift"]) & c["boardAddressMask"]
        name = self.message_types.get(str(type_id), f"type#{type_id}")
        return {
            "type_id": type_id, "type": name,
            "dir": "resp" if is_response else "req",
            "src": src, "dst": dst,
            "src_name": self.addresses.get(str(src), str(src)),
            "dst_name": self.addresses.get(str(dst), str(dst)),
        }

    def addr(self, a: int) -> str:
        return self.addresses.get(str(a), str(a))

    # -- payload -------------------------------------------------------------

    def decode(self, can_id: int, payload: bytes) -> dict:
        info = self.decode_id(can_id)
        type_id = info["type_id"]
        payload = bytes(payload)
        info["length"] = len(payload)

        struct_spec = self.structs_by_type.get(str(type_id))
        if struct_spec is not None:
            info["format"] = struct_spec["name"]
            info["fields"] = self._decode_fields(struct_spec["fields"], payload)
        elif str(type_id) in self.generic_types:
            info["format"] = "generic"
            info["fields"] = self._decode_generic(info["type"], payload)
        else:
            info["format"] = "raw"
            info["fields"] = {"data": payload.hex()}
        return info

    def _decode_fields(self, fields: list, payload: bytes, base: int = 0) -> dict:
        out = {}
        for f in fields:
            val = self._decode_one(f, payload, base)
            if val is not None:
                out[f["name"]] = val
        return out

    def _decode_one(self, f: dict, payload: bytes, base: int):
        kind = f["kind"]
        if kind == "bitfield":
            return _extract_bits(payload, base * 8 + f["bit_offset"],
                                 f["bit_width"], f.get("signed", False))
        off = base + f["byte_offset"]
        if kind == "scalar":
            raw = payload[off:off + f["bytes"]]
            if len(raw) < f["bytes"]:
                return None
            return _decode_scalar(f["scalar"], raw)
        if kind == "string":
            raw = payload[off:off + f["max_len"]]
            return raw.split(b"\x00", 1)[0].decode("latin-1", "replace")
        if kind == "array":
            n = min(f["count"], max(0, (len(payload) - off) // f["elem_bytes"]))
            vals = []
            for i in range(n):
                p = off + i * f["elem_bytes"]
                vals.append(_decode_scalar(f["scalar"], payload[p:p + f["elem_bytes"]]))
            return vals if vals else None
        if kind == "struct":
            if off + f["bytes"] > len(payload):
                # decode what we can (variable-length tail)
                pass
            return self._decode_fields(f["fields"], payload, off)
        if kind == "array_struct":
            n = min(f["count"], max(0, (len(payload) - off) // f["elem_bytes"]))
            elems = []
            for i in range(n):
                elems.append(self._decode_fields(
                    f["fields"], payload, off + i * f["elem_bytes"]))
            return elems if elems else None
        return None

    # -- generic M-code messages --------------------------------------------

    def _decode_generic(self, type_name: str, payload: bytes) -> dict:
        if len(payload) < 4:
            return {"data": payload.hex()}
        header = int.from_bytes(payload[:4], "little")
        request_id = header & 0x0FFF
        param_map = (header >> 12) & 0xFFFFF
        out = {"requestId": request_id, "paramMap": f"0x{param_map:05X}"}

        table_name = self.type_to_table.get(type_name)
        if not table_name:
            out["data"] = payload[4:].hex()
            return out

        table = self.param_tables[table_name]
        params = {}
        pos = 4
        for i, p in enumerate(table):
            if not (param_map & (1 << i)):
                continue
            macro = p["macro"]
            kind, size = _PARAM_MACRO.get(macro, ("u", 1))
            letter = p["letter"]
            if kind == "string":
                raw = payload[pos:].split(b"\x00", 1)[0]
                params[letter] = raw.decode("latin-1", "replace")
                pos += len(raw) + 1
            elif macro in _ARRAY_MACROS:
                if pos >= len(payload):
                    break
                count = payload[pos]
                pos += 1
                vals = []
                for _ in range(count):
                    vals.append(_decode_scalar(kind, payload[pos:pos + size]))
                    pos += size
                params[letter] = vals
            else:
                raw = payload[pos:pos + size]
                if len(raw) < size:
                    break
                params[letter] = (raw.decode("latin-1") if kind == "char"
                                  else _decode_scalar(kind, raw))
                pos += size
        out["params"] = params
        out["_table"] = table_name
        return out

    # -- presentation --------------------------------------------------------

    def contents(self, decoded: dict, show_reserved: bool = False) -> str:
        """Format just the decoded payload fields as 'key=val  key=val ...'."""
        parts = []
        for k, v in decoded.get("fields", {}).items():
            if not show_reserved and (k.startswith("zero") or k == "_table"):
                continue
            parts.append(f"{k}={self._fmt(v)}")
        return "  ".join(parts)

    def title(self, decoded: dict) -> str:
        """The message's struct name, or the type name for generic/raw messages."""
        fmt = decoded.get("format")
        if fmt and fmt not in ("generic", "raw"):
            return fmt
        return decoded["type"]

    def summary(self, decoded: dict, show_reserved: bool = False) -> str:
        head = (f"{decoded['type']} {decoded['src_name']}→{decoded['dst_name']} "
                f"[{decoded['dir']}]")
        body = self.contents(decoded, show_reserved)
        return f"{head} | {body}" if body else head

    def _fmt(self, v):
        if isinstance(v, float):
            return f"{v:.4g}"
        if isinstance(v, dict):
            return "{" + ",".join(f"{k}:{self._fmt(x)}" for k, x in v.items()) + "}"
        if isinstance(v, list):
            if len(v) > 6:
                return f"[{len(v)} items]"
            return "[" + ",".join(self._fmt(x) for x in v) + "]"
        return str(v)
