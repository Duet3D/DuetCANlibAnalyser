"""Unit tests for duet_decoder against hand-built packed payloads.

Run: python -m pytest tests/  (or: python tests/test_decoder.py)
"""
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from duet_decoder import DuetDecoder  # noqa: E402

D = DuetDecoder()


def make_id(type_id, src, dst, response=False):
    v = (type_id << 16) | (src << 8) | dst
    if response:
        v |= 1 << 15
    return v


def test_id_decode():
    info = D.decode_id(make_id(30, 0, 127))
    assert info["type"] == "timeSync"
    assert info["dir"] == "req"
    assert info["src"] == 0 and info["dst"] == 127
    assert info["dst_name"] == "broadcast"

    info = D.decode_id(make_id(4510, 121, 0, response=True))
    assert info["type"] == "standardReply"
    assert info["dir"] == "resp"
    assert info["src"] == 121 and info["src_name"] == "tool"


def test_timesync_payload():
    bf = (0x0123) | (1 << 16) | (2 << 17) | (13 << 20)  # ack=0x123,print=1,rate=2,tseg=13
    payload = struct.pack("<IIIII", 0x11111111, 0x22222222, bf, 0x33333333, 0x44444444)
    out = D.decode(make_id(30, 0, 127), payload)
    f = out["fields"]
    assert out["format"] == "CanMessageTimeSync"
    assert f["timeSent"] == 0x11111111
    assert f["lastTimeSent"] == 0x22222222
    assert f["lastTimeAcknowledgeDelay"] == 0x0123
    assert f["isPrinting"] == 1
    assert f["fastDataRate"] == 2
    assert f["tseg1Minus1"] == 13
    assert f["realTime"] == 0x33333333
    assert f["movementDelay"] == 0x44444444


def test_standard_reply():
    # requestId:12, resultCode:4, fragmentNumber:7, moreFollows:1, extra:8 ; then text
    header = 0x123 | (0 << 12) | (0 << 16) | (0 << 23) | (0 << 24)
    text = b"ok done\x00"
    payload = struct.pack("<I", header) + text
    out = D.decode(make_id(4510, 121, 0, response=True), payload)
    f = out["fields"]
    assert f["requestId"] == 0x123
    assert f["resultCode"] == 0
    assert f["text"] == "ok done"


def test_heaters_status_array():
    # uint64 whichHeaters, then CanHeaterReport[ ] (mode u8, averagePwm u8, temp f32)
    whichHeaters = 0b101
    r0 = struct.pack("<BBf", 3, 128, 215.5)
    r1 = struct.pack("<BBf", 1, 64, 60.0)
    payload = struct.pack("<Q", whichHeaters) + r0 + r1
    out = D.decode(make_id(4515, 121, 0, response=True), payload)
    f = out["fields"]
    assert f["whichHeaters"] == 0b101
    assert len(f["reports"]) == 2
    assert f["reports"][0]["mode"] == 3
    assert abs(f["reports"][0]["temperature"] - 215.5) < 1e-3
    assert abs(f["reports"][1]["temperature"] - 60.0) < 1e-3


def test_nested_heater_model():
    # CanMessageHeaterModelV3: 4-byte bitfield header, then HeaterModel(40), maxPwm, ...
    # requestId:12 + zero:4 fill the first uint16; heater:8 starts the second.
    header = 0x111 | (5 << 16)  # requestId=0x111, heater=5
    model = struct.pack("<9fI", 2.43, 0.56, 0.0, 1.35, 5.5, 0.0, 220.0, 0.0, 0.0, 1)
    payload = struct.pack("<I", header) + model + struct.pack("<ffff", 1.0, 0, 0, 0)
    out = D.decode(make_id(6069, 0, 121), payload)
    f = out["fields"]
    assert f["requestId"] == 0x111
    assert f["heater"] == 5
    assert abs(f["basicModel"]["heatingRate"] - 2.43) < 1e-3
    assert abs(f["basicModel"]["deadTime"] - 5.5) < 1e-3
    assert abs(f["maxPwm"] - 1.0) < 1e-3


def test_generic_m569():
    # M569Params order: P(localDriver u8)=0, S(u8)=1, R(int8)=2, D(u8)=3, ...
    # Set P (bit0) and S (bit1) and D (bit3).
    param_map = (1 << 0) | (1 << 1) | (1 << 3)
    header = 0x222 | (param_map << 12)
    payload = struct.pack("<I", header) + bytes([2, 1, 4])  # P=2, S=1, D=4
    out = D.decode(make_id(6018, 0, 121), payload)
    f = out["fields"]
    assert out["format"] == "generic"
    assert f["requestId"] == 0x222
    assert f["params"]["P"] == 2
    assert f["params"]["S"] == 1
    assert f["params"]["D"] == 4


def test_summary_runs():
    payload = struct.pack("<IIIII", 1, 2, 0, 3, 4)
    out = D.decode(make_id(30, 0, 127), payload)
    s = D.summary(out)
    assert "timeSync" in s and "→broadcast" in s


def test_title_and_contents_split():
    payload = struct.pack("<IIIII", 7, 8, 0, 9, 10)
    out = D.decode(make_id(30, 0, 127), payload)
    # title is the struct name; route/addresses are NOT in the contents field
    assert D.title(out) == "CanMessageTimeSync"
    contents = D.contents(out)
    assert "timeSent=7" in contents
    assert "broadcast" not in contents and "timeSync" not in contents

    # generic/raw messages fall back to the type name for the title
    gen = D.decode(make_id(6018, 0, 121), struct.pack("<I", 0))
    assert D.title(gen) == "m569"


# CanMessageStandardReply as CANlib declares it from 3.7 onwards, with the
# annotations generate_spec.py adds for GetText(). Built here rather than read
# from the committed spec so the test holds whichever CANlib is pinned.
_STANDARD_REPLY_V37 = {
    "name": "CanMessageStandardReply", "size": 64,
    "fields": [
        {"name": "requestId", "kind": "bitfield", "bit_offset": 0,
         "bit_width": 12, "signed": False},
        {"name": "resultCode", "kind": "bitfield", "bit_offset": 12,
         "bit_width": 4, "signed": False},
        {"name": "fragmentNumber", "kind": "bitfield", "bit_offset": 16,
         "bit_width": 5, "signed": False},
        {"name": "numWords", "kind": "bitfield", "bit_offset": 21,
         "bit_width": 2, "signed": False},
        {"name": "moreFollows", "kind": "bitfield", "bit_offset": 23,
         "bit_width": 1, "signed": False},
        {"name": "extra", "kind": "bitfield", "bit_offset": 24,
         "bit_width": 8, "signed": False},
        {"name": "dataWords", "kind": "array", "byte_offset": 4,
         "scalar": "u", "elem_bytes": 4, "signed": False,
         "count_from": {"field": "numWords"}},
        {"name": "text", "kind": "string", "byte_offset": 4, "max_len": 60,
         "offset_from": {"field": "numWords", "scale": 4},
         "max_len_from": {"field": "numWords", "scale": 4}},
    ],
}


def _decoder_with_v37_reply():
    spec = dict(D.spec)
    spec["structsByType"] = dict(spec["structsByType"])
    spec["structsByType"]["4510"] = _STANDARD_REPLY_V37
    return DuetDecoder(spec)


def _reply_payload(num_words, words, text):
    header = 0x123 | (0 << 12) | (0 << 16) | (num_words << 21)
    return (struct.pack("<I", header)
            + b"".join(struct.pack("<I", w) for w in words) + text)


def test_standard_reply_data_words():
    # From 3.7 fragment 0 may carry up to three 32-bit words ahead of the text;
    # decoding the text from a fixed offset would return the first word's bytes.
    d = _decoder_with_v37_reply()
    rid = make_id(4510, 121, 0, response=True)

    f = d.decode(rid, _reply_payload(0, [], b"ok done\x00"))["fields"]
    assert f["text"] == "ok done"
    assert "dataWords" not in f          # nothing to report when numWords == 0

    f = d.decode(rid, _reply_payload(1, [1234], b"tare done\x00"))["fields"]
    assert f["numWords"] == 1
    assert f["dataWords"] == [1234]
    assert f["text"] == "tare done"

    f = d.decode(rid, _reply_payload(3, [1, 2, 0xFFFFFFFF], b"three\x00"))["fields"]
    assert f["dataWords"] == [1, 2, 0xFFFFFFFF]
    assert f["text"] == "three"


def test_standard_reply_text_is_shortened_by_data_words():
    # GetMaxTextLength() is sizeof(text) - numWords * 4, so unterminated text
    # must stop at the end of the message rather than run past it.
    d = _decoder_with_v37_reply()
    rid = make_id(4510, 121, 0, response=True)
    f = d.decode(rid, _reply_payload(1, [7], b"x" * 60))["fields"]
    assert f["text"] == "x" * 56


def test_param_table_override():
    # setConnectionTimeout carries M959 parameters, which the name heuristic
    # cannot see; m970 still resolves by name alone.
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "generator"))
    from generate_spec import map_types_to_tables

    mapped = map_types_to_tables(
        {6071: "setConnectionTimeout", 6072: "m970", 6015: "setDateTime"},
        {"M959Params": [], "M970Params": []})
    assert mapped["setConnectionTimeout"] == "M959Params"
    assert mapped["m970"] == "M970Params"
    assert "setDateTime" not in mapped          # no table exists for it


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"PASS {fn.__name__}")
    print(f"\nAll {len(fns)} tests passed.")


if __name__ == "__main__":
    _run_all()
