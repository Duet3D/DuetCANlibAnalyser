"""
Duet3D CANlib High Level Analyzer for Saleae Logic 2.

This sits on top of a low-level CAN-FD analyzer and turns raw CAN fields into
decoded Duet3D CANlib messages (source/destination/type plus the full payload
field decode generated from the CANlib headers).

Supported input analyzers (selectable in the settings):
  * "CAN FD" by Pierre Molinaro
    (https://github.com/pierremolinaro/canfd-plugin-for-saleae-logic-analyzer)
    Emits per-field frames: "Ext Idf"/"Std Idf", "Ctrl (FDF...)", "D0".."D63",
    "CRC15/17/21", "ACK", "EOF", "IFS", "Error".
  * Saleae's built-in "CAN" analyzer (best effort; CAN-FD support is limited).
    Emits "identifier_field", "control_field", "data_field", "crc_field", ...

Because the low-level analyzer emits one frame per CAN field, this HLA is a
small state machine: it captures the identifier, accumulates the data bytes,
and emits a single decoded "duet" frame when the message ends (at the CRC/EOF).
"""

from saleae.analyzers import HighLevelAnalyzer, AnalyzerFrame

from duet_decoder import DuetDecoder

# Display defaults. (Logic 2 settings are intentionally not declared here; see
# the note in __init__.) Flip these if you want a different default view.
SHOW_RESERVED = False      # include zero/reserved padding fields in the summary
ADDRESS_AS_NAMES = True    # show "tool->main" instead of "121->0"


class Hla(HighLevelAnalyzer):
    result_types = {
        # The bubble/summary shows just the message struct name and the
        # source->destination route. The decoded payload lives in the separate
        # "contents" data field (visible in the Data table / terminal columns).
        "duet": {
            "format": "{{data.name}}  {{data.route}}",
        },
        "duet_error": {"format": "CAN error"},
    }

    def __init__(self):
        # NOTE: we deliberately declare no ChoicesSetting/Setting class members.
        # When settings are declared, Logic's HLA runtime validates them against
        # the settings dict it passes in; in some Logic builds that dict arrives
        # empty and the analyzer fails to load ("Missing setting: ..."). Using
        # module-level constants avoids that entirely. Edit SHOW_RESERVED /
        # ADDRESS_AS_NAMES at the top of this file to change the display.
        self.decoder = DuetDecoder()
        self._reset()
        self._show_reserved = SHOW_RESERVED

    def _reset(self):
        self.can_id = None
        self.payload = bytearray()
        self.is_fd = False
        self.brs = False
        self.start_time = None

    # -- field-stream state machine -----------------------------------------

    def decode(self, frame: AnalyzerFrame):
        ftype = frame.type
        data = frame.data or {}

        # --- identifier: start of a new frame ---
        if ftype in ("Ext Idf", "Std Idf", "identifier_field"):
            self._reset()
            self.start_time = frame.start_time
            self.can_id = self._read_identifier(ftype, data)
            return None

        if self.can_id is None:
            return None  # haven't seen an identifier yet

        # --- control field: note CAN-FD / BRS ---
        if ftype.startswith("Ctrl") or ftype == "control_field":
            self.is_fd = "FDF" in ftype
            self.brs = "BRS" in ftype
            return None

        # --- data bytes ---
        if ftype.startswith("D") and ftype[1:].isdigit():
            self.payload.append(self._byte(data))
            return None
        if ftype == "data_field":
            self.payload.append(self._byte(data, key="data"))
            return None

        # --- end of message: decode and emit ---
        if ftype.startswith("CRC") or ftype in ("crc_field",):
            return self._emit(frame.end_time)
        if ftype in ("EOF", "ACK", "IFS"):
            # Some captures end a frame without our seeing the CRC; emit on EOF
            # if we still hold an undelivered message.
            if self.can_id is not None and ftype == "EOF":
                return self._emit(frame.end_time)
            return None
        if ftype in ("Error", "error"):
            start = self.start_time
            self._reset()
            if start is not None:
                return AnalyzerFrame("duet_error", start, frame.end_time, {})
            return None
        return None

    def _emit(self, end_time):
        if self.can_id is None:
            return None
        try:
            decoded = self.decoder.decode(self.can_id, bytes(self.payload))
            out = AnalyzerFrame("duet", self.start_time, end_time, {
                "name": self.decoder.title(decoded),       # message struct name
                "route": self._route(decoded),             # src->dst
                "contents": self.decoder.contents(decoded, self._show_reserved),
                "type": decoded["type"],
                "dir": decoded["dir"],
                "src": decoded["src"],
                "dst": decoded["dst"],
                "length": decoded["length"],
            })
        except Exception as exc:  # never let one frame kill the stream
            out = AnalyzerFrame("duet", self.start_time, end_time, {
                "name": "decode-error", "route": "",
                "contents": f"{type(exc).__name__}: {exc}",
            })
        self._reset()
        return out

    # -- helpers -------------------------------------------------------------

    def _route(self, decoded):
        if not ADDRESS_AS_NAMES:
            return f"{decoded['src']}->{decoded['dst']}"
        return f"{decoded['src_name']}->{decoded['dst_name']}"

    @staticmethod
    def _read_identifier(ftype, data):
        if ftype == "identifier_field":
            v = data.get("identifier", 0)
            return int(v) & 0x1FFFFFFF
        val = data.get("Value", b"")
        if isinstance(val, (bytes, bytearray)):
            return int.from_bytes(bytes(val), "big") & 0x1FFFFFFF
        return int(val) & 0x1FFFFFFF

    @staticmethod
    def _byte(data, key="Value"):
        v = data.get(key, 0)
        if isinstance(v, (bytes, bytearray)):
            return v[0] if v else 0
        return int(v) & 0xFF
