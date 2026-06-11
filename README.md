# Duet3D CANlib — Saleae Logic High Level Analyzer

Decodes [Duet3D CANlib](https://github.com/Duet3D/CANlib) CAN-FD messages in
Saleae Logic 2: source/destination board, message type, request/response, and
the **full payload field decode** for every message that has a dedicated struct,
plus parameter decoding for the generic M-code messages.

The decode tables are **generated directly from the CANlib C++ headers**, so the
analyzer tracks CANlib as it changes — add or change a message in CANlib, run one
command (or let CI do it), and the analyzer updates. See
[Keeping it up to date](#keeping-it-up-to-date).

---

## How it works (two layers)

Saleae captures logic-level edges; it does not understand CAN by itself. Two
layers turn edges into decoded Duet messages:

1. **Low-level CAN-FD analyzer** — decodes the wire into CAN fields (identifier,
   control, data bytes, CRC). Use either:
   - **Pierre Molinaro's "CAN FD" plugin** (recommended; full CAN-FD + BRS):
     <https://github.com/pierremolinaro/canfd-plugin-for-saleae-logic-analyzer>
   - Saleae's **built-in "CAN"** analyzer (works for classic CAN; CAN-FD support
     is limited). Both are supported by this HLA.
2. **This High Level Analyzer** — sits on top of (1), reassembles the fields into
   a whole message, and decodes the Duet CANlib semantics.

```
   wire ──▶ [CAN-FD low-level analyzer] ──fields──▶ [Duet3D CANlib HLA] ──▶ decoded message
```

---

## ⚠️ Probe wiring — read this first

CAN is a **differential** bus. Do **not** decode by putting analyzer ground on
CAN‑L and signal on CAN‑H:

- Referenced that way the signal is only ~2 V and **inverted** relative to a
  normal logic-level CAN signal, so the low-level analyzer will struggle.
- Worse, tying analyzer ground (PC-referenced) to CAN‑L — which floats 1.5–2.5 V
  above system ground — can disturb or short the bus.

**Correct tap:** probe the **RX (or TX) pin of the CAN transceiver** on the
board, with analyzer **ground on the board's system GND**. That gives a clean,
correctly-polarised 0–3.3 V copy of the bus. If you can only reach the
differential pair, put a CAN transceiver breakout on it and probe its RX line.

---

## Install

1. Build/install a low-level CAN-FD analyzer (Molinaro plugin or built-in CAN).
2. In Logic 2: **Extensions ▸ ⋯ ▸ Load Existing Extension…** and select this
   folder's [`extension.json`](extension.json).
3. Add the low-level CAN analyzer to your capture and configure its bit rates
   (e.g. 1 Mbit/s nominal; Duet uses BRS for the data phase).
4. Add the **"Duet3D CANlib"** analyzer and set its **Input Analyzer** to the
   CAN analyzer from step 3.

### Settings

| Setting | Options | Meaning |
|---|---|---|
| Show reserved/zero fields | No / Yes | Include `zero*` padding fields in the summary |
| Address display | Names / Numbers | Show `tool→main` vs `121→0` |

---

## What it decodes

- **CAN identifier** (29-bit extended): message type → name, request/response,
  source and destination addresses (with friendly names like `main`, `tool`,
  `broadcast`).
- **Full payload** for every CANlib message with a dedicated struct
  (~53 messages: time sync, movement, heater/fan/GPIO control, status & sensor
  reports, accelerometer/closed-loop data, firmware update, standard reply, …),
  including packed bitfields, nested structs (e.g. `HeaterModel`), and
  variable-length report arrays.
- **Generic M-code messages** (`M569`, `M950…`, `M915`, …): `requestId`, the
  `paramMap`, and the individual parameters where the message name maps to a
  parameter table in CANlib.
- Anything unmapped still gets the full identifier decode plus raw payload hex,
  so **every** message on the bus is identified.

---

## Keeping it up to date

The decode table [`duet_can_spec.json`](duet_can_spec.json) is generated from the
CANlib headers in [`CANlib/`](CANlib) by
[`generator/generate_spec.py`](generator/generate_spec.py). The generator
**self-validates** the computed struct layouts against the `static_assert`s and
size constants in the headers, so a layout error fails loudly.

```bash
# regenerate from the current CANlib checkout
python regenerate.py

# pull the latest CANlib first, then regenerate + test
python regenerate.py --update
```

`CANlib/` is intended to be a git submodule pinned to a CANlib branch/tag (e.g.
`3.7-docker`). The included GitHub Action
([`.github/workflows/regenerate.yml`](.github/workflows/regenerate.yml)) updates
the submodule weekly, regenerates the spec, runs the tests, and opens a pull
request if anything changed.

To pin a different CANlib version, check out that ref in `CANlib/` and rerun
`python regenerate.py`.

---

## Project layout

| Path | Purpose |
|---|---|
| `extension.json` | Saleae extension manifest |
| `HighLevelAnalyzer.py` | The HLA: reassembles CAN fields, emits decoded frames |
| `duet_decoder.py` | Pure, dependency-free decoder (id + payload → fields) |
| `duet_can_spec.json` | **Generated** decode tables (committed) |
| `generator/generate_spec.py` | Parses CANlib headers → `duet_can_spec.json` |
| `tests/test_decoder.py` | Decoder unit tests over hand-built payloads |
| `regenerate.py` | One command: (update) → generate → test |
| `CANlib/` | CANlib source (submodule) the spec is generated from |

---

## Limitations

- **CAN-FD coverage depends on the low-level analyzer.** Use the Molinaro plugin
  for reliable CAN-FD/BRS; the built-in CAN analyzer is best-effort.
- **Variable-length messages** are decoded using the bytes actually captured.
  CAN-FD rounds message length up to the next frame size with zero padding, so
  trailing report-array elements may appear as zero-valued entries.
- **Templated messages** (`CanMessageMultipleDrivesRequest<T>`: `setMotorCurrents`,
  `setStepsPerMmAndMicrostepping`, `setPressureAdvance`, …) are not modelled as
  fixed structs; they currently fall back to identifier + `requestId` + raw hex.
- **Generic message → parameter-table** mapping is heuristic by name. CANlib
  defines the tables but the type→table binding lives in RRF, so a few generic
  types show `paramMap` + raw data instead of named parameters.
