# EX-BUF1 — single Schmitt-trigger buffer

**Stand-in datasheet, revision A (2026-10-01). An example, not a part.**

No EX-BUF1 exists. This page stands in for the vendor PDF a real part's
model cites, so that the model in `../src/buffer.rs` can cite a section for
every figure it uses, the way embsim's own models cite their parts'
datasheets (`DESIGN.md` rules 6 and 9; `BOARD_ENGINE.md`, "Model provenance
convention"). Every figure below was chosen for the example and appears
here once. A project's real model cites its real part's datasheet, by
document number, revision, section and page.

## 1. Pin functions

| Pin | Name | Type | Function |
|---|---|---|---|
| 1 | A | input | the buffer's input |
| 2 | GND | ground | the reference of every voltage below |
| 3 | Y | output | push-pull; Y = A |
| 4 | VCC | supply | |

## 2. Recommended operating conditions

| Parameter | Min | Max | Unit |
|---|---|---|---|
| VCC, supply voltage | 3.0 | 3.6 | V |

Outside this range the device's behaviour is not specified.

## 3. Electrical characteristics, VCC = 3.0 V to 3.6 V

| Parameter | Conditions | Min | Max | Unit |
|---|---|---|---|---|
| V_T+, positive-going input threshold | | | 2.0 | V |
| V_T−, negative-going input threshold | | 0.9 | | V |
| V_OH, high-level output voltage | I_OH = −8 mA | VCC − 0.4 | | V |
| V_OL, low-level output voltage | I_OL = 8 mA | | 0.32 | V |

Between V_T− and V_T+ the input keeps the level it last read (Schmitt
action).

## 4. Switching characteristics, VCC = 3.3 V ± 0.3 V, C_L = 15 pF

| Parameter | From | To | Max | Unit |
|---|---|---|---|---|
| t_pd, propagation delay | A | Y | 12 | ns |

## 5. Application notes

An input must be driven or tied to VCC or GND. The output of an input left
open is not specified.
