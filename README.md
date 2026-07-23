# XPeel Driver

A small Python driver for the **XPeel Automated Plate Seal Remover**, a
device that peels foil/adhesive seals off microplates. It talks to the
instrument over RS-232 using the manufacturer's text-based command/response
protocol.

- Driver: [`xpeel.py`](xpeel.py)
- Tests: [`xpeel_tests.py`](xpeel_tests.py)

- Protocol reference: XPeel User Manual 440140
- command reference p.45–51, unsolicited messages p.51–52, error codes p.56.

## Requirements

```bash
pip install pyserial      # runtime
pip install pytest        # tests only
```

## Quick start

```python
from xpeel import Xpeel, XpeelCommandError

# Open by explicit port, or leave blank to auto-discover by USB VID/PID.
peeler = Xpeel(port="/dev/ttyUSB0")

try:
    peeler.peel()              # default *xpeel:41 — waits until the plate is peeled
    peeler.peel(A=2, B=4)      # slower start, 10 s adhere time
except XpeelCommandError as e:
    print("Peel failed:", e.codes)   # e.g. [4] = seal not removed → retry or abort

print("Seal present:", peeler.check_seal())
print("Tape remaining (supply, take-up):", peeler.tapeleft())

peeler.close()
```

Logging is used for all device communication and error notifications. Enable it with:

```python
import logging
logging.basicConfig(level=logging.INFO)
```

## The protocol, briefly

The port is opened at **9600, 8, N, 1**. Commands are ASCII, terminated with
`<CR><LF>`.

```
host   -> *xpeel:41<CR><LF>
device -> *ack<CR><LF>              # sent immediately
device -> *ready:00,00,00<CR><LF>   # sent when the mechanical action finishes
```

Commands:
    - *peel(A, B)* '*xpeel:AB'.
        'A' = parameter set 1–9 (begin-peel location + speed)
        'B' = adhere time 1–4 (2.5 / 5 / 7.5 / 10 s). Defaults 'A=4, B=1'
    - *check_seal()* '*sealcheck'. 
        The first '*ready' field is '04' if a seal is detected, '00' if not.
    - *tapeleft()* '*tapeleft'. **Not acknowledged**; 
        the device replies with `*tape:SS,TT` then `*ready`. 
        'SS'|'TT' × 10 = approximate peels remaining on the supply spool | capacity on the take-up spool. '99' means "unknown"
        (before the first motion after power-up) and is returned as `None`.
    - *reset() | restart()* — `*reset` | `*restart`. 
        restart` re-emits the power-up sequence, which is drained automatically.

### `*ready` error codes

The `*ready:XX,XX,XX` response carries three 2-digit error-code fields. The
driver parses all three and looks them up in the manual's table (p.56):

| Code | Meaning | Code | Meaning |
|-----:|---------|-----:|---------|
| 00 | No error | 08 | Parameters not saved |
| 01 | Conveyor motor stalled | 09 | Stop button pressed while running |
| 02 | Elevator motor stalled | 10 | Seal sensor unplugged/broken |
| 03 | Take-up spool stalled | 20 | < 30 seals left on supply roll *(warning)* |
| 04 | Seal not removed | 21 | Room for < 30 seals on take-up spool *(warning)* |
| 05 | Illegal command | 51 | Emergency stop (cover open / hardware) |
| 06 | No plate found | 52 | Circuitry fault — remove power |
| 07 | Out of tape / tape broke | | |

Codes **20** and **21** are treated as advisory warnings (logged, non-fatal).
Any other non-zero code is a hard error.

## Design

- **`set_serial_port(port)`** opens an explicit port, or scans `comports()` for
  a matching USB VID/PID. Returns `None` if nothing suitable is found (the
  placeholder VID/PID at the top of `xpeel.py` should be set to your adapter's
  real values).
- **`initialize()`** drains any unsolicited messages sitting on the line at
  connect time (power-up sequence, front-panel activity), so command/response
  operation starts from a clean slate. It stops as soon as the line goes quiet.
- **`handle_message()`** is the single message parser. It classifies a line by
  prefix, decodes `*ready` error fields and `*tape` counts, and **logs any
  non-zero error codes** (warning vs. error). It does not decide whether to
  abort — that policy lives in each command method.
- **`_execute()`** centralizes the send → (optional) ack → ready flow so every
  command shares the same, well-tested control path.

### Error handling

Distinct exceptions (all subclass `XpeelError`) so callers can react precisely:

| Exception | Raised when |
|-----------|-------------|
| `XpeelConnectionError` | Port isn't open, or a serial read/write fails at the OS level. |
| `XpeelTimeoutError` | No response within the timeout — **including the "ack received but no ready" case**. |
| `XpeelResponseError` | A malformed/unexpected message (e.g. non-numeric `*ready` fields, `ready` before `ack`, missing `*tape`). |
| `XpeelCommandError` | The device completed the command but reported a hard error code. `.codes` lists them so the caller can retry with different parameters or abort. |

Malformed input never crashes the driver: bad bytes are decoded with
`errors="replace"` and either surfaced as `XpeelResponseError` or logged and
ignored (for unrecognized/unsolicited lines).

### Timeouts (and why)

All timeouts are constructor arguments so they can be tuned per deployment.

| Timeout | Default | Rationale |
|---------|--------:|-----------|
| `poll_timeout` | 1 s | Per-`readline` blocking window; bounds read-loop responsiveness. |
| `ack_timeout` | 5 s | The device acks "immediately"; 5 s is ample for serial latency. |
| `ready_timeout` | 60 s | A full peel = mechanical motion + up to 10 s adhere; 60 s covers the worst case with margin. Override per call for quick commands. |
| `startup_timeout` | 5 s | Window to drain unsolicited startup messages on connect. |

Reads are paced by `poll_timeout` inside a loop bounded by the operation
deadline, so a slow/absent device is detected without blocking forever.

## Running the tests

```bash
pytest xpeel_tests.py -v
```

The suite (41 tests) uses a `FakeSerial` double and patches `serial.Serial` /
`comports`, so **no hardware is required**. Coverage includes: port discovery,
message parsing (ready/tape/malformed), the ack→ready flow, all timeout and
error paths, and each public command.

## Possible next steps

- Confirm and set the real USB VID/PID for the XPeel's serial adapter.
- Add a retry policy on `peel()` (e.g. re-attempt at a slower parameter set on
  error `04`, seal not removed).
- Implement the remaining documented commands (motor jog commands,
  `*platecheck`, seal-threshold get/set) using the same `_execute()` path.
- Consider an async or thread-based reader if unsolicited messages must be
  handled concurrently with commands.
