"""Python driver for the Azenta XPeel Automated Plate Seal Remover.

The XPeel communicates over RS-232 (9600 baud, 8 data bits, no parity, 1 stop
bit) using a simple text-based command/response protocol. Commands are ASCII
strings terminated by ``<CR><LF>``. Most commands are answered first with an
``*ack`` and then, once the mechanical action completes, with a
``*ready:XX,XX,XX`` message whose three fields carry error codes.

The device may also emit *unsolicited* messages (power-up sequence, front-panel
activity). Those are drained on connect and ignored while waiting for a
command's response.

Protocol reference: XPeel User Manual (Azenta P/N 440140), command reference
p.45-51, unsolicited messages p.51-52, error codes p.56.
"""

import logging
import time
from collections import namedtuple
from enum import Enum

import serial
import serial.tools.list_ports

logger = logging.getLogger(__name__)

# Line framing shared by every command.
CR = "\r"
LF = "\n"
TERMINATOR = CR + LF

# USB-serial adapter identifiers used to auto-discover the device. These are
# placeholders for the CH340-style adapter commonly shipped with the XPeel;
# replace with the real VID/PID for your hardware.
DEFAULT_VID = 0x1A86
DEFAULT_PID = 0x7523

# Command error codes, from the manual's "Command Error Codes" table (p.56).
ERROR_CODES = {
    0: "No error",
    1: "Conveyor motor stalled",
    2: "Elevator motor stalled",
    3: "Take-up spool stalled",
    4: "Seal not removed",
    5: "Illegal command",
    6: "No plate found",
    7: "Out of tape, or tape broke",
    8: "Parameters not saved",
    9: "Stop button pressed while running",
    10: "Seal sensor unplugged or broken",
    20: "Less than 30 seals left on the supply roll",
    21: "Room for less than 30 seals on take-up spool",
    51: "Emergency stop: power relay not settable (cover open or hardware problem)",
    52: "Circuitry fault detected: remove power",
}

# Non-zero codes that are advisory (low tape) rather than hard failures. These
# are logged as warnings but do not cause a command to raise.
WARNING_CODES = {20, 21}

# Value the device reports for tape counts when they are not yet known
# (before the first motion after power-up).
TAPE_UNKNOWN = 99


class Command(Enum):
    """Outgoing command verbs."""

    PEEL = "*xpeel:"
    SEALCHECK = "*sealcheck"
    TAPELEFT = "*tapeleft"
    RESET = "*reset"
    RESTART = "*restart"


# Parsed representation of a single message received from the device.
#   kind:  'ack' | 'ready' | 'tape' | 'poweron' | 'homing' | 'unsolicited'
#          | 'unknown' | 'empty'
#   codes: list[int] error codes for 'ready', else None
#   tape:  (supply, takeup) tuple for 'tape', else None
#   text:  the raw decoded/stripped message
Message = namedtuple("Message", ["kind", "codes", "tape", "text"])


# --------------------------------------------------------------------------- #
# Exceptions
# --------------------------------------------------------------------------- #
class XpeelError(Exception):
    """Base class for all XPeel driver errors."""


class XpeelConnectionError(XpeelError):
    """The serial port is not open or a read/write failed at the OS level."""


class XpeelTimeoutError(XpeelError):
    """Expected a message from the device but none arrived within the timeout.

    Also raised when an ``*ack`` is received but the subsequent ``*ready`` never
    arrives within the operation timeout.
    """


class XpeelResponseError(XpeelError):
    """A message was received but was malformed or unexpected for the context."""


class XpeelCommandError(XpeelError):
    """The device completed a command but reported one or more error codes."""

    def __init__(self, codes: list[int]) -> None:
        self.codes = codes
        described = ", ".join(f"{c:02d} ({ERROR_CODES.get(c, 'Unknown error')})" for c in codes)
        super().__init__(f"Device reported error code(s): {described}")


class Xpeel:
    """Driver for a single XPeel Automated Plate Seal Remover.

    Args:
        port: Serial device path (e.g. ``/dev/ttyUSB0``). If empty, the driver
            scans available ports for a matching VID/PID.
        id: Caller-supplied identifier, useful when several peelers are managed.
        poll_timeout: Per-``readline`` blocking timeout in seconds. Bounds how
            long any single read waits for bytes.
        ack_timeout: Max seconds to wait for the ``*ack`` after sending a
            command. The device acks "immediately", so this is generous.
        ready_timeout: Max seconds to wait for the ``*ready`` response. A full
            peel involves mechanical motion plus up to 10 s of adhere time, so
            the default is deliberately large; override per command if needed.
        startup_timeout: Max seconds spent draining unsolicited messages on
            connect (see :meth:`initialize`).
    """

    def __init__(
        self,
        port: str = "",
        id: int = 0,
        poll_timeout: float = 1.0,
        ack_timeout: float = 5.0,
        ready_timeout: float = 60.0,
        startup_timeout: float = 5.0,
    ) -> None:
        self.id = id
        self.poll_timeout = poll_timeout
        self.ack_timeout = ack_timeout
        self.ready_timeout = ready_timeout
        self.startup_timeout = startup_timeout
        self.serial: serial.Serial | None = self.set_serial_port(port)
        self.initialize()

    # ------------------------------------------------------------------ #
    # Connection setup
    # ------------------------------------------------------------------ #
    def set_serial_port(self, port: str = "") -> serial.Serial | None:
        """Open the serial port at 9600,8,N,1.

        If ``port`` is given, that device is opened directly. Otherwise the
        available ports are scanned and the first one whose USB VID/PID matches
        :data:`DEFAULT_VID`/:data:`DEFAULT_PID` is opened.

        Returns:
            An open ``serial.Serial`` instance, or ``None`` if the port could
            not be opened / no matching device was found.
        """
        settings = {
            "baudrate": 9600,
            "bytesize": serial.EIGHTBITS,
            "parity": serial.PARITY_NONE,
            "stopbits": serial.STOPBITS_ONE,
            "timeout": self.poll_timeout,
        }

        if port:
            try:
                return serial.Serial(port, **settings)
            except serial.SerialException as e:
                logger.error("Could not open port %s: %s", port, e)
                return None

        for port_info in serial.tools.list_ports.comports():
            if port_info.vid == DEFAULT_VID and port_info.pid == DEFAULT_PID:
                try:
                    return serial.Serial(port_info.device, **settings)
                except serial.SerialException as e:
                    logger.error("Could not open discovered port %s: %s", port_info.device, e)
                    return None

        logger.warning("No XPeel device found (VID=%#06x, PID=%#06x).", DEFAULT_VID, DEFAULT_PID)
        return None

    def initialize(self) -> None:
        """Drain any unsolicited messages present on the line before use.

        On power-up the XPeel emits ``*poweron``/``*homing``/``*ready``; front
        panel use produces other unsolicited messages. We consume everything
        currently queued and stop as soon as the line goes quiet (a ``readline``
        returns empty) or ``startup_timeout`` elapses, so the driver starts from
        a clean state regardless of what the device was doing when we connected.
        """
        if self.serial is None:
            return

        deadline = time.monotonic() + self.startup_timeout
        while time.monotonic() < deadline:
            try:
                raw = self.serial.readline()
            except serial.SerialException as e:
                logger.error("Error draining serial port on init: %s", e)
                return
            if not raw:
                return  # line is quiet, nothing else pending
            self.handle_message(raw)

    # ------------------------------------------------------------------ #
    # Low-level I/O
    # ------------------------------------------------------------------ #
    def _write(self, command_str: str) -> None:
        """Encode and write a fully-framed command string to the device."""
        if self.serial is None:
            raise XpeelConnectionError("Serial port is not open.")
        try:
            self.serial.write(command_str.encode("ascii"))
        except serial.SerialException as e:
            raise XpeelConnectionError(f"Failed to send command {command_str!r}: {e}") from e

    def _read_line(self, deadline: float) -> str:
        """Read one line, retrying until data arrives or ``deadline`` passes.

        ``readline`` blocks up to ``poll_timeout`` and returns ``b""`` on
        timeout; we loop so the effective wait is bounded by ``deadline`` rather
        than a single poll interval.

        Raises:
            XpeelTimeoutError: if no complete line arrives before ``deadline``.
            XpeelConnectionError: if the serial layer raises.
        """
        if self.serial is None:
            raise XpeelConnectionError("Serial port is not open.")
        while True:
            try:
                raw = self.serial.readline()
            except serial.SerialException as e:
                raise XpeelConnectionError(f"Serial read failed: {e}") from e
            if raw:
                return raw.decode("ascii", errors="replace").strip()
            if time.monotonic() >= deadline:
                raise XpeelTimeoutError("Timed out waiting for a response from XPeel.")

    # ------------------------------------------------------------------ #
    # Message parsing / handling
    # ------------------------------------------------------------------ #
    def handle_message(self, raw: bytes | str, command: Command | None = None) -> Message:
        """Parse a single device message into a :class:`Message`.

        This is the central message handler. It classifies the message by its
        prefix and, for a ``*ready`` response, decodes the three error-code
        fields and logs any non-zero codes (warnings vs. errors). It never
        raises for device error codes — callers decide whether an error should
        abort the operation — but it does raise :class:`XpeelResponseError` if a
        ``*ready``/``*tape`` payload is structurally malformed.

        Args:
            raw: Raw bytes (or an already-decoded string) from the device.
            command: The command awaiting a response, used for context. For
                :attr:`Command.SEALCHECK` the first ready field encodes seal
                presence (04) rather than an error, so error logging is skipped.
        """
        text = raw.decode("ascii", errors="replace").strip() if isinstance(raw, (bytes, bytearray)) else raw.strip()
        if not text:
            return Message("empty", None, None, text)

        logger.info("XPeel -> %s", text)

        if text.startswith("*ready"):
            codes = self._parse_ready(text)
            if command is not Command.SEALCHECK:
                self._notify_errors(codes)
            return Message("ready", codes, None, text)
        if text.startswith("*ack"):
            return Message("ack", None, None, text)
        if text.startswith("*tape"):
            return Message("tape", None, self._parse_tape(text), text)
        if text.startswith("*poweron"):
            return Message("poweron", None, None, text)
        if text.startswith("*homing"):
            return Message("homing", None, None, text)
        if text.startswith(("*manual", "*xpeel", "*setup", "*seal")):
            return Message("unsolicited", None, None, text)

        logger.warning("Unrecognized message from XPeel: %r", text)
        return Message("unknown", None, None, text)

    def _parse_ready(self, text: str) -> list[int]:
        """Parse ``*ready:XX,XX,XX`` into a list of three integer error codes.

        Raises:
            XpeelResponseError: if the payload is missing or not three integers.
        """
        try:
            payload = text.split(":", 1)[1]
            codes = [int(field) for field in payload.split(",")]
        except (IndexError, ValueError) as e:
            raise XpeelResponseError(f"Malformed ready response: {text!r}") from e
        if len(codes) != 3:
            raise XpeelResponseError(f"Expected 3 error fields in ready response, got {text!r}")
        return codes

    def _parse_tape(self, text: str) -> tuple[int | None, int | None]:
        """Parse ``*tape:SS,TT`` into ``(supply_deseals, takeup_deseals)``.

        Each field times 10 is the number of "deseals" (supply remaining /
        take-up capacity). A raw value of 99 means "unknown" and is returned as
        ``None`` for that field.

        Raises:
            XpeelResponseError: if the payload is malformed.
        """
        try:
            payload = text.split(":", 1)[1]
            ss, tt = (int(field) for field in payload.split(","))
        except (IndexError, ValueError) as e:
            raise XpeelResponseError(f"Malformed tape response: {text!r}") from e
        supply = None if ss == TAPE_UNKNOWN else ss * 10
        takeup = None if tt == TAPE_UNKNOWN else tt * 10
        return supply, takeup

    def _notify_errors(self, codes: list[int]) -> None:
        """Log any non-zero error codes, distinguishing warnings from errors."""
        for code in codes:
            if code == 0:
                continue
            description = ERROR_CODES.get(code, "Unknown error")
            if code in WARNING_CODES:
                logger.warning("XPeel warning %02d: %s", code, description)
            else:
                logger.error("XPeel error %02d: %s", code, description)

    def _raise_for_errors(self, codes: list[int]) -> None:
        """Raise :class:`XpeelCommandError` if any hard (non-warning) error is set."""
        hard = [c for c in codes if c != 0 and c not in WARNING_CODES]
        if hard:
            raise XpeelCommandError(hard)

    # ------------------------------------------------------------------ #
    # Command execution flow
    # ------------------------------------------------------------------ #
    def _await_ack(self) -> None:
        """Wait for the ``*ack`` that follows a command, ignoring noise.

        Raises:
            XpeelTimeoutError: if no ack arrives within ``ack_timeout``.
            XpeelResponseError: if a ``*ready`` arrives before the ack.
        """
        deadline = time.monotonic() + self.ack_timeout
        while True:
            msg = self.handle_message(self._read_line(deadline))
            if msg.kind == "ack":
                return
            if msg.kind == "ready":
                raise XpeelResponseError(f"Received ready before ack: {msg.text!r}")
            # Ignore unsolicited/unknown messages and keep waiting.

    def _await_ready(
        self, timeout: float, command: Command | None = None
    ) -> tuple[list[int], dict]:
        """Wait for the ``*ready`` response, collecting any interim messages.

        Args:
            timeout: Max seconds to wait for the ready message.
            command: Forwarded to :meth:`handle_message` for context.

        Returns:
            ``(codes, collected)`` where ``codes`` are the three ready error
            codes and ``collected`` maps interim message kinds (e.g. ``"tape"``)
            to their parsed payloads.

        Raises:
            XpeelTimeoutError: if no ready arrives within ``timeout`` (this is
                also the "ack but never ready" case).
        """
        deadline = time.monotonic() + timeout
        collected: dict = {}
        while True:
            msg = self.handle_message(self._read_line(deadline), command)
            if msg.kind == "ready":
                return msg.codes, collected
            if msg.kind == "tape":
                collected["tape"] = msg.tape
            # Ignore unsolicited/unknown messages and keep waiting.

    def _execute(
        self,
        command_str: str,
        *,
        expect_ack: bool = True,
        ready_timeout: float | None = None,
        command: Command | None = None,
    ) -> tuple[list[int], dict]:
        """Send a command and return its parsed ``*ready`` result.

        Handles framing, the optional ack step, and waiting for ready. Does not
        interpret the error codes — callers apply their own policy.
        """
        if self.serial is None:
            raise XpeelConnectionError("Serial port is not open.")
        self._write(command_str + TERMINATOR)
        if expect_ack:
            self._await_ack()
        return self._await_ready(ready_timeout or self.ready_timeout, command)

    # ------------------------------------------------------------------ #
    # Public commands
    # ------------------------------------------------------------------ #
    def peel(self, A: int = 4, B: int = 1) -> list[int]:
        """Run a peel cycle and return once the plate has been peeled.

        Sends ``*xpeel:AB``, waits for the ack, then blocks until the ``*ready``
        response confirms the cycle finished.

        Args:
            A: Parameter set 1-9 (begin-peel location + speed). Default 4.
            B: Adhere time 1-4 (2.5/5/7.5/10 s). Default 1. Defaults produce the
                manual's documented ``*xpeel:41``.

        Returns:
            The three ready error codes (all zero on a clean peel).

        Raises:
            ValueError: if A or B are out of range.
            XpeelCommandError: if the device reports a hard error (e.g. 04,
                seal not removed). Callers can inspect ``.codes`` to decide
                whether to retry with different parameters or abort.
            XpeelTimeoutError / XpeelResponseError / XpeelConnectionError: on
                communication problems.
        """
        if not 1 <= A <= 9:
            raise ValueError(f"Parameter set A must be 1-9, got {A}")
        if not 1 <= B <= 4:
            raise ValueError(f"Adhere time B must be 1-4, got {B}")

        codes, _ = self._execute(
            f"{Command.PEEL.value}{A}{B}", command=Command.PEEL
        )
        self._raise_for_errors(codes)
        return codes

    def check_seal(self) -> bool:
        """Check whether a seal is present on the plate (``*sealcheck``).

        Returns:
            ``True`` if a seal is detected (first ready field == 04), else
            ``False``.

        Raises:
            XpeelTimeoutError / XpeelResponseError / XpeelConnectionError: on
                communication problems.
        """
        codes, _ = self._execute(Command.SEALCHECK.value, command=Command.SEALCHECK)
        return codes[0] == 4

    def tapeleft(self) -> tuple[int | None, int | None]:
        """Query remaining tape (``*tapeleft``).

        This command is not acknowledged; the device replies directly with
        ``*tape:SS,TT`` followed by ``*ready``.

        Returns:
            ``(supply_deseals, takeup_deseals)`` — approximate number of peels
            remaining on the supply spool and capacity remaining on the take-up
            spool. Either value is ``None`` if the device reports it as unknown
            (before the first motion after power-up).

        Raises:
            XpeelResponseError: if no ``*tape`` message is received.
            XpeelTimeoutError / XpeelConnectionError: on communication problems.
        """
        _, collected = self._execute(
            Command.TAPELEFT.value, expect_ack=False, command=Command.TAPELEFT
        )
        if "tape" not in collected:
            raise XpeelResponseError("No *tape message received in response to tapeleft.")
        return collected["tape"]

    def reset(self) -> list[int]:
        """Advance to fresh tape and re-home the axes (``*reset``).

        Returns:
            The three ready error codes.

        Raises:
            XpeelCommandError: on a hard device error.
            XpeelTimeoutError / XpeelResponseError / XpeelConnectionError: on
                communication problems.
        """
        codes, _ = self._execute(Command.RESET.value, command=Command.RESET)
        self._raise_for_errors(codes)
        return codes

    def restart(self) -> None:
        """Restart the device (``*restart``), equivalent to a power cycle.

        After the ack the device re-emits the power-up sequence
        (``*poweron``/``*homing``/``*ready``), which is drained via
        :meth:`initialize`.

        Raises:
            XpeelTimeoutError / XpeelResponseError / XpeelConnectionError: on
                communication problems.
        """
        self._write(Command.RESTART.value + TERMINATOR)
        self._await_ack()
        self.initialize()

    def close(self) -> None:
        """Close the underlying serial port if it is open."""
        if self.serial is not None:
            self.serial.close()
            self.serial = None
