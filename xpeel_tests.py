"""Unit tests for the XPeel serial driver (xpeel.py).

The driver is exercised without hardware by substituting a lightweight
``FakeSerial`` double for pyserial's ``Serial`` object, and by patching
``serial.Serial`` / ``comports`` where port discovery is tested.

Timeouts on the instances under test are set very small so the timeout paths
resolve quickly.

Run with:  pytest xpeel_tests.py -v
"""

import time

import serial
from unittest.mock import MagicMock, patch

import pytest

from xpeel import (
    Command,
    ERROR_CODES,
    Xpeel,
    XpeelCommandError,
    XpeelConnectionError,
    XpeelResponseError,
    XpeelTimeoutError,
)


# --------------------------------------------------------------------------- #
# Test doubles / helpers
# --------------------------------------------------------------------------- #
class FakeSerial:
    """Minimal stand-in for serial.Serial.

    ``readline`` pops queued byte lines and returns b"" once exhausted (which
    mimics a pyserial read timeout). ``write`` records what was sent.
    """

    def __init__(self, lines=None, readline_error=False):
        self._lines = list(lines or [])
        self._readline_error = readline_error
        self.written = []
        self.closed = False

    def readline(self):
        if self._readline_error:
            raise serial.SerialException("boom")
        return self._lines.pop(0) if self._lines else b""

    def write(self, data):
        self.written.append(data)
        return len(data)

    def close(self):
        self.closed = True


def make_xpeel(fake=None, ack_timeout=0.05, ready_timeout=0.05, startup_timeout=0.05):
    """Build an Xpeel without running __init__ (which opens a port)."""
    obj = Xpeel.__new__(Xpeel)
    obj.id = 1
    obj.poll_timeout = 0.001
    obj.ack_timeout = ack_timeout
    obj.ready_timeout = ready_timeout
    obj.startup_timeout = startup_timeout
    obj.serial = fake
    return obj


# --------------------------------------------------------------------------- #
# Protocol constants
# --------------------------------------------------------------------------- #
class TestProtocolConstants:
    def test_command_values(self):
        assert Command.PEEL.value == "*xpeel:"
        assert Command.SEALCHECK.value == "*sealcheck"  # note the leading '*'
        assert Command.TAPELEFT.value == "*tapeleft"

    def test_error_code_table(self):
        assert ERROR_CODES[0] == "No error"
        assert ERROR_CODES[4] == "Seal not removed"
        assert ERROR_CODES[5] == "Illegal command"


# --------------------------------------------------------------------------- #
# set_serial_port
# --------------------------------------------------------------------------- #
class TestSetSerialPort:
    def test_opens_explicit_port_with_9600_8n1(self):
        obj = make_xpeel()
        with patch("serial.Serial") as mock_serial:
            result = obj.set_serial_port(port="/dev/ttyUSB0")

        assert result is mock_serial.return_value
        mock_serial.assert_called_once_with(
            "/dev/ttyUSB0",
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=obj.poll_timeout,
        )

    def test_explicit_port_failure_returns_none(self):
        obj = make_xpeel()
        with patch("serial.Serial", side_effect=serial.SerialException):
            assert obj.set_serial_port(port="/dev/ttyUSB0") is None

    def test_autodiscovers_port_by_vid_pid(self):
        obj = make_xpeel()
        non_match = MagicMock(device="/dev/tty.other", vid=0x1234, pid=0x5678)
        match = MagicMock(device="/dev/tty.xpeel", vid=0x1A86, pid=0x7523)

        with patch("serial.tools.list_ports.comports", return_value=[non_match, match]), \
             patch("serial.Serial") as mock_serial:
            result = obj.set_serial_port()

        assert result is mock_serial.return_value
        assert mock_serial.call_args[0][0] == "/dev/tty.xpeel"

    def test_no_matching_device_returns_none(self):
        obj = make_xpeel()
        non_match = MagicMock(device="/dev/tty.other", vid=0x1234, pid=0x5678)
        with patch("serial.tools.list_ports.comports", return_value=[non_match]), \
             patch("serial.Serial") as mock_serial:
            assert obj.set_serial_port() is None
        mock_serial.assert_not_called()


# --------------------------------------------------------------------------- #
# handle_message / parsing
# --------------------------------------------------------------------------- #
class TestHandleMessage:
    def test_ack(self):
        assert make_xpeel().handle_message(b"*ack\r\n").kind == "ack"

    def test_ready_parses_three_error_fields(self):
        msg = make_xpeel().handle_message(b"*ready:00,01,02\r\n")
        assert msg.kind == "ready"
        assert msg.codes == [0, 1, 2]

    def test_ready_malformed_payload_raises(self):
        with pytest.raises(XpeelResponseError):
            make_xpeel().handle_message(b"*ready:oops\r\n")

    def test_ready_wrong_field_count_raises(self):
        with pytest.raises(XpeelResponseError):
            make_xpeel().handle_message(b"*ready:00,00\r\n")

    def test_tape_parsed_and_scaled_by_ten(self):
        msg = make_xpeel().handle_message(b"*tape:11,13\r\n")
        assert msg.kind == "tape"
        assert msg.tape == (110, 130)

    def test_tape_unknown_value_maps_to_none(self):
        msg = make_xpeel().handle_message(b"*tape:99,99\r\n")
        assert msg.tape == (None, None)

    def test_poweron_and_homing(self):
        obj = make_xpeel()
        assert obj.handle_message(b"*poweron\r\n").kind == "poweron"
        assert obj.handle_message(b"*homing\r\n").kind == "homing"

    def test_unrecognized_message_is_unknown(self):
        assert make_xpeel().handle_message(b"garbage\r\n").kind == "unknown"

    def test_ready_errors_are_logged(self, caplog):
        with caplog.at_level("WARNING"):
            make_xpeel().handle_message(b"*ready:04,00,00\r\n")
        assert any("Seal not removed" in r.message for r in caplog.records)

    def test_sealcheck_context_suppresses_error_logging(self, caplog):
        # For sealcheck, 04 means "seal detected", not an error.
        with caplog.at_level("ERROR"):
            make_xpeel().handle_message(b"*ready:04,00,00\r\n", command=Command.SEALCHECK)
        assert not any(r.levelname == "ERROR" for r in caplog.records)


# --------------------------------------------------------------------------- #
# initialize (drain unsolicited messages)
# --------------------------------------------------------------------------- #
class TestInitialize:
    def test_returns_when_no_serial(self):
        assert make_xpeel(fake=None).initialize() is None

    def test_drains_pending_messages_until_quiet(self):
        fake = FakeSerial(lines=[b"*poweron\r\n", b"*homing\r\n", b"*ready:00,00,00\r\n"])
        obj = make_xpeel(fake)
        obj.initialize()
        assert fake.readline() == b""  # everything consumed

    def test_read_error_is_handled_gracefully(self):
        obj = make_xpeel(FakeSerial(readline_error=True))
        assert obj.initialize() is None  # does not raise


# --------------------------------------------------------------------------- #
# _read_line timeout
# --------------------------------------------------------------------------- #
class TestReadLineTimeout:
    def test_times_out_when_no_data(self):
        obj = make_xpeel(FakeSerial())
        with pytest.raises(XpeelTimeoutError):
            obj._read_line(deadline=time.monotonic())  # already expired


# --------------------------------------------------------------------------- #
# peel
# --------------------------------------------------------------------------- #
class TestPeel:
    def test_default_command_and_success(self):
        fake = FakeSerial(lines=[b"*ack\r\n", b"*ready:00,00,00\r\n"])
        obj = make_xpeel(fake)
        assert obj.peel() == [0, 0, 0]
        assert fake.written == [b"*xpeel:41\r\n"]

    def test_custom_parameters(self):
        fake = FakeSerial(lines=[b"*ack\r\n", b"*ready:00,00,00\r\n"])
        obj = make_xpeel(fake)
        obj.peel(A=3, B=2)
        assert fake.written == [b"*xpeel:32\r\n"]

    @pytest.mark.parametrize("A,B", [(0, 1), (10, 1), (4, 0), (4, 5)])
    def test_invalid_parameters_raise_value_error(self, A, B):
        obj = make_xpeel(FakeSerial())
        with pytest.raises(ValueError):
            obj.peel(A=A, B=B)

    def test_device_error_raises_command_error(self):
        fake = FakeSerial(lines=[b"*ack\r\n", b"*ready:04,00,00\r\n"])
        obj = make_xpeel(fake)
        with pytest.raises(XpeelCommandError) as exc:
            obj.peel()
        assert exc.value.codes == [4]

    def test_warning_code_does_not_raise(self):
        # 20 = low tape: advisory, peel should still succeed.
        fake = FakeSerial(lines=[b"*ack\r\n", b"*ready:20,00,00\r\n"])
        obj = make_xpeel(fake)
        assert obj.peel() == [20, 0, 0]

    def test_ack_without_ready_times_out(self):
        fake = FakeSerial(lines=[b"*ack\r\n"])  # ack but no ready ever
        obj = make_xpeel(fake)
        with pytest.raises(XpeelTimeoutError):
            obj.peel()

    def test_ready_before_ack_is_response_error(self):
        fake = FakeSerial(lines=[b"*ready:00,00,00\r\n"])
        obj = make_xpeel(fake)
        with pytest.raises(XpeelResponseError):
            obj.peel()

    def test_no_response_times_out(self):
        obj = make_xpeel(FakeSerial())
        with pytest.raises(XpeelTimeoutError):
            obj.peel()

    def test_no_serial_raises_connection_error(self):
        obj = make_xpeel(fake=None)
        with pytest.raises(XpeelConnectionError):
            obj.peel()


# --------------------------------------------------------------------------- #
# check_seal
# --------------------------------------------------------------------------- #
class TestCheckSeal:
    def test_seal_detected(self):
        fake = FakeSerial(lines=[b"*ack\r\n", b"*ready:04,00,00\r\n"])
        obj = make_xpeel(fake)
        assert obj.check_seal() is True
        assert fake.written == [b"*sealcheck\r\n"]

    def test_no_seal(self):
        fake = FakeSerial(lines=[b"*ack\r\n", b"*ready:00,00,00\r\n"])
        obj = make_xpeel(fake)
        assert obj.check_seal() is False


# --------------------------------------------------------------------------- #
# tapeleft
# --------------------------------------------------------------------------- #
class TestTapeLeft:
    def test_returns_scaled_counts_without_ack(self):
        # tapeleft is not acknowledged: *tape then *ready.
        fake = FakeSerial(lines=[b"*tape:11,13\r\n", b"*ready:00,00,00\r\n"])
        obj = make_xpeel(fake)
        assert obj.tapeleft() == (110, 130)
        assert fake.written == [b"*tapeleft\r\n"]

    def test_unknown_counts(self):
        fake = FakeSerial(lines=[b"*tape:99,99\r\n", b"*ready:00,00,00\r\n"])
        obj = make_xpeel(fake)
        assert obj.tapeleft() == (None, None)

    def test_missing_tape_message_raises(self):
        fake = FakeSerial(lines=[b"*ready:00,00,00\r\n"])
        obj = make_xpeel(fake)
        with pytest.raises(XpeelResponseError):
            obj.tapeleft()


# --------------------------------------------------------------------------- #
# reset / restart / close
# --------------------------------------------------------------------------- #
class TestResetRestartClose:
    def test_reset_success(self):
        fake = FakeSerial(lines=[b"*ack\r\n", b"*ready:00,00,00\r\n"])
        obj = make_xpeel(fake)
        assert obj.reset() == [0, 0, 0]
        assert fake.written == [b"*reset\r\n"]

    def test_reset_error_raises(self):
        fake = FakeSerial(lines=[b"*ack\r\n", b"*ready:01,00,00\r\n"])
        obj = make_xpeel(fake)
        with pytest.raises(XpeelCommandError):
            obj.reset()

    def test_restart_consumes_startup_sequence(self):
        fake = FakeSerial(
            lines=[b"*ack\r\n", b"*poweron\r\n", b"*homing\r\n", b"*ready:00,00,00\r\n"]
        )
        obj = make_xpeel(fake)
        obj.restart()
        assert fake.written == [b"*restart\r\n"]
        assert fake.readline() == b""  # startup drained

    def test_close(self):
        fake = FakeSerial()
        obj = make_xpeel(fake)
        obj.close()
        assert fake.closed is True
        assert obj.serial is None

    
def main():
    """Smoke-check the doubles, then run the full test suite via pytest.

    Running ``python xpeel_tests.py`` will:
      1. Build a FakeSerial pre-loaded with a successful peel exchange.
      2. Wrap it in an Xpeel instance (using make_xpeel, which skips the
         hardware-opening __init__) and drive one peel() end to end.
      3. Hand off to pytest to execute every test in this file.
    """
    # 1. Instantiate the FakeSerial double with a scripted ack -> ready reply.
    fake_serial = FakeSerial(lines=[b"*ack\r\n", b"*ready:00,00,00\r\n"])

    # 2. Instantiate the driver around the fake and run a command.
    peeler = make_xpeel(fake_serial)
    codes = peeler.peel()  # sends *xpeel:41, waits for ack + ready
    print(f"[smoke] peel() returned error codes: {codes}")
    print(f"[smoke] bytes sent to device:        {fake_serial.written}")

    # 3. Run the whole suite and exit with pytest's status code.
    print("\n[smoke] running full test suite...\n")
    return pytest.main([__file__, "-v"])


if __name__ == '__main__':
    raise SystemExit(main())