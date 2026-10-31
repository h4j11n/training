"""Exercise the bridge through a Linux PTY and actual ROS topic endpoints.

No physical serial device is opened. Run from the workspace with the ROS
environment sourced and rm_serial_demo installed or on PYTHONPATH.
"""

import os
import pty
import time
import uuid

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
import serial
from std_msgs.msg import String

from rm_serial_demo.serial_bridge import SerialBridge


def spin_until(executor, condition, timeout=5.0):
    """Service ROS callbacks until the condition holds, with a fixed deadline."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return
        executor.spin_once(timeout_sec=0.02)
    assert condition(), "Timed out waiting for ROS/serial communication"


def spin_for(executor, duration):
    deadline = time.monotonic() + duration
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        executor.spin_once(timeout_sec=min(0.02, remaining))


def read_available(fd):
    data = bytearray()
    while True:
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            return bytes(data)
        if not chunk:
            return bytes(data)
        data.extend(chunk)


class BridgeHarness:
    def __init__(self, master_fd, bridge, probe, executor):
        self.master_fd = master_fd
        self.bridge = bridge
        self.executor = executor
        self.received = []
        self.tx_pub = probe.create_publisher(String, 'serial/tx', 10)
        self.rx_sub = probe.create_subscription(
            String, 'serial/rx', lambda msg: self.received.append(msg.data), 10)
        self.bridge_destroyed = False

    def write_serial(self, payload, timeout=5.0):
        """Handle PTY backpressure while writing a large simulated board frame."""
        offset = 0
        deadline = time.monotonic() + timeout
        while offset < len(payload):
            assert time.monotonic() < deadline, "Timed out writing PTY data"
            try:
                offset += os.write(self.master_fd, payload[offset:offset + 4096])
            except BlockingIOError:
                self.executor.spin_once(timeout_sec=0.01)

    def destroy_bridge(self):
        if not self.bridge_destroyed:
            self.executor.remove_node(self.bridge)
            self.bridge.destroy_node()
            self.bridge_destroyed = True


@pytest.fixture
def serial_bridge():
    master_fd, slave_fd = pty.openpty()
    os.set_blocking(master_fd, False)
    slave_name = os.ttyname(slave_fd)
    namespace = '/serial_test_' + uuid.uuid4().hex
    bridge = None
    probe = None
    executor = None
    harness = None
    initialized = False

    try:
        rclpy.init(args=[
            '--ros-args', '-r', '__ns:=' + namespace,
            '-p', 'port:=' + slave_name, '-p', 'baudrate:=115200',
        ])
        initialized = True
        bridge = SerialBridge()
        probe = rclpy.create_node(
            'serial_test_probe', namespace=namespace, use_global_arguments=False)
        executor = SingleThreadedExecutor()
        executor.add_node(bridge)
        executor.add_node(probe)
        harness = BridgeHarness(master_fd, bridge, probe, executor)

        spin_until(executor, lambda: (
            harness.tx_pub.get_subscription_count() >= 1
            and bridge.rx_pub.get_subscription_count() >= 1))
        yield harness
    finally:
        if harness is not None:
            harness.destroy_bridge()
        elif bridge is not None:
            bridge.destroy_node()
        if probe is not None:
            if executor is not None:
                executor.remove_node(probe)
            probe.destroy_node()
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if initialized:
            rclpy.shutdown()
        os.close(slave_fd)
        os.close(master_fd)


def test_full_duplex_framing_and_shutdown(serial_bridge):
    harness = serial_bridge

    # A real ROS subscription callback must write UTF-8 plus one newline.
    expected = '目标 0.8 0.4\n'.encode('utf-8')
    written = bytearray()
    harness.tx_pub.publish(String(data='目标 0.8 0.4'))

    def tx_received():
        written.extend(read_available(harness.master_fd))
        return len(written) >= len(expected)

    spin_until(harness.executor, tx_received)
    assert bytes(written) == expected

    # A complete line must travel from the PTY through the actual RX topic.
    os.write(harness.master_fd, b'first\n')
    spin_until(harness.executor, lambda: len(harness.received) >= 1)
    assert harness.received == ['first']

    # An incomplete serial frame must remain buffered, rather than published.
    os.write(harness.master_fd, b'par')
    spin_for(harness.executor, 0.15)
    assert harness.received == ['first']

    # Complete the partial frame and send two more in one serial write.
    os.write(harness.master_fd, b'tial\none\ntwo\r\n')
    spin_until(harness.executor, lambda: len(harness.received) >= 4)
    assert harness.received == ['first', 'partial', 'one', 'two']

    # Shutdown must release the port and join the background reader even when
    # there is no more data to wake it up. Do this outside executor callbacks.
    assert harness.bridge.thread.is_alive()
    assert harness.bridge.ser.is_open
    harness.destroy_bridge()
    assert not harness.bridge.thread.is_alive()
    assert not harness.bridge.ser.is_open


def test_invalid_utf8_frame_does_not_corrupt_next_frame(serial_bridge):
    harness = serial_bridge
    os.write(harness.master_fd, b'\xff\xfe\n' + '有效消息\n'.encode('utf-8'))
    spin_until(harness.executor, lambda: bool(harness.received))
    spin_for(harness.executor, 0.1)
    assert harness.received == ['有效消息']


def test_oversize_unterminated_frame_is_discarded_through_newline(serial_bridge):
    harness = serial_bridge
    harness.write_serial(b'x' * (harness.bridge.MAX_FRAME_BYTES + 1))
    spin_for(harness.executor, 0.15)
    assert harness.received == []

    # After discarding the overfull buffer, its remaining bytes still belong
    # to the bad frame. Only data after its delimiter starts a new frame.
    harness.write_serial(b'bad_frame_tail\nvalid_after_oversize\n')
    spin_until(harness.executor, lambda: bool(harness.received))
    spin_for(harness.executor, 0.1)
    assert harness.received == ['valid_after_oversize']


def test_tx_rejects_embedded_line_delimiters(serial_bridge):
    harness = serial_bridge
    harness.tx_pub.publish(String(data='bad\nline'))
    harness.tx_pub.publish(String(data='bad\rline'))
    harness.tx_pub.publish(String(data='valid_marker'))
    written = bytearray()

    def marker_received():
        written.extend(read_available(harness.master_fd))
        return b'valid_marker\n' in written

    spin_until(harness.executor, marker_received)
    assert bytes(written) == b'valid_marker\n'


@pytest.mark.parametrize('failure', ['short_write', 'timeout'])
def test_failed_write_stops_later_tx(serial_bridge, monkeypatch, failure):
    harness = serial_bridge
    bridge = harness.bridge
    original_write = bridge.ser.write
    original_callback = bridge.tx_sub.callback
    write_calls = []
    handled_messages = []

    def partial_write(payload):
        write_calls.append(payload)
        # Simulate a frame whose first bytes reached the board before failure.
        original_write(payload[:2])
        if failure == 'short_write':
            return 2
        raise serial.SerialTimeoutException('simulated write timeout')

    def observe_callback(msg):
        handled_messages.append(msg.data)
        return original_callback(msg)

    monkeypatch.setattr(bridge.ser, 'write', partial_write)
    monkeypatch.setattr(bridge.tx_sub, 'callback', observe_callback)
    harness.tx_pub.publish(String(data='first_request'))
    spin_until(harness.executor, lambda: bool(handled_messages))
    assert bridge._stop_event.is_set()
    assert write_calls == [b'first_request\n']

    harness.tx_pub.publish(String(data='later_request'))
    spin_until(harness.executor, lambda: len(handled_messages) >= 2)
    assert handled_messages == ['first_request', 'later_request']
    # The second ROS message really ran the callback, but must not be appended
    # to the unterminated prefix left by the failed first write.
    assert write_calls == [b'first_request\n']
    assert read_available(harness.master_fd) == b'fi'
