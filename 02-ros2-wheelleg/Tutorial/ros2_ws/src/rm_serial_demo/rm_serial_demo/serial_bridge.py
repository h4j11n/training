"""把 UTF-8 换行文本在串口和 ROS 2 话题之间双向转发。"""

import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import serial
from std_msgs.msg import String


class SerialBridge(Node):
    """一个 ROS 节点、一个串口对象，以及一个后台读取线程。"""

    MAX_FRAME_BYTES = 64 * 1024

    def __init__(self):
        super().__init__('serial_bridge')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        self._stop_event = threading.Event()
        self._write_lock = threading.Lock()
        try:
            if not port or baudrate <= 0:
                raise ValueError('port 不能为空，baudrate 必须大于 0')
            # read 最多等待 0.1 秒；write 也有超时，避免一直占用 ROS 回调。
            self.ser = serial.Serial(
                port, baudrate, timeout=0.1, write_timeout=0.5)
        except (serial.SerialException, OSError, ValueError):
            super().destroy_node()
            raise

        # RX/TX 方向以桥接节点为参照；消息类型沿用现成的 String。
        self.rx_pub = self.create_publisher(String, 'serial/rx', 10)
        self.tx_sub = self.create_subscription(String, 'serial/tx', self.on_tx, 10)

        self.thread = threading.Thread(
            target=self.read_loop, name='serial_reader', daemon=True)
        self.thread.start()
        self.get_logger().info(f'串口已打开: {port} @ {baudrate}')

    def read_loop(self):
        """串口是字节流：先缓存，遇到换行才发布一条完整消息。"""
        buffer = b''
        discarding = False
        while not self._stop_event.is_set() and rclpy.ok(context=self.context):
            try:
                size = min(self.ser.in_waiting or 1, 4096)
                data = self.ser.read(size)
            except (serial.SerialException, OSError) as exc:
                if not self._stop_event.is_set():
                    self.get_logger().error(f'读串口失败: {exc}')
                self._stop_event.set()
                break

            if not data:
                continue

            # 超长坏帧必须一直丢弃到下一个换行，不能把其尾部当成新帧。
            if discarding:
                _, separator, data = data.partition(b'\n')
                if not separator:
                    continue
                discarding = False

            buffer += data
            while b'\n' in buffer:
                line, buffer = buffer.split(b'\n', 1)
                if len(line) > self.MAX_FRAME_BYTES:
                    self.get_logger().warn('丢弃超过 64 KiB 的串口帧')
                    continue
                if self._stop_event.is_set() or not rclpy.ok(context=self.context):
                    return
                self._publish_line(line)

            if len(buffer) > self.MAX_FRAME_BYTES:
                self.get_logger().warn('串口帧超过 64 KiB 且没有换行，等待下一帧')
                buffer = b''
                discarding = True

    def _publish_line(self, line):
        """去掉 CRLF 中的 CR，严格按 UTF-8 解码后发布。"""
        try:
            text = line.removesuffix(b'\r').decode('utf-8')
        except UnicodeDecodeError:
            self.get_logger().warn('丢弃无法按 UTF-8 解码的串口帧')
            return

        try:
            self.rx_pub.publish(String(data=text))
        except Exception as exc:
            # Ctrl+C 可能先关闭 ROS context，再由 main 停止读取线程。
            if not self._stop_event.is_set() and rclpy.ok(context=self.context):
                self.get_logger().error(f'发布串口消息失败: {exc}')
            self._stop_event.set()
            return
        self.get_logger().info(f'RX 串口 -> serial/rx: {text!r}')

    def on_tx(self, msg):
        """ROS 回调：一条 String 对应一行，编码后写入串口。"""
        if self._stop_event.is_set():
            self.get_logger().warn('串口桥接已停止，忽略发送请求')
            return
        if '\n' in msg.data or '\r' in msg.data:
            self.get_logger().warn('发送内容不能自带换行；节点会自动添加行尾')
            return

        payload = (msg.data + '\n').encode('utf-8')
        try:
            with self._write_lock:
                count = self.ser.write(payload)
            if count != len(payload):
                self._stop_event.set()
                self.get_logger().error(
                    f'串口只写入了 {count}/{len(payload)} 字节，桥接已停止；'
                    '请检查设备和协议帧状态后重启节点')
                return
        except (serial.SerialException, OSError) as exc:
            # 异常前可能已写入半帧，停止后续发送，避免拼接出错误命令。
            self._stop_event.set()
            self.get_logger().error(
                f'写串口失败，桥接已停止: {exc}；请检查设备和协议帧状态后重启节点')
            return
        self.get_logger().info(f'TX serial/tx -> 串口: {msg.data!r}')

    def destroy_node(self):
        """先结束串口线程、关闭设备，再销毁 ROS 接口。"""
        self._stop_event.set()
        if self.ser.is_open:
            try:
                self.ser.cancel_read()
            except (serial.SerialException, OSError):
                pass
        self.thread.join(timeout=1.0)
        with self._write_lock:
            self.ser.close()
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = SerialBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (serial.SerialException, OSError, ValueError) as exc:
        rclpy.logging.get_logger('serial_bridge').error(f'无法启动串口桥接: {exc}')
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
