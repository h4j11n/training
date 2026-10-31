# rm_serial_demo：串口与 ROS 话题之间的桥接

这个包只启动一个 ROS 节点 `/serial_bridge`。节点打开串口，将两种通信接起来：

```text
板端串口输出 → 桥接节点读线程 → /serial/rx → 其他 ROS 节点
板端串口输入 ← 桥接节点 on_tx 回调 ← /serial/tx ← 其他 ROS 节点
```

`rx`、`tx` 的方向以桥接节点为参照。桥接节点有一个 Publisher、一个 Subscription 和一个后台读线程；创建这些对象不会增加节点数量。

ROS 节点当然可以直接调用 `pyserial` 读写串口，这个包就是这样实现的。教程若提到“节点不要直接读串口”，应理解成建议把硬件通信集中到桥接节点，方便其他节点通过话题使用它；这不是 ROS 对节点代码的限制。

## 文件分工

```text
src/rm_serial_demo/
├── package.xml                 ROS 包资料、依赖、ament_python 构建方式
├── setup.py                    安装 Python 包、资源，注册运行入口
├── setup.cfg                   指定 ros2 run 查找的入口脚本安装位置
├── resource/rm_serial_demo      空标记文件，安装到 ROS 包索引
├── README.md                   本说明
└── rm_serial_demo/
    ├── __init__.py              标识 Python 包
    └── serial_bridge.py         实际节点、串口读写与清理逻辑
```

`setup.py` 中的入口：

```python
'serial_bridge = rm_serial_demo.serial_bridge:main'
```

表示：生成名为 `serial_bridge` 的可执行入口，运行时导入内层包的 `serial_bridge.py`，调用 `main()`。节点名称则由程序里的 `super().__init__('serial_bridge')` 决定。

外层 `rm_serial_demo` 是 ROS 包目录，内层同名目录是 Python 包目录。添加这个独立包不需要修改 `my_first_pkg`、`rm_gimbal_demo`、`rm_tf_demo` 或 `rm_msgs`，也不需要给本示例编写 CMakeLists.txt 或 launch 文件。

## 构建和环境

当前工作区的 `rm_ros2` Conda 环境使用 Python 3.10，已安装 `pyserial`；系统 Python 当前没有这个模块。因此这里明确用该环境的 Python 构建，确保生成的入口脚本也使用能导入 `serial` 的解释器。

在 Bash 终端执行：

```bash
cd /home/h44/vemon_cpp_basics/training/02-ros2-wheelleg/Tutorial/ros2_ws
source /opt/ros/humble/setup.bash
/home/h44/miniforge3/envs/rm_ros2/bin/python -c "import rclpy, serial; print(serial.__version__)"
/home/h44/miniforge3/envs/rm_ros2/bin/python /usr/bin/colcon build --packages-select rm_serial_demo --symlink-install
source install/setup.bash
ros2 pkg executables rm_serial_demo
```

最后一条应列出：

```text
rm_serial_demo serial_bridge
```

`package.xml` 的 `python3-serial` 是 ROS/rosdep 层的依赖声明；`setup.py` 的 `pyserial` 是 Python 包分发名称。`import serial` 导入的是这同一个库。修改依赖声明本身不会自动给当前 Python 安装库。如果另一个环境缺少它，要为实际运行入口所使用的 Python 安装依赖。

`source install/setup.bash` 是在当前 Bash 中加载搜索路径，不会启动节点。不同终端都需要配置自己的环境。下面每个运行 ROS 命令的新终端先执行：

```bash
cd /home/h44/vemon_cpp_basics/training/02-ros2-wheelleg/Tutorial/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 没有硬件时：两只虚拟串口的实验

需要 `socat`；若命令不存在，在 Ubuntu 上安装：

```bash
sudo apt install socat
```

下面的两个 PTY（伪终端）模拟一根串口线的两端：桥接节点占一端，终端模拟板子占另一端。一端写入的内容从另一端读出。

### 终端 A：创建串口对，保持运行

```bash
socat -d -d \
  pty,raw,echo=0,link=/tmp/ros_serial_bridge \
  pty,raw,echo=0,link=/tmp/ros_serial_board
```

`link=...` 为系统动态分配的 `/dev/pts/...` 创建稳定名称；`raw,echo=0` 避免伪终端修改、回显字节。两个链接只在这次 `socat` 运行期间有效。

### 终端 B：启动桥接节点

先按上文加载 ROS 环境，然后执行：

```bash
ros2 run rm_serial_demo serial_bridge --ros-args \
  -p port:=/tmp/ros_serial_bridge \
  -p baudrate:=115200
```

`port`、`baudrate` 是本节点的参数。PTY 不模拟真实串口波特率，但可以验证收发逻辑。

### 终端 C：订阅 ROS 接收话题

先加载 ROS 环境，再执行：

```bash
ros2 topic echo /serial/rx
```

### 终端 D：模拟板子，向桥接节点发送数据

```bash
printf 'hello from board\n' > /tmp/ros_serial_board
```

终端 C 应出现：

```text
data: hello from board
---
```

终端 B 同时会打印 `RX 串口 -> serial/rx: 'hello from board'`，表示读线程已经切出完整帧并发布。

还可以观察“收到字节”与“凑齐一帧”的区别：

```bash
printf 'half' > /tmp/ros_serial_board
```

这时没有换行符，桥接节点只在本地缓存，暂不发布。接着执行：

```bash
printf ' line\nsecond line\n' > /tmp/ros_serial_board
```

终端 C 应收到两条消息：`half line` 和 `second line`。这说明串口读到的一块数据不一定等于一条完整消息。

### 终端 D：改为观察板子收到的内容

执行：

```bash
cat /tmp/ros_serial_board
```

保持这个 `cat` 运行，让它成为板端唯一的读者。不要同时在其他地方再用 `cat`、串口工具或脚本读取同一个 `/tmp/ros_serial_board`，否则字节会被不同读者分走。

### 终端 E：通过 ROS 发送给板子

先加载 ROS 环境，再执行：

```bash
ros2 topic pub --once /serial/tx std_msgs/msg/String "{data: 'hello from ROS'}"
```

终端 D 应打印 `hello from ROS`。完整路径是：

```text
ros2 topic pub → /serial/tx → on_tx(msg) → 串口 write → socat → cat
```

终端 B 同时会打印 `TX serial/tx -> 串口: 'hello from ROS'`。

`cat` 只是读板端，终端仍可以通过 `printf ... > /tmp/ros_serial_board` 写同一端；读写是两个方向。

### 查看 ROS 结构

在已加载环境且空闲的终端执行：

```bash
ros2 node list
ros2 node info /serial_bridge
ros2 topic info /serial/rx --verbose
ros2 topic info /serial/tx --verbose
ros2 param get /serial_bridge port
ros2 param get /serial_bridge baudrate
```

`node info` 中应有 `/serial/rx` Publisher、`/serial/tx` Subscription。`topic echo`、`topic pub` 本身也会创建临时 ROS 节点，节点列表中看到它们是正常的。

退出时先在终端 B 按 `Ctrl+C`，让节点停止线程、释放串口；再停止 `cat`、`topic echo` 和终端 A 的 `socat`。

## 连接真实板子

确认设备路径、串口权限，并让板端使用相同波特率和文本协议。加载环境后运行：

```bash
ros2 run rm_serial_demo serial_bridge --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baudrate:=115200
```

有的设备名称是 `/dev/ttyACM0`，以实际设备为准。其他 ROS 节点仍然只需要发布 `/serial/tx`、订阅 `/serial/rx`，不用各自打开串口。

## 协议和线程为何这样写

- 本示例协议是 UTF-8 文本，每条消息以换行符 `\n` 结束。ROS 接收消息不包含作为分隔符的换行；板端使用 `\r\n` 时也会去掉末尾的 `\r`。发送到 `/serial/tx` 的字符串不能自带 `\r` 或 `\n`，否则节点会拒绝该条消息；发送回调会自动补上一个 `\n`。它不实现真实机器人常见的二进制帧头、长度、CRC 校验、序号或重发，需要按板端协议替换编解码逻辑。
- 串口是字节流：一次 `read()` 可能得到半帧，也可能得到多帧。因此读线程先累积字节，找到 `\n` 才切出完整消息。
- 无法按 UTF-8 解码的帧会被丢弃并打印警告。单帧限制为 64 KiB；超长帧会丢弃到下一处换行，然后恢复处理后续帧，避免未结束的坏帧无限积累内存。
- `timeout=0.1` 使串口无数据时最多等待约 0.1 秒再返回。读循环在独立线程里运行，避免长期占用默认单线程执行器，使 `/serial/tx` 的回调仍能执行。这是一种常用方案，也可用短时非阻塞轮询等其他设计。
- 串口写入配置 `write_timeout=0.5`，避免设备异常时发送回调无限期阻塞；这不代表写串口完全没有耗时。
- 发生串口读写异常或短写（没有写完完整帧）后，节点设置停止标志，读线程停止，后续发送请求也会被拒绝；检查端口、板子和 `socat` 后重启节点。程序不实现自动重连或自动重发。
- `Ctrl+C` 会触发退出清理：程序通知读线程停止、等待线程结束、关闭串口并销毁节点。ROS 信号处理可能已经先关闭 ROS 上下文，清理代码也处理这种情况；若上下文仍开启，最后再关闭它。`daemon=True` 本身不能替代资源清理。
