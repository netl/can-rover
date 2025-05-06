# can-rover

The goal is to convert cheap RC car kits in to computer driven robots by
replacing the receiver with a separate module.  
The project consists of three parts working as a whole:

1. [Hardware](#hardware)
2. [Firmware](#firmware)
3. [ROS2 node](#ros2-node)

## Hardware

features:

* XT60 passthrough
* fuse protected battery input
* battery current & voltage measurement
    * maximum rating: 20 V, 30 A
    * a charge current of 10 A max can be measured
* 2x canbus+power connectors
* 8 buffered servo channels
* 8 selectable addresses
* 8 buffered inputs with pull-up to 5V
* e-stop controlled servo power

### CAN header

Jumper for 120R termination.  
May be used to power low power stuff. Connector rated at 3 A.  

|pin|  signal |
|---|---------|
| 1 | GND     |
| 2 | Battery |
| 3 | CAN\_L  |
| 4 | CAN\_H  |

## Firmware

* can-utils: `sudo ip link set up can0 type can bitrate 500000`
* All messages are `big-endian`. 
* [python-can](https://python-can.readthedocs.io/en/stable/scripts.html) has great tools for debugging.
* build with `cd firmware; make`

### CAN messages

`base_address` is selectable by rotary switch. All messages have an offset from that address.
```c
uint8_t arbitration_id = base_address << 8; // address 0 would be: 0x000 and 7: 0x070
```

offset  |parameter       |I/O|bytes
--------|----------------|---|-----
0x0     | servo 1        | I | 1
...     | servo ...      | I | 1
0x7     | servo 8        | I | 1
0x8     | battery Voltage| O | 2
0x9     | battery Current| O | 2
0xa     | sensors 1 .. 8 | O | 1

### Servo

PWM in `1 µs` resolution and if channel hasn't been set within `1 s` it will default to `1500 µs`. For example: `servo 3` with `base_address = 5` would be set to `-90°` with:

```bash
$ cansend can0 052#3e8
```

### Battery monitor

Analog signals directly from A/D, maximum: `4095`

### Inputs

One byte where each bit represents an input

```c
bool value = inputs >> n-1 & 0x1;
```

## ROS2 node
Provided node bridges firmware to ros topics.

### Installation

1. Install [ROS2](https://docs.ros.org/en/jazzy/Installation.html)
2. create [ros2_ws](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

```bash
apt install python3-rosdep python3-colcon-common-extensions
ln -s ros2_can_rover ~/ros2_ws/src/can_rover
cd ~/ros2_ws/
colcon build
source install/setup.bash
rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
ros2 run can_rover can_rover
```

### Topics and parameters

Interfaces provided are bare minimum for simplicity

topic                      |type     |publish / subscribe|min   |max  |
---------------------------|---------|-------------------|------|-----|
`/rover/voltage`           |[Float32]|subscribe          |0   \*|20\* |
`/rover/current`           |[Float32]|subscribe          |-10 \*|30\* |
`/rover/watts`             |[Float32]|subscribe          |-200\*|600\*|
`/rover/wattHours`         |[Float32]|publish & subscribe|      |     |
`/rover/channel_<n>`       |[Float32]|publish            | -1   | 1   |
`/rover/input_<n>`         |[Bool]   |subscribe          | 0    | 1   |

*\*min/max values limited hardware tolerances and calibration*

parameter                  |type     |default   |
---------------------------|---------|----------|
`/rover/voltage_scale`     |[Float32]|20 / 2^12 |
`/rover/voltage_offset`    |[Float32]|0         |
`/rover/current_scale`     |[Float32]|-40 / 2^12|
`/rover/current_offset`    |[Float32]|10        |
`/rover/channel_<n>_scale` |[Float32]|500       |
`/rover/channel_<n>_offset`|[Float32]|1500      |

[Float32]:https://github.com/ros2/example_interfaces/blob/master/msg/Float32.msg
[Bool]:https://github.com/ros2/example_interfaces/blob/master/msg/Bool.msg
