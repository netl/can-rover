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
* 2x canbus+power connectors
* 8 servo channels
* 8 selectable addresses
* maximum rating: 20 V, 30 A
* **TODO: 8x buffered inputs with pull-up to 5V**
* **TODO: power switch with e-stop to servo power when no channels are active**

### CAN header
Jumper for 120R termination.  
May be used to power low power stuff. Connector rated at 3 A.  
|XH4|  signal |
|:-:|:--------|
| 1 | GND     |
| 2 | Battery |
| 3 | CAN\_L  |
| 4 | CAN\_H  |

### Servo channels
Jumper for powering via the PCB, remove if powering via ESC.

### Battery monitor
measures voltage and current

|measurement|  min  | max  |
|-----------|-------|------|
|voltage    |  0 V  | 20 V |
|current    | -10 A | 30 A |

## Firmware
* `sudo ip link set up can0 type can bitrate 500000`  
* All messages are `big-endian`. 
* [python-can](https://python-can.readthedocs.io/en/stable/scripts.html) has great tools for debugging.
* `cd firmware; make`

### Address
`base address` selectable by DIP switches:
| base address |1|2|3|
|-------------:|-|-|-|
| 0x010        |0|0|0|
| 0x020        |1|0|0|
| 0x030        |0|1|0|
| ...          |.|.|.|
| 0x070        |1|1|1|

**note: the dip switches are numbered left to right**

### Servo

* 8 channels starting from `base address`
* servo time in `1 µs` resolution
* error state: set channels to `1500 µs`

### Battery monitor
* responses in `base address + 8 + offset`
* analog signals directly from A/D, maximum: `4095`

| offset | value               | bytes |
|--------|---------------------|-------|
| 0      | battery voltage     | 2     |
| 1      | battery current     | 2     |

## ROS2 node
Provided node bridges firmware to ros topics.

### Installation
1. Install [ROS2](https://docs.ros.org/en/jazzy/Installation.html)
2. create [ros2_ws](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
3. `ln -s ros2_can_rover ~/ros2_ws/src/can_rover`
4. `cd ~/ros2_ws/`
5. `colcon build`
6. `ros2 run can_rover rover`

### Servo channels
defaulted to drive regular 180 degree servos.
* type: `Float32`
* topic: `/rover/channel_<n>`
* range: `-1 ... 1`

|parameter           |default |
|--------------------|--------|
|channel\_<n>\_scale |500     |
|channel\_<n>\_offset|1500    |

### Battery monitor

* type: `Float32`
* `/rover/voltage`
* `/rover/current`

|parameter      |default |
|---------------|--------|
|voltage\_scale |20/4096 |
|voltage\_offset|0.      |
|current\_scale |-30/4096|
|current\_offset|10.     |
