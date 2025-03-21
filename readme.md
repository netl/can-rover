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
* 2x canbus+power connectors
* 8 buffered servo channels
* 8 selectable addresses
* 8 buffered inputs with pull-up to 5V
* e-stop controlled servo power

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
* can-utils: `sudo ip link set up can0 type can bitrate 500000`
* All messages are `big-endian`. 
* [python-can](https://python-can.readthedocs.io/en/stable/scripts.html) has great tools for debugging.
* `cd firmware; make`

### Address
`base address` selectable by DIP switches:
| base address |3|2|1|
|-------------:|-|-|-|
| 0x000        |0|0|0|
| 0x010        |0|0|1|
| 0x020        |0|1|0|
| ...          |.|.|.|
| 0x070        |1|1|1|

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
```
apt install python3-rosdep python3-colcon-common-extensions
ln -s ros2_can_rover ~/ros2_ws/src/can_rover
cd ~/ros2_ws/`
colcon build`
source install/setup.bash'
rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
ros2 run can_rover can_rover
```

### Servo channels
defaulted to drive regular 180 degree servos.
* type: `Float32`
* topic: `/rover/channel_<n>`
* range: `-1 ... 1`

|parameter           |default |
|--------------------|--------|
|channel\_n\_scale |500     |
|channel\_n\_offset|1500    |

### Battery monitor

* type: `Float32`
* `/rover/voltage`
* `/rover/current`

|parameter      |default |
|---------------|--------|
|voltage\_scale |20/4096 |
|voltage\_offset|0.      |
|current\_scale |-40/4096|
|current\_offset|10.     |
