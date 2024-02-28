# jlink
`JLinkGDBServer -device stm32f103c8`

# gdb
```
arm-none-eabi-gdb / gdb-multiarch
file build/crawler-servo.elf
target remote localhost:2331
monitor reset
load
break main
continue
TUI: c-x c-a

list
exec build/crawler-servo.elf
```

# canbus
`sudo ip link set up can0 type can bitrate 500000`
base address selectable by DIP switches

| dip | base address |
|-----|--------------|
| 000 | 0x010        |
| 001 | 0x020        |
| 010 | 0x030        |
| 011 | 0x040        |


### servo

* 8 channels starting from `base address`
* `2 bytes big-endian`, servo time in `1 µs` resolution
* error state: set channels to `1500 µs`

### monitoring

* responses in base `address + 8 + offset`
* analog signals directly from A/D, maximum: `4095`

| offset | value               | bytes |
|--------|---------------------|-------|
| 0      | battery voltage     | 2     |
| 1      | battery current     | 2     |
| ~2~    | ~wattage~           | 3     |
| ~3~    | ~watt seconds~      | 3     |
| ~4~    | ~uptime~            | 3     |
| ~5~    | ~user adc channel?~ | 2     |

### candump/send examples
with dip set to `000` battery voltage at `20 v` would come as `0x018#0fff`  
and setting `channel 0` to middle or `1500 µs` be `cansend can0 0x010#05dc` 
