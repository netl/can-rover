# jlink
`JLinkGDBServer -device stm32f103c8`

# gdb
```
arm-none-eabi-gdb
target remote localhost:2331
exec build/crawler-servo.elf
file build/crawler-servo.elf
TUI: c-x c-a
list
```

# canbus
`sudo ip link set up can0 type can bitrate 500000`
