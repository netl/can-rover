# jlink
`JLinkGDBServer -device stm32f103c8`

# gdb
```
arm-none-eabi-gdb / gdb-multiarch
file build/crawler-servo.elf
target remote localhost:2331
break main
continue
TUI: c-x c-a

list
exec build/crawler-servo.elf
```

# canbus
`sudo ip link set up can0 type can bitrate 500000`
