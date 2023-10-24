```
arm-none-eabi-gdb
target remote localhost:2331
exec build/crawler-servo.elf
file build/crawler-servo.elf
TUI: c-x c-a
list
```
