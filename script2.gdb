set mi-async on
set mem inaccessible-by-default off
target extended-remote /dev/ttyBmpGdb
monitor swdp_scan
attach 1
monitor traceswo
stop

#file ./build/modbus.elf 
load ./build/modbus.hex 
compare-sections
#hbreak main
#next
#watch huart
#watch data[0]
run

