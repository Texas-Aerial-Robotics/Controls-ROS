#!/bin/bash 
while [ 1 ]
do
	cpu_temp0=$(< /sys/class/thermal/thermal_zone0/temp)
	cpu_temp1=$(< /sys/class/thermal/thermal_zone1/temp)
	cpu_temp=$((cpu_temp0 + cpu_temp1))
	cpu_temp=$((cpu_temp/2))
	echo "Temp: Thermal_zone0: " | tr '\n' ' ' >> log.txt
	echo "scale=2; ( $cpu_temp0 / 1000 ) * 9 / 5 + 32" | bc | tr '\n' ' ' >> log.txt
	echo "Thermal_zone1: " | tr '\n' ' ' >> log.txt
	echo "scale=2; ( $cpu_temp1 / 1000 ) * 9 / 5 + 32" | bc | tr '\n' ' ' >> log.txt
	echo "Avg: " | tr '\n' ' ' >> log.txt
	echo "scale=2; ( $cpu_temp / 1000 ) * 9 / 5 + 32" | bc >> log.txt
	# echo "$(nvidia-settings -q GPUUtilization -q useddedicatedgpumemory)" >> log.txt
	echo "$(top -bn1 | grep 'Cpu')" >> log.txt
	echo "$(top -bn1 | grep 'Mem')" >> log.txt
	echo "------------------------" >> log.txt
	sleep 5
done