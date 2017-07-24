roscore &

sleep 3

roslaunch mavros apm.launch fcu_url:="/dev/ttyS0:921600" &

sleep 3

rosrun flightCode takeoffTest &
