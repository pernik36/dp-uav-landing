______  __   __    ___ 
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.

INFO  [px4] startup script: /bin/sh etc/init.d-posix/rcS 0
env SYS_AUTOSTART: 4001
INFO  [param] selected parameter default file parameters.bson
INFO  [param] importing from 'parameters.bson'
INFO  [parameters] BSON document size 450 bytes, decoded 450 bytes (INT32:15, FLOAT:7)
INFO  [param] selected parameter backup file parameters_backup.bson
INFO  [dataman] data manager file './dataman' size is 7866680 bytes
INFO  [init] starting gazebo with world: /home/vojta/PX4/PX4-Autopilot/Tools/simulation/gz/worlds/pokus1.sdf
INFO  [init] PX4_GZ_MODEL_POSE set, spawning at: -5.0,3.0,0,0,0,0.0
INFO  [gz_bridge] world: pokus1, model name: x500_cam_0, simulation model: x500_cam
INFO  [gz_bridge] Requested Model Position: -5.0,3.0,0,0,0,0.0
Subscribing to topic [/camera]
libEGL warning: egl: failed to create dri2 screen
libEGL warning: egl: failed to create dri2 screen
INFO  [lockstep_scheduler] setting initial absolute time to 2000 us
libEGL warning: egl: failed to create dri2 screen
libEGL warning: egl: failed to create dri2 screen
[GUI] [Err] [SystemPaths.cc:525] Could not resolve file [tag_48_12__36_11_164__137.png]
[Err] [SystemPaths.cc:525] Could not resolve file [tag_48_12__36_11_164__137.png]
INFO  [commander] LED: open /dev/led0 failed (22)
INFO  [mavlink] mode: Normal, data rate: 4000000 B/s on udp port 18570 remote port 14550
INFO  [tone_alarm] home set
INFO  [mavlink] mode: Onboard, data rate: 4000000 B/s on udp port 14580 remote port 14540
INFO  [mavlink] mode: Onboard, data rate: 4000 B/s on udp port 14280 remote port 14030
INFO  [mavlink] mode: Gimbal, data rate: 400000 B/s on udp port 13030 remote port 13280
INFO  [logger] logger started (mode=all)
INFO  [logger] Start file log (type: full)
INFO  [logger] [logger] ./log/2024-03-19/10_48_23.ulg
INFO  [logger] Opened full log file: ./log/2024-03-19/10_48_23.ulg
INFO  [mavlink] MAVLink only on localhost (set param MAV_{i}_BROADCAST = 1 to enable network)
INFO  [mavlink] MAVLink only on localhost (set param MAV_{i}_BROADCAST = 1 to enable network)
INFO  [px4] Startup script returned successfully
pxh> INFO  [tone_alarm] notify negative
INFO  [mavlink] partner IP: 127.0.0.1
INFO  [commander] Ready for takeoff!