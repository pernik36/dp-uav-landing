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
QObject::moveToThread: Current thread (0x56d65da9ef50) is not the object's thread (0x56d65daa2ab0).
Cannot move to target thread (0x56d65da9ef50)

qt.qpa.plugin: Could not load the Qt platform plugin "xcb" in "/home/vojta/.local/lib/python3.10/site-packages/cv2/qt/plugins" even though it was found.
This application failed to start because no Qt platform plugin could be initialized. Reinstalling the application may fix this problem.

Available platform plugins are: xcb, eglfs, linuxfb, minimal, minimalegl, offscreen, vnc.

Stack trace (most recent call last):
#28   Object "[0xffffffffffffffff]", at 0xffffffffffffffff, in 
#27   Object "gz sim -g", at 0x56d65c40d1c4, in _start
#26   Object "/lib/x86_64-linux-gnu/libc.so.6", at 0x76a21a829e3f, in __libc_start_main
#25   Object "/lib/x86_64-linux-gnu/libc.so.6", at 0x76a21a829d8f, in 
#24   Object "gz sim -g", at 0x56d65c40d17e, in 
#23   Object "/lib/x86_64-linux-gnu/libruby-3.0.so.3.0", at 0x76a21aca8e19, in ruby_run_node
#22   Object "/lib/x86_64-linux-gnu/libruby-3.0.so.3.0", at 0x76a21aca5317, in 
#21   Object "/lib/x86_64-linux-gnu/libruby-3.0.so.3.0", at 0x76a21ae3a30c, in rb_vm_exec
#20   Object "/lib/x86_64-linux-gnu/libruby-3.0.so.3.0", at 0x76a21ae34c96, in 
#19   Object "/lib/x86_64-linux-gnu/libruby-3.0.so.3.0", at 0x76a21ae31fc5, in 
#18   Object "/lib/x86_64-linux-gnu/libruby-3.0.so.3.0", at 0x76a21ae2fc34, in 
#17   Object "/usr/lib/x86_64-linux-gnu/ruby/3.0.0/fiddle.so", at 0x76a21aa5444b, in 
#16   Object "/lib/x86_64-linux-gnu/libruby-3.0.so.3.0", at 0x76a21adfd088, in rb_nogvl
#15   Object "/usr/lib/x86_64-linux-gnu/ruby/3.0.0/fiddle.so", at 0x76a21aa53d6b, in 
#14   Object "/lib/x86_64-linux-gnu/libffi.so.8", at 0x76a21aa45492, in 
#13   Object "/lib/x86_64-linux-gnu/libffi.so.8", at 0x76a21aa48e2d, in 
#12   Object "/usr/lib/x86_64-linux-gnu/libgz-sim7-gz.so.7.7.0", at 0x76a2169ba95f, in runGui
#11   Object "/lib/x86_64-linux-gnu/libgz-sim7-gui.so.7", at 0x76a215a79c85, in gz::sim::v7::gui::runGui(int&, char**, char const*, char const*, int, char const*)
#10   Object "/lib/x86_64-linux-gnu/libgz-sim7-gui.so.7", at 0x76a215a774dc, in gz::sim::v7::gui::createGui(int&, char**, char const*, char const*, bool, char const*, int, char const*)
#9    Object "/lib/x86_64-linux-gnu/libgz-gui7.so.7", at 0x76a215d3c187, in gz::gui::Application::Application(int&, char**, gz::gui::WindowType)
#8    Object "/lib/x86_64-linux-gnu/libQt5Widgets.so.5", at 0x76a213f71cec, in QApplicationPrivate::init()
#7    Object "/lib/x86_64-linux-gnu/libQt5Gui.so.5", at 0x76a212b3bb6f, in QGuiApplicationPrivate::init()
#6    Object "/lib/x86_64-linux-gnu/libQt5Core.so.5", at 0x76a2148c0b16, in QCoreApplicationPrivate::init()
#5    Object "/lib/x86_64-linux-gnu/libQt5Gui.so.5", at 0x76a212b38c07, in QGuiApplicationPrivate::createEventDispatcher()
#4    Object "/lib/x86_64-linux-gnu/libQt5Gui.so.5", at 0x76a212b38712, in QGuiApplicationPrivate::createPlatformIntegration()
#3    Object "/lib/x86_64-linux-gnu/libQt5Core.so.5", at 0x76a214690ba2, in QMessageLogger::fatal(char const*, ...) const
#2    Object "/lib/x86_64-linux-gnu/libc.so.6", at 0x76a21a8287f2, in abort
#1    Object "/lib/x86_64-linux-gnu/libc.so.6", at 0x76a21a842475, in raise
#0    Object "/lib/x86_64-linux-gnu/libc.so.6", at 0x76a21a8969fc, in pthread_kill
Neúspěšně ukončen (SIGABRT) (Signál zaslán pomocí tkill() 29920 1000)
Subscribing to topic [/camera]
INFO  [lockstep_scheduler] setting initial absolute time to 2000 us
libEGL warning: egl: failed to create dri2 screen
libEGL warning: egl: failed to create dri2 screen
[Err] [SystemPaths.cc:525] Could not resolve file [tag_48_12__36_11_164__137.png]
INFO  [commander] LED: open /dev/led0 failed (22)
INFO  [mavlink] mode: Normal, data rate: 4000000 B/s on udp port 18570 remote port 14550
INFO  [tone_alarm] home set
INFO  [mavlink] mode: Onboard, data rate: 4000000 B/s on udp port 14580 remote port 14540
INFO  [mavlink] mode: Onboard, data rate: 4000 B/s on udp port 14280 remote port 14030
INFO  [mavlink] mode: Gimbal, data rate: 400000 B/s on udp port 13030 remote port 13280
INFO  [logger] logger started (mode=all)
INFO  [logger] Start file log (type: full)
INFO  [logger] [logger] ./log/2024-03-19/10_42_19.ulg
INFO  [logger] Opened full log file: ./log/2024-03-19/10_42_19.ulg
INFO  [mavlink] MAVLink only on localhost (set param MAV_{i}_BROADCAST = 1 to enable network)
INFO  [mavlink] MAVLink only on localhost (set param MAV_{i}_BROADCAST = 1 to enable network)
INFO  [px4] Startup script returned successfully
pxh> INFO  [tone_alarm] notify negative
INFO  [commander] Ready for takeoff!
INFO  [mavlink] partner IP: 127.0.0.1