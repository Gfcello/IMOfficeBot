
# Start GPIO Daemon
echo "Starting GPIO Daemon"
pigpiod

# Setup virtual video interface 
echo "Starting video interface"
depmod -a
modprobe v4l2loopback video_nr=9
v4l2-ctl --list-devices

