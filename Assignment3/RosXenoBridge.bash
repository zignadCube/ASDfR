echo ===========================================
echo == Ask root password for installation    ==
echo ===========================================
sudo ls > /dev/null

# Set certain registers to init the SPI driver into correct state
# See https://github.com/DenizUgur/RPi4-EVL-4xSPI
sudo busybox devmem 0xfe204e00 32 0x03800000
# Give corect permissions to x-buffer
sudo chmod 666 /dev/evl/xbuf/Ros-Xeno
sudo chmod 666 /dev/evl/xbuf/Xeno-Ros
# Start ros_xeno_bridge node 
ros2 run ros_xeno_bridge RosXenoBridge
