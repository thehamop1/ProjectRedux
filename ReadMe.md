# Explanation
- This is a refactored version of what the old repository used to be. It adds new features and overall improvements. 

# Dependencies
- CMake 3.24
- GCC 11.3
- can-utils

# How to setup vcan0 if it doesnt exist
modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# How to monitor can activity:
candump vcan0

# How to send packets:
cansend vcan0 123#1122334455667788
