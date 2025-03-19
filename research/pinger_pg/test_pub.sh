#!bin/bash
source install/local_setup.bash
ros2 topic pub -r 1 /network_status network_scanner/msg/NetworkStatus "{stamp: {sec: 1610000000, nanosec: 0}, addresses: [{ip_address: '192.168.1.10', status: 'online'}, {ip_address: '192.168.1.20', status: 'offline'}]}"
