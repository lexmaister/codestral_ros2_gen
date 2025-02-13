# Example: Object Height Service

This example demonstrates how to generate the `object_height` service to get the height of an object in the scene.

## Prerequisites

Before you can use this project, you need to have `rosdep` installed. You can install it using the following commands:

### Ubuntu/Debian

```bash
sudo apt-get update
sudo apt-get install -y python3-rosdep
sudo rosdep init
rosdep update
```

### Other Distributions

Please refer to the [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html) for instructions on installing `rosdep` on other distributions.

## Setup Test Package

* Make executable and run setup script:
```bash
cd codestral_ros2_gen/scripts
chmod +x setup_pkg.sh
./setup_pkg.sh -p object_height
```

* Source the workspace for the current shell (for ROS2 humble):
```bash
source /opt/ros/humble/setup.bash
cd ../../test_ws
source install/setup.bash
```

* Check that package is in ros packages list and service is in ros interfaces list:
```
$ ros2 pkg list | grep object_height
object_height

$ ros2 interface list | grep ObjectHeight
    object_height/srv/ObjectHeight
```

## Generate Object Height Service


