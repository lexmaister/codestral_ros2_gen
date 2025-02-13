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

## Setup of test node

Make executable and run setup script:

```bash
cd codestral_ros2_gen/scripts
chmod +x setup_pkg.sh
source setup_pkg.sh -p object_height
```


