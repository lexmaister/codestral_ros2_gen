#!/bin/bash

# Function to display help message
usage() {
    echo "Usage: $0 -p <package_name>"
    echo "  -p <package_name>   Specify the package name"
    echo "  -h                  Display this help message"
    exit 1
}

# Parse command line arguments
while getopts ":p:h" opt; do
    case ${opt} in
        p)
            PACKAGE_NAME=$OPTARG
            ;;
        h)
            usage
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            usage
            ;;
        :)
            echo "Option -$OPTARG requires an argument." >&2
            usage
            ;;
    esac
done

# Check if PACKAGE_NAME is set
if [ -z "$PACKAGE_NAME" ]; then
    echo "Package name is required."
    usage
fi

# Define the workspace directory
WS_DIR="../../test_ws"

# Source the ROS2 setup script
source /opt/ros/humble/setup.bash

# Create the workspace directory if it doesn't exist
if [ ! -d "$WS_DIR" ]; then
    mkdir -p "$WS_DIR/src"
fi

# Navigate to the workspace source directory
cd "$WS_DIR/src"

# Check if the examples directory exists
EXAMPLES_DIR="../../examples/$PACKAGE_NAME"
if [ ! -d "$EXAMPLES_DIR" ]; then
    echo "Examples directory for $PACKAGE_NAME does not exist."
    exit 1
fi

# Create the package directory if it doesn't exist
if [ ! -d "$PACKAGE_NAME" ]; then
    ros2 pkg create --build-type ament_cmake "$PACKAGE_NAME" --dependencies rclpy pytest
fi

# Copy all files and folders recursively from examples to the package directory
cp -r "$EXAMPLES_DIR"/* "$PACKAGE_NAME/"

# Navigate to the workspace directory
cd "$WS_DIR"

# Check if all dependencies are installed
rosdep install --from-paths . --ignore-src -r -y

# Build the workspace and source it
colcon build --symlink-install --packages-select $PACKAGE_NAME
source install/setup.bash

echo "Setup for $PACKAGE_NAME package completed successfully."