#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

# Get the script's directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Function to display help message
usage() {
    echo "Usage: $0 -p <package_name> [-d <ros2_distro>]"
    echo "  -p <package_name>   Specify the package name"
    echo "  -d <ros2_distro>    Specify ROS2 distribution (default: humble)"
    echo "  -h                  Display this help message"
    exit 1
}

# Function to log messages with timestamp
log() {
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] $1"
}

# Function to handle errors
error_exit() {
    log "ERROR: $1" >&2
    exit 1
}

# Parse command line arguments
while getopts ":p:d:h" opt; do
    case ${opt} in
        p)
            PACKAGE_NAME=$OPTARG
            ;;
        d)
            ROS_DISTRO=$OPTARG
            ;;
        h)
            usage
            ;;
        \?)
            error_exit "Invalid option: -$OPTARG"
            ;;
        :)
            error_exit "Option -$OPTARG requires an argument."
            ;;
    esac
done

# Check if PACKAGE_NAME is set
if [ -z "$PACKAGE_NAME" ]; then
    error_exit "Package name is required."
fi

# Set default ROS2 distribution if not specified
ROS_DISTRO=${ROS_DISTRO:-"humble"}

# Define important directories
EXAMPLES_DIR="${SCRIPT_DIR}/../examples/$PACKAGE_NAME"
WS_DIR="${SCRIPT_DIR}/../../test_ws"
PACKAGE_DIR="$WS_DIR/src/$PACKAGE_NAME"

# Check if the examples directory exists
if [ ! -d "$EXAMPLES_DIR" ]; then
    error_exit "Examples directory for $PACKAGE_NAME does not exist at $EXAMPLES_DIR"
fi

# Check and source ROS2 setup
ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
if [ ! -f "$ROS_SETUP" ]; then
    error_exit "ROS2 $ROS_DISTRO not found at /opt/ros/$ROS_DISTRO"
fi
source "$ROS_SETUP"

# Create and navigate to the workspace source directory
log "Creating workspace directory structure..."
mkdir -p "$WS_DIR/src"
cd "$WS_DIR/src"

# Create the package directory if it doesn't exist
if [ ! -d "$PACKAGE_NAME" ]; then
    log "Creating new ROS2 package: $PACKAGE_NAME"
    ros2 pkg create --build-type ament_python "$PACKAGE_NAME" \
        --license MIT \
        --dependencies rclpy pytest || error_exit "Failed to create package"
fi

# Copy files from examples to package directory
log "Copying package files..."
find "$EXAMPLES_DIR" -type f ! -name 'generate.py' ! -name 'config.yaml' -exec cp --parents {} "$PACKAGE_NAME/" \; || \
    error_exit "Failed to copy package files"

# Make any python files executable
log "Setting executable permissions for Python files..."
find "$PACKAGE_DIR" -type f -name "*.py" -exec chmod +x {} \; || \
    error_exit "Failed to set executable permissions"

# Navigate to the workspace directory
cd "$WS_DIR"

# Install dependencies
log "Installing dependencies..."
rosdep install --from-paths . --ignore-src -r -y || \
    error_exit "Failed to install dependencies"

# Build the workspace
log "Building workspace..."
colcon build --symlink-install --packages-select "$PACKAGE_NAME" || \
    error_exit "Failed to build workspace"

log "Setup for $PACKAGE_NAME package completed successfully."

# Source the local workspace
WORKSPACE_SETUP="$WS_DIR/install/setup.bash"
if [ -f "$WORKSPACE_SETUP" ]; then
    log "To use the package, source the workspace:"
    echo "source $WORKSPACE_SETUP"
else
    error_exit "Workspace setup file not found at $WORKSPACE_SETUP"
fi