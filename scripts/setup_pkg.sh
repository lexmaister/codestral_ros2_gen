#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

# Function for logging
log() {
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] $1"
}

# Function for error handling
error_exit() {
    log "ERROR: $1" >&2
    exit 1
}

# Function to display help message
show_help() {
    cat << EOF
Usage: $(basename "$0") -p <package_name> [-d <ros_distro>] [-h]

Options:
    -p, --package     Package name
    -d, --distro      ROS2 distribution (default: humble)
    -h, --help        Show this help
EOF
    exit 0
}

# Parse command line arguments
PACKAGE_NAME=""
ROS_DISTRO="humble"  # Default ROS distro

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            ;;
        -p|--package)
            PACKAGE_NAME="$2"
            shift 2
            ;;
        -d|--distro)
            ROS_DISTRO="$2"
            shift 2
            ;;
        *)
            error_exit "Unknown option: $1\nUse -h for help"
            ;;
    esac
done

# Verify package name was provided
if [ -z "$PACKAGE_NAME" ]; then
    error_exit "Package name is required.\nUse -h for help"
fi

# Verify ROS distro exists
if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    error_exit "ROS2 $ROS_DISTRO not found at /opt/ros/$ROS_DISTRO"
fi

log "Using ROS2 $ROS_DISTRO"

# Source ROS2 setup script
source "/opt/ros/$ROS_DISTRO/setup.bash" || error_exit "Failed to source ROS2 setup script"

# Get the script's directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
log "Script directory: $SCRIPT_DIR"

# Set workspace directories
EXAMPLES_DIR="$SCRIPT_DIR/../examples/$PACKAGE_NAME"
WS_DIR="$SCRIPT_DIR/../../test_ws"
SRC_DIR="$WS_DIR/src"
PACKAGE_DIR="$SRC_DIR/$PACKAGE_NAME"

# Create workspace directories
log "Creating workspace directories..."
mkdir -p "$SRC_DIR/$PACKAGE_NAME"

# Copy package files
log "Copying package files..."
cp -r "$EXAMPLES_DIR"/* "$PACKAGE_DIR/"

# Clean any existing build artifacts
log "Cleaning previous build artifacts..."
rm -rf "$WS_DIR/build/$PACKAGE_NAME"
rm -rf "$WS_DIR/install/$PACKAGE_NAME"

# Make Python files executable
find "$PACKAGE_DIR" -type f -name "*.py" -exec chmod +x {} \;

# Install dependencies
cd "$WS_DIR"
log "Installing dependencies..."
rosdep install --from-paths . --ignore-src -r -y

log "Package setup completed successfully. Now you can:"
log "1. cd test_ws"
log "2. source /opt/ros/humble/setup.bash"
log "3. colcon build --packages-select $PACKAGE_NAME"
