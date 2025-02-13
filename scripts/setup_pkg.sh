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

# Check workspace directories existance
if [ ! -d "$EXAMPLES_DIR" ]; then
    error_exit "Sources directory for package '$PACKAGE_NAME' not found at $EXAMPLES_DIR"
fi

if [ ! -d "$WS_DIR" ]; then
    error_exit "ROS2 workspace directory not found at $WS_DIR"
fi

# Create workspace directories
log "Creating workspace directories..."
mkdir -p "$SRC_DIR" || error_exit "Failed to create workspace directories"

# Navigate to the src directory
cd "$SRC_DIR" || error_exit "Failed to navigate to src directory"

# Create the package directory
log "Creating new ROS2 package: $PACKAGE_NAME"
ros2 pkg create --build-type ament_cmake "$PACKAGE_NAME" \
    --license MIT \
    --dependencies rclpy python3-pytest rosidl_default_generators || error_exit "Failed to create package"

log "Copying package files and directories..."
# First, create directory structure
find "$EXAMPLES_DIR" -type d | while read dir; do
    # Create relative path by removing EXAMPLES_DIR prefix
    rel_dir="${dir#$EXAMPLES_DIR}"
    # Create directory in package
    mkdir -p "$PACKAGE_NAME$rel_dir"    
    log "Created '$PACKAGE_NAME$rel_dir'"
done

# Then copy all files (excluding generate.py and config.yaml)
log "Finding files to copy..."
mapfile -d $'\0' files < <(find "$EXAMPLES_DIR" -type f \
    ! -name 'generate.py' \
    ! -name 'config.yaml' -print0)

if [ ${#files[@]} -eq 0 ]; then
    log "No files found to copy"
    exit 0
fi

log "Copying ${#files[@]} files to $PACKAGE_NAME..."
for file in "${files[@]}"; do
    rel_path="${file#$EXAMPLES_DIR/}"
    mkdir -p "$PACKAGE_NAME/$(dirname "$rel_path")"
    log "Copying file '$file'"
    cp "$file" "$PACKAGE_NAME/$rel_path" || error_exit "Failed to copy: $file"
done

log "Directory structure and files copied successfully"

# Update package.xml if interface files exist
if find "$PACKAGE_DIR" -type f \( -name "*.srv" -o -name "*.msg" -o -name "*.action" \) | grep -q .; then
    log "Interface files found. Updating package.xml..."
    
    # Check if package.xml exists
    if [ ! -f "$PACKAGE_DIR/package.xml" ]; then
        error_exit "package.xml not found"
    fi

    log "Adding rosidl dependencies to package.xml..."
    # Add interface packages group membership first
    sed -i '/<\/package>/i \  <member_of_group>rosidl_interface_packages<\/member_of_group>' "$PACKAGE_DIR/package.xml"
    # Add runtime dependency
    sed -i '/<\/package>/i \  <exec_depend>rosidl_default_runtime<\/exec_depend>' "$PACKAGE_DIR/package.xml"
    log "Dependencies added successfully"

    # Update CMakeLists.txt
    log "Updating CMakeLists.txt for interface generation..."
    CMAKE_FILE="$PACKAGE_DIR/CMakeLists.txt"
    
    # Backup original CMakeLists.txt
    cp "$CMAKE_FILE" "${CMAKE_FILE}.backup"
    
    # Create new CMakeLists.txt with proper interface generation
    cat > "$CMAKE_FILE" << EOL
cmake_minimum_required(VERSION 3.8)
project(${PACKAGE_NAME})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Generate interfaces
file(GLOB srv_files RELATIVE \${CMAKE_CURRENT_SOURCE_DIR} "srv/*.srv")
file(GLOB msg_files RELATIVE \${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")
file(GLOB action_files RELATIVE \${CMAKE_CURRENT_SOURCE_DIR} "action/*.action")

rosidl_generate_interfaces(\${PROJECT_NAME}
  \${srv_files}
  \${msg_files}
  \${action_files}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
EOL
fi

# Make any python files executable
log "Setting executable permissions for Python files..."
find "$PACKAGE_DIR" -type f -name "*.py" -exec chmod +x {} \; || \
    error_exit "Failed to set executable permissions"

# Navigate to the workspace directory
cd "$WS_DIR" || error_exit "Failed to navigate to workspace directory"

# Install dependencies
log "Installing dependencies..."
rosdep install --from-paths . --ignore-src -r -y || \
    error_exit "Failed to install dependencies"

# Verify interface files
log "Verifying interface files..."
find "$PACKAGE_DIR" -type f \( -name "*.srv" -o -name "*.msg" -o -name "*.action" \) -exec echo "Found interface file: {}" \;

# Clean build if needed
log "Cleaning build directory..."
rm -rf "$WS_DIR/build/$PACKAGE_NAME"
rm -rf "$WS_DIR/install/$PACKAGE_NAME"

# Build the workspace
log "Building workspace..."
colcon build --symlink-install --packages-select "$PACKAGE_NAME" --event-handlers console_direct+ || \
    error_exit "Failed to build workspace"

log "Setup for $PACKAGE_NAME package completed successfully."