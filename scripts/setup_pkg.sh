#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

# Get the script's directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# ... existing code until package creation ...

# Create the package directory if it doesn't exist
if [ ! -d "$PACKAGE_NAME" ]; then
    log "Creating new ROS2 package: $PACKAGE_NAME"
    ros2 pkg create --build-type ament_cmake "$PACKAGE_NAME" \
        --license MIT \
        --dependencies rclpy pytest rosidl_default_generators || error_exit "Failed to create package"
fi

# Copy files from examples to package directory
log "Copying package files..."
find "$EXAMPLES_DIR" -type f ! -name 'generate.py' ! -name 'config.yaml' -exec cp --parents {} "$PACKAGE_NAME/" \; || \
    error_exit "Failed to copy package files"

# Update package.xml if interface files exist
if find "$PACKAGE_DIR" -type f \( -name "*.srv" -o -name "*.msg" -o -name "*.action" \) | grep -q .; then
    log "Interface files found. Updating package.xml..."
    
    # Check if package.xml exists
    if [ ! -f "$PACKAGE_DIR/package.xml" ]; then
        error_exit "package.xml not found"
    }

    # Add necessary dependencies for interface generation if not already present
    sed -i '/<\/package>/i \  <build_depend>rosidl_default_generators<\/build_depend>' "$PACKAGE_DIR/package.xml"
    sed -i '/<\/package>/i \  <exec_depend>rosidl_default_runtime<\/exec_depend>' "$PACKAGE_DIR/package.xml"
    sed -i '/<\/package>/i \  <member_of_group>rosidl_interface_packages<\/member_of_group>' "$PACKAGE_DIR/package.xml"

    # Update CMakeLists.txt
    log "Updating CMakeLists.txt for interface generation..."
    CMAKE_FILE="$PACKAGE_DIR/CMakeLists.txt"
    
    # Add interface generation code if not already present
    if ! grep -q "find_package(rosidl_default_generators REQUIRED)" "$CMAKE_FILE"; then
        # Add after the first find_package line
        sed -i '/find_package(ament_cmake REQUIRED)/a find_package(rosidl_default_generators REQUIRED)' "$CMAKE_FILE"
        
        # Add interface generation
        echo "" >> "$CMAKE_FILE"
        echo "# Generate interfaces" >> "$CMAKE_FILE"
        echo "file(GLOB srv_files RELATIVE \${CMAKE_CURRENT_SOURCE_DIR} \"srv/*.srv\")" >> "$CMAKE_FILE"
        echo "file(GLOB msg_files RELATIVE \${CMAKE_CURRENT_SOURCE_DIR} \"msg/*.msg\")" >> "$CMAKE_FILE"
        echo "file(GLOB action_files RELATIVE \${CMAKE_CURRENT_SOURCE_DIR} \"action/*.action\")" >> "$CMAKE_FILE"
        echo "" >> "$CMAKE_FILE"
        echo "rosidl_generate_interfaces(\${PROJECT_NAME}" >> "$CMAKE_FILE"
        echo "  \${srv_files}" >> "$CMAKE_FILE"
        echo "  \${msg_files}" >> "$CMAKE_FILE"
        echo "  \${action_files}" >> "$CMAKE_FILE"
        echo ")" >> "$CMAKE_FILE"
        echo "" >> "$CMAKE_FILE"
        echo "ament_export_dependencies(rosidl_default_runtime)" >> "$CMAKE_FILE"
    fi
fi

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