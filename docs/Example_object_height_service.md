# Example: Object Height Service

This example demonstrates generating a ROS2 service that calculates real-world object height using camera parameters.

## Service Description
The service calculates the actual height of an object using:
- Camera focal length
- Object's height in image pixels
- Camera sensor pixel size
- Distance to the object

The implementation must handle unit conversions and edge cases appropriately.

## Package Structure
```
object_height/
├── object_height/              # Python package
│   ├── __init__.py
│   └── service_node.py         # Test service implementation for package init
├── srv/
│   └── ObjectHeight.srv        # Service definition
├── test/
│   └── test_service_node.py    # Unit tests
├── CMakeLists.txt              # Build configuration
└── package.xml                 # Package metadata
```

## Setup and Testing

1. Make setup script executable and run it:
```bash
cd codestral_ros2_gen/scripts
chmod +x setup_pkg.sh
./setup_pkg.sh -p object_height
```

2. Verify the package setup:
```bash
cd ../../test_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Check if package is installed
ros2 pkg list | grep object_height

# Verify service interface
ros2 interface package object_height

# List available executables
ros2 pkg executables object_height

# Run test service
ros2 run object_height object_height_service
```

4. Open another terminal and test test that service publishes in topic `/service_status`:
``` bash
# Navigate to the test_ws directory
cd test_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /service_status
```

## Generating Service with the Model

Follow these steps to generate the service implementation using the AI model:

### **Generate the Service Implementation:**

Ensure you are in the test workspace directory (e.g., `test_ws`), then execute:
```bash
python3 ../codestral_ros2_gen/examples/object_height/generator.py
```
This command will:
- Load the local configuration from `config.yaml`
- Construct a detailed prompt by reading the service interface and test file (as specified in the input section)
- Use the AI model to create a ROS2 service node implementation
- Save the generated code to the output file defined in the configuration

### **Review the Generation Output:**

The generator will output summary metrics and status messages. Check the console for:
- Success or failure status
- Detailed metrics about generation time, attempt count, and any errors

## Testing the Generated Service

Once the code is generated, test it as described below:

- **Start the Service Node:**
  In Terminal 1, run:
  ```bash
  ros2 run object_height object_height_service
  ```

- **Call the Service:**
  In Terminal 2, issue a service call with appropriate parameters:
  ```bash
  ros2 service call /calculate_object_height object_height/srv/ObjectHeight \
  "{focal_length: 35.0, image_height: 1152, pixel_size: 3.45, object_distance: 6.5}"
  ```

## Expected Output

The service call should return a response similar to:
```
response:
  success: True
  object_height: 738.1
  feedback: ''
```

## Troubleshooting

If the service fails to generate or the output is not as expected, verify:
- **ROS2 Environment:** Ensure you have sourced the ROS2 setup files:
  ```bash
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ```
- **Package Status:** Confirm the `object_height` package is installed:
  ```bash
  ros2 pkg list | grep object_height
  ```
- **Node and Service Status:** Check active nodes and services:
  ```bash
  ros2 node list
  ros2 service list
  ```
