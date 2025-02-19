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
├── src/
│   └── object_height/          # Python package
│       ├── __init__.py
│       └── service_node.py     # Service implementation
├── srv/
│   └── ObjectHeight.srv        # Service definition
├── test/
│   └── test_service_node.py    # Unit tests
├── CMakeLists.txt             # Build configuration
└── package.xml                # Package metadata
```

## Setup and Testing

1. Make setup script executable and run it:
```bash
cd codestral_ros2_gen/scripts
chmod +x setup_pkg.sh
./setup_pkg.sh -p object_height
```

2. Build the package:
```bash
cd ../../test_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select object_height
source install/setup.bash
```

3. Run the generator to create service implementation:
```bash
python3 ../codestral_ros2_gen/examples/object_height/generator.py
```

The generator will:
- Create service implementation
- Build the package automatically
- Run tests to validate
- Report performance metrics

4. After successful generation, run the service:
```bash
ros2 run object_height object_height_service
```

5. Test in another terminal:
```bash
ros2 service call /calculate_object_height object_height/srv/ObjectHeight \
"{focal_length: 35.0, image_height: 1152, pixel_size: 3.45, object_distance: 6.5}"
```

## Expected Output
```
response:
  success: True
  object_height: 738.1
  feedback: ''
```

## Generation Process

1. Navigate to the test workspace and source it:
```bash
cd test_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

2. Run the generator:
```bash
python3 ../codestral_ros2_gen/examples/object_height/generator.py
```

The generator will:
1. Read service definition and test files from the package
2. Generate implementation using AI
3. Save the code to `src/object_height/src/object_height/service_node.py`
4. Build the package automatically
5. Run tests to validate the implementation
6. Perform multiple iterations for evaluation
7. Report performance metrics

After successful generation, you can immediately run the service:
```bash
ros2 run object_height object_height_service
```
