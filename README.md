
# Point Cloud Saver

This ROS 2 package subscribes to a PointCloud2 message and saves the data to PCD files.

## Usage
### Step 0: Build the ROS Node
```
colcon build
source ./install/setup.bash
```

### Step 1: Play Your ROS Bag

Play your ROS bag with the appropriate QoS profile and topic remapping:

```sh
ros2 bag play ~/bag/bag_0.db3 -l --remap /TPCK_HESAI_FRONT_MSG:=TPCK_LIDAR_TOP_CENTER
```

### Step 2: Run the Point Cloud Saver Node

Run the `point_cloud_saver` node with the required parameters:

```sh
ros2 run point_cloud_saver point_cloud_saver --ros-args -p topic_name:=/TPCK_LIDAR_TOP_CENTER -p qos_history:=10 -p qos_reliability:=best_effort -p qos_durability:=volatile
```

### Step 3: Specify Arguments in VS Code

Refer to the `.vscode/launch.json` file for specifying arguments when launching the node from Visual Studio Code.

### Troubleshooting

1. **No PCD Files Saved**: If the program does not save PCD files, the QoS settings are likely incorrect.
2. **Check QoS Type**: Run the following command to check the QoS type of the subscribed PointCloud2 message:

```sh
ros2 topic info /TPCK_LIDAR_TOP_CENTER --verbose
```
For example, here is what you may get
```
ros2 topic info /TPCK_LIDAR_TOP_CENTER --verbose
Type: sensor_msgs/msg/PointCloud2

Publisher count: 1

Node name: _ros2cli_rosbag2
Node namespace: /
Topic type: sensor_msgs/msg/PointCloud2
Endpoint type: PUBLISHER
GID: 01.0f.70.b7.f3.7f.cb.0a.01.00.00.00.00.00.20.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
  Durability: RMW_QOS_POLICY_DURABILITY_VOLATILE
  Lifespan: 2147483651294967295 nanoseconds
  Deadline: 2147483651294967295 nanoseconds
  Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
  Liveliness lease duration: 2147483651294967295 nanoseconds

Subscription count: 1

Node name: _ros2cli_18508
Node namespace: /
Topic type: sensor_msgs/msg/PointCloud2
Endpoint type: SUBSCRIPTION
GID: 01.0f.70.b7.4c.48.eb.79.01.00.00.00.00.00.05.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
  Durability: RMW_QOS_POLICY_DURABILITY_VOLATILE
  Lifespan: 2147483651294967295 nanoseconds
  Deadline: 2147483651294967295 nanoseconds
  Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
  Liveliness lease duration: 2147483651294967295 nanoseconds
```

Adjust the QoS settings in your launch configuration or node parameters accordingly.

### Example `.vscode/launch.json`

Below is an example of how to specify arguments in your `launch.json` file:

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "ROS2 Launch",
            "type": "cppdbg",
            "request": "launch",
            "program": "${workspaceFolder}/build/your_package_name/your_executable_name",
            "args": [
                "--ros-args",
                "-p", "topic_name:=/TPCK_LIDAR_TOP_CENTER",
                "-p", "output_dir:=${workspaceFolder}/saved",
                "-p", "qos_history:=10",
                "-p", "qos_reliability:=best_effort",
                "-p", "qos_durability:=volatile"
            ],
            "stopAtEntry": false,
            "cwd": "${workspaceFolder}",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                }
            ],
            "preLaunchTask": "build",
            "miDebuggerPath": "/usr/bin/gdb",
            "miDebuggerArgs": "",
            "customLaunchSetupCommands": [],
            "setupCommands": [],
            "launchCompleteCommand": "exec-continue",
            "logging": {
                "engineLogging": false,
                "trace": true,
                "traceResponse": true
            }
        }
    ]
}
```

Ensure you replace `your_package_name` and `your_executable_name` with the actual names used in your project.

## Notes

- Make sure the `topic_name` parameter is provided and not empty.
- Adjust the QoS settings according to the information retrieved from the `ros2 topic info` command.
- The default output directory is the current working directory unless specified otherwise.

