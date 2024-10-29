# `nav2_port_drayage_demo` package

This package integrates Port Drayage operations with a Nav2-based autonomy stack. The package's node listens for
incoming port drayage messages of type `carma_v2x_msgs/msg/MobilityOperation` and then sends the target location to the Nav2 Waypoint
Follower via a ROS 2 action call.

## Sending Mobility Operation Messages
The definition of the `carma_v2x_msgs/msg/MobilityOperation` is as follows:

```
# MobilityOperation.msg

# This message is used by plugins to send necessary parameters to other
# CAVs at any desired frequency in order to accomplish a collaborative plan

# standard header for all mobility messages
carma_v2x_msgs/MobilityHeader  m_header
	string  sender_id
	string  recipient_id
	string sender_bsm_id
	string  plan_id
	uint64  timestamp

# strategy string to indicate which strategy type this message is used for
string                   strategy

# strategy parameters specified by each individual plugin/component
# the maximum length of this string is 100
string                   strategy_params
```

For the purpose of this demo, the only necessary fields to populate when sending instructions to the vehicle are `strategy` and `strategy_params`. The `strategy` field should always be set to `port_drayage`. The `strategy_params` field should be set to a JSON string with the following format:

```
// Example Mobility Operation strategy_params JSON payload with message's strategy set to "port_drayage":
{
  "cmv_id": "DOT-80550",                // [Required from all] string unique identifier for CMV 
  "operation": "ENTERING_STAGING_AREA", // [Required from all] Enum to indicate the type of action
                                        // NOTE: Possible operations include: PICKUP, DROPOFF, PORT_CHECKPOINT, HOLDING_AREA, 
                                        //                                    ENTER_STAGING_AREA, EXIT_STAGING_AREA, ENTER_PORT, EXIT_PORT
  "cargo": false,                       // [Required from all with PICKUP/DROPOFF operation] boolean flag to indicate whether the CMV is loaded with cargo
  "cargo_id": "SOME_CARGO",             // [Required from all with PICKUP/DROPOFF operation] string unique identifier for cargo
  "location": {                         // [Required from CMV] current location of the CMV
    "longitude": 0, 
    "latitude": 0}, 
  "destination": {                      // [Required from infrastructure] optional destination for CMV
    "longitude": 0,
    "latitude": 0},
  "action_id": "SOMEUID"                // [Required from all] string UUID to identify action
  }
```


For example, to send a Mobility Operation Message instructing the Turtlebot to pick up cargo with the ID "`CARGO_A`" at `(x,y)` coordinates `(3.8, 0.5)` using the ROS2 CLI, run the command:

```
ros2 topic pub --once /incoming_mobility_operation carma_v2x_msgs/msg/MobilityOperation "m_header:
  sender_id: ''
  recipient_id: ''
  sender_bsm_id: ''
  plan_id: ''
  timestamp: 0
strategy: 'port_drayage'
strategy_params: '{\"cmv_id\":\"turtlebot\",\"operation\":\"PICKUP\",\"cargo\":false,\"cargo_id\":\"CARGO_A\",\"destination\":{\"longitude\":3.8,\"latitude\":0.5},\"action_id\":\"PORT_DRAYAGE\"}'"
```

## Turtlebot Gazebo Launch Instructions

To run the Port Drayage demo using the Turtlebot in the Gazebo simulator, first follow the [cda1tenth_bringup Prerequisites](https://github.com/usdot-fhwa-stol/cda1tenth_bringup?tab=readme-ov-file#prerequisites) to download and build the necessary software.

Then, download and run the simulation environment using:

```
sudo apt install ros-humble-turtlebot3*
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

To launch the Port Drayage demo, run 
```
ros2 launch cda1tenth_bringup cda1tenth_bringup_launch.xml vehicle:=turtlebot
```

Next, provide the Turtlebot with an initial pose estimate using the 2D Pose Estimate arrow in Rviz. Once the vehicle has localized, it is now ready to
receive a Mobility Operation message to determine an action to complete. To start a simple demonstration of the Turtlebot picking up and dropping off cargo, run:

```
cd nav2_port_drayage_demo/
./test/turtlebot_port_drayage_test.sh
```

This will publish a series of Mobility Operation messages on `/incoming_mobility_operation` that instruct the Turtlebot to navigate to the top of the map to pickup cargo and then subsequently navigate to the bottom of the map to drop it off. After each successful operation, a Mobility Operation message acknowledging the Turtlebot completed the desired action will be published on `/outgoing_mobility_operation`.
