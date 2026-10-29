# Gambler: RX200 Card-Dealing Robot

## Project overview

Gambler is a ROS 2 card-dealing project built around the Interbotix RX200 robotic arm. The operator provides pickup and drop-off table coordinates through a command-line interface (CLI). The robot moves to the pickup position, waits for the operator to position the card between the gripper fingers, then picks it up and moves it to the requested drop-off position. A 3D-printed card block measuring 88.9 mm × 63.5 mm × 25 mm gives the gripper a thicker object to hold because a standard playing card is too thin for a reliable grasp.

The workspace contains two ROS 2 packages:

- `rx200_dealer_client`: Python node, CLI, and launch file.
- `rx200_dealer_interfaces`: custom ROS 2 service definitions used by the CLI and robot node.

Build the workspace with `colcon`. The launch file starts the RX200 MoveIt stack and the dealer node; it uses real hardware by default. The fake hardware option launches the simulated robot.

To launch robot:

```sh  
ros2 launch rx200_dealer_client dealer.launch.py
```

To launch simulated robot:

```sh
ros2 launch rx200_dealer_client dealer.launch.py hardware_type:=fake
```

To deal a card, run the CLI's `deal` subcommand with the pick up and drop off locations as `x,y` table coordinates:

```sh
ros2 run rx200_dealer_client cli deal 0.2,0.1 0.3,-0.1
```

The arm first points to the pick up location. Place the card between the gripper's fingers, then press enter to deal it (or type `c` to cancel).
