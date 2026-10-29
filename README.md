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
Keep this launch running while using the CLI. The CLI checks for the dealer services at startup and exits with a message if they have not appeared within three seconds.

## Deal a card

In a second terminal, source the ROS 2 installation and workspace overlay, then run the `deal` command:

```sh
ros2 run rx200_dealer_client cli deal 0.2,0.1 0.3,-0.1
```

The command takes two coordinates:

```text
cli deal <PICKUP_X,PICKUP_Y> <DROPOFF_X,DROPOFF_Y>
```

Each coordinate is written as `x,y`, with `x` first and `y` second. Values are in metres relative to the robot base frame. Use a comma between the values and no spaces. Negative values are allowed; for example, `0.3,-0.1` means x = 0.3 m and y = -0.1 m.

The node's default software limits for x and y are -0.35 m to 0.35 m. A coordinate inside those limits is not necessarily reachable for every arm pose, so use positions verified for the robot and table setup.

The CLI first sends the robot to the pickup position. When it reaches the position, place the card block between the gripper fingers and press Enter. Type `c` or `cancel` to cancel before the pick-and-place operation begins. The robot then closes the gripper, lifts from pickup, moves above the drop-off point, lowers the card, opens the gripper, and retreats.

Team members

- Jason Rogena
- Sophie O’Brien
- Janam Pandya
