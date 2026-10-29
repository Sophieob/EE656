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
