To launch robot:

```sh  
ros2 launch rx200_dealer_client dealer.launch.py
```

To launch simulated robot:

```sh
ros2 launch rx200_dealer_client dealer.launch.py hardware_type:=fake
```

Before the CLI is created, you can change the start and end positions by running `ros2 run` directly on the dealer node. Possible values for the start and end position parameters are (`cc`, `player1`, and `player2`):

```sh
ros2 run rx200_dealer_client rx200_dealer --ros-args -p start_position:=cc -p end_position:=player2
```
