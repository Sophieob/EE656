To launch robot:

```sh  
ros2 launch rx200_dealer_client dealer.launch.py
```

To launch simulated robot:

```sh
ros2 launch rx200_dealer_client dealer.launch.py hardware_type:=fake
```

Use the CLI to make the dealer move a card from the community cards to player 1 using the command:

```sh
ros2 run rx200_dealer_client cli move cc player1
```

Use the CLI to make the dealer move a card from the community cards to player 2 using the command:

```sh
ros2 run rx200_dealer_client cli move cc player2
```

The CLI currently accepts the following locations; `cc`, `player1` and `player2`.

Temporary Commands for Point & Move:
To point to community cards position:

```sh
ros2 service call /point_to_position rx200_dealer_interfaces/srv/PointToPosition "{position: 'cc'}"
```

To move from community cards to player1:

```sh
ros2 service call /pick_and_place_card rx200_dealer_interfaces/srv/PickAndPlaceCard "{start_position: 'cc', end_position: 'player1'}"
```
