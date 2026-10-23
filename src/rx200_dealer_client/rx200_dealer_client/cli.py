import argparse
import sys
import rclpy
from rclpy.node import Node
from rx200_dealer_interfaces.srv import PickAndPlaceCard
from rx200_dealer_client.rx200_dealer_client_node import TablePosition

PICK_AND_PLACE_SRV_NAME = 'pick_and_place_card'
POSITIONS = [p.value for p in TablePosition]


def pick_and_place(node, start, end, timeout):
    client = node.create_client(PickAndPlaceCard, PICK_AND_PLACE_SRV_NAME)
    while not client.wait_for_service(timeout_sec=timeout):
        pass

    request = PickAndPlaceCard.Request(start_position=start, end_position=end)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)

    if not future.done():
        raise SystemExit(f'No response from {PICK_AND_PLACE_SRV_NAME} within {timeout}s')

    response = future.result()
    if response.accepted:
        print(f'Accepted: {start} -> {end}')
    else:
        raise SystemExit(f'Rejected: {response.error}')


def main(args=None):
    parser = argparse.ArgumentParser(prog='rx200_dealer_cli', description='Control the RX200 dealer')
    parser.add_argument('--timeout', type=float, default=5.0, help='seconds to wait for the service')
    subparsers = parser.add_subparsers(dest='command', required=True)

    move = subparsers.add_parser('move', help='Pick a card up from one position and place it at another')
    move.add_argument('start', choices=POSITIONS)
    move.add_argument('end', choices=POSITIONS)

    parsed = parser.parse_args(rclpy.utilities.remove_ros_args(args if args is not None else sys.argv)[1:])

    rclpy.init(args=args)
    node = Node('rx200_dealer_cli')
    try:
        if parsed.command == 'move':
            pick_and_place(node, parsed.start, parsed.end, parsed.timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
