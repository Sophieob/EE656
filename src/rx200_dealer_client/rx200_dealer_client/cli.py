import argparse
import re
import sys
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from rx200_dealer_interfaces.srv import PickAndPlaceCard, PointToPosition
from rx200_dealer_client.rx200_dealer_client_node import PICK_AND_PLACE_SRV_NAME, POINT_TO_POSITION_SRV_NAME, TableCoordinates

SRV_TIMEOUT_SECS = 10

class ServicesClient:
    def __init__(self, node):
        self.node = node
        self.pick_and_place_client = node.create_client(PickAndPlaceCard, PICK_AND_PLACE_SRV_NAME)
        self._wait_for_service(self.pick_and_place_client)

        self.point_to_position_client = node.create_client(PointToPosition, POINT_TO_POSITION_SRV_NAME)
        self._wait_for_service(self.point_to_position_client)

    def _wait_for_service(self, client):
        discovery_timeout_sec = 3.0
        check_interval_sec = 0.2
        elapsed_sec = 0.0

        while elapsed_sec < discovery_timeout_sec:
            if client.wait_for_service(timeout_sec=check_interval_sec):
                return
            elapsed_sec += check_interval_sec

        if not client.wait_for_service(timeout_sec=0.0):
            raise RuntimeError(
                "MoveIt/dealer services were not discovered within "
                f"{discovery_timeout_sec:.0f} seconds. "
                "Launch MoveIt and the RX200 dealer node first, then run the CLI again."
            )

    def pick_and_place(self, start_coords: TableCoordinates, end_coords: TableCoordinates):
        request = PickAndPlaceCard.Request(start_x=start_coords.x, start_y=start_coords.y, end_x=end_coords.x, end_y=end_coords.y)
        future = self.pick_and_place_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_TIMEOUT_SECS)

        if not future.done():
            raise RuntimeError(f'No response from {PICK_AND_PLACE_SRV_NAME} within {SRV_TIMEOUT_SECS}s')

        response = future.result()
        if response.accepted:
            print(f'Accepted: ({start_coords.x}, {start_coords.y}) -> ({end_coords.x}, {end_coords.y})')
        else:
            raise ValueError(f'Rejected: {response.error}')

    def point_to_position(self, coords: TableCoordinates):
        request = PointToPosition.Request(position_x=coords.x, position_y=coords.y)
        future = self.point_to_position_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_TIMEOUT_SECS)

        if not future.done():
            raise RuntimeError(f'No response from {POINT_TO_POSITION_SRV_NAME} within {SRV_TIMEOUT_SECS}s')

        response = future.result()
        if response.accepted:
            print(f'Pointing to position ({coords.x}, {coords.y})')
        else:
            raise ValueError(f'Rejected: {response.error}')


def parse_coordinates(arg):
    position = arg.strip()
    x_y_str = position.split(',')
    if len(x_y_str) != 2:
        raise argparse.ArgumentTypeError(f"'{position}' is not a valid location. Use the format 'x,y' where x and y are floats")
    try:
        x = float(x_y_str[0].strip())
        y = float(x_y_str[1].strip())
    except ValueError:
        raise argparse.ArgumentTypeError(f"'{position}' is not a valid location. Use the format 'x,y' where x and y are floats")

    return TableCoordinates(x, y)


def deal(srv_client, start_coords: TableCoordinates, end_coords: TableCoordinates):
    srv_client.point_to_position(start_coords)

    try:
        answer = input("Wait for the sequence to complete, then Place the card between the gripper's left and right fingers, then press enter to deal it (or type 'c' to cancel): ")
    except EOFError:
        answer = 'c'
    if answer.strip().lower() in ('c', 'cancel'):
        print("Deal cancelled")
        return

    srv_client.pick_and_place(start_coords, end_coords)


def build_parser():
    parser = argparse.ArgumentParser(prog='cli', description='Control the RX200 card dealer')
    subparsers = parser.add_subparsers(dest='command', required=True)

    deal_parser = subparsers.add_parser(
        'deal',
        help='Deal a card from one location to another',
        description='Points to the start location, waits for you to confirm the card is in the gripper, then moves it to the end location')
    # Treat coords with negative x (e.g. '-0.1,0.2') as positional arguments rather than options
    deal_parser._negative_number_matcher = re.compile(r'^-\d*\.?\d+(,|$)')
    deal_parser.add_argument('start', type=parse_coordinates, metavar='START', help="pick up location as 'x,y'")
    deal_parser.add_argument('end', type=parse_coordinates, metavar='END', help="drop off location as 'x,y'")

    return parser


def main(args=None):
    # Parse before starting ROS so that bad input fails without waiting for services
    parsed = build_parser().parse_args(remove_ros_args(args if args is not None else sys.argv)[1:])

    rclpy.init(args=args)
    node = Node('rx200_dealer_cli')
    exit_code = 0
    try:
        srv_client = ServicesClient(node)
        if parsed.command == 'deal':
            deal(srv_client, parsed.start, parsed.end)
    except KeyboardInterrupt:
        print()
        exit_code = 130 # Exit code indicating SIGINT
    except (ValueError, RuntimeError) as ve:
        print(f"Error: {ve}", file=sys.stderr)
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
