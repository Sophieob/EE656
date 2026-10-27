import rclpy
import cmd
from enum import Enum
from rclpy.node import Node
from rx200_dealer_interfaces.srv import PickAndPlaceCard, PointToPosition
from rx200_dealer_client.rx200_dealer_client_node import PICK_AND_PLACE_SRV_NAME, POINT_TO_POSITION_SRV_NAME, TableCoordinates

SRV_TIMEOUT_RETRIES = 6
SRV_TIMEOUT_SECS = 10

class ServicesClient:
    def __init__(self, node):
        self.node = node
        self.pick_and_place_client = node.create_client(PickAndPlaceCard, PICK_AND_PLACE_SRV_NAME)
        self._wait_for_service(self.pick_and_place_client)

        self.point_to_position_client = node.create_client(PointToPosition, POINT_TO_POSITION_SRV_NAME)
        self._wait_for_service(self.point_to_position_client)

    def _wait_for_service(self, client):
        wait_count = 0
        print(f"Waiting for the {client.srv_name} service", end="", flush=True)
        while not client.wait_for_service(timeout_sec=SRV_TIMEOUT_SECS):
            print(".", end="", flush=True)
            wait_count += 1
            if wait_count > SRV_TIMEOUT_RETRIES:
                print()
                raise RuntimeError(f"The {client.srv_name} service was not available after {wait_count * SRV_TIMEOUT_SECS}s. Is the dealer node running?")
        print(" ready")

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


class ShellPrompt(Enum):
    NONE = 0
    START = 1
    MOVE = 2


class DealerShell(cmd.Cmd):
    intro = 'Welcome to the dealer shell. Type help or ? to list commands.\n'
    prompt = '(dealer) '
    last_prompt = ShellPrompt.NONE

    def __init__(self, srv_client):
        super().__init__()
        self.srv_client = srv_client
        self.start_coords = None

    def reset(self):
        self.last_prompt = ShellPrompt.NONE

    def precmd(self, line):
        return line if line == 'EOF' else line.lower()

    def parse_arg(self, arg):
        position = arg.strip()
        x_y_str = position.split(',')
        if len(x_y_str) != 2:
            raise ValueError(f"'{position}' is not a valid location. Use the format 'x,y' where x and y are floats")
        try:
            x = float(x_y_str[0].strip())
            y = float(x_y_str[1].strip())
        except ValueError:
            raise ValueError(f"'{position}' is not a valid location. Use the format 'x,y' where x and y are floats")

        return TableCoordinates(x, y)

    def default(self, line):
        if line == 'EOF':
            print()
            return True

        print(f"Unknown command '{line}'. Type help or ? to list commands")

    def emptyline(self):
        # cmd.Cmd re-runs the last command by default, which would move the arm again
        pass

    def do_start(self, arg):
        'Positions the gripper over the pick up location: start <x,y>'
        try:
            start_coords = self.parse_arg(arg)
        except ValueError as ve: # Something wrong with the input
            print(f"An error occurred while processing the input provided. Try again: {ve}")
            return False
        try:
            self.srv_client.point_to_position(start_coords)
        except ValueError as ve: # Something wrong with the input
            print(f"An error occurred while servicing your request: {ve}")
            return False
        except RuntimeError as re:
            print(f"A runtime error was thrown. Exiting: {re}")
            self.reset()
            return True

        print("Place the card between the gripper's left and right fingers. Once that is done, call 'move <x,y>' and press enter")
        self.last_prompt = ShellPrompt.START
        self.start_coords = start_coords

    def do_move(self, arg):
        'Moves the card from the start location to another location: move <x,y>'
        if self.last_prompt != ShellPrompt.START or self.start_coords is None:
            print("First run the start command")
            return False

        try:
            end_coords = self.parse_arg(arg)
        except ValueError as ve: # Something wrong with the input
            print(f"An error occurred while processing the input provided. Try again: {ve}")
            return False

        try:
            self.srv_client.pick_and_place(self.start_coords, end_coords)
        except ValueError as ve: # The dealer node rejected the request
            print(f"An error occurred while servicing your request: {ve}")
            return False
        except RuntimeError as re:
            print(f"A runtime error was thrown. Exiting: {re}")
            self.reset()
            return True

        print("Once movement is done, run the start command to deal another card")
        self.last_prompt = ShellPrompt.MOVE
        self.start_coords = None

    def do_quit(self, arg):
        'Exits the dealer shell'
        print("Bye!")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = Node('rx200_dealer_cli')
    try:
        srv_client = ServicesClient(node)
        DealerShell(srv_client).cmdloop()
    except KeyboardInterrupt:
        print("\nBye!")
    except RuntimeError as re:
        print(f"Error: {re}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
