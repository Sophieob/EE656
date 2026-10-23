import rclpy
import cmd
from enum import Enum
from rclpy.node import Node
from rx200_dealer_interfaces.srv import PickAndPlaceCard, PointToPosition
from rx200_dealer_client.rx200_dealer_client_node import TablePosition, PICK_AND_PLACE_SRV_NAME, POINT_TO_POSITION_SRV_NAME

POSITIONS = [p.value for p in TablePosition]

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
        print(f"Waiting for client for the {client.srv_name} service to initialize")
        while not client.wait_for_service(timeout_sec=SRV_TIMEOUT_SECS):
            print(".", end="")
            wait_count += 1
            if wait_count > SRV_TIMEOUT_RETRIES:
                raise RuntimeError("Timed out waiting for a service")

    def pick_and_place(self, start, end):
        request = PickAndPlaceCard.Request(start_position=start, end_position=end)
        future = self.pick_and_place_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_TIMEOUT_SECS)

        if not future.done():
            raise RuntimeError(f'No response from {PICK_AND_PLACE_SRV_NAME} within {SRV_TIMEOUT_SECS}s')

        response = future.result()
        if response.accepted:
            print(f'Accepted: {start} -> {end}')
        else:
            raise ValueError(f'Rejected: {response.error}')

    def point_to_cc(self):
        request = PointToPosition.Request(position=TablePosition.CC.value)
        future = self.point_to_position_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=SRV_TIMEOUT_SECS)

        if not future.done():
            raise RuntimeError(f'No response from {POINT_TO_POSITION_SRV_NAME} within {SRV_TIMEOUT_SECS}s')

        response = future.result()
        if response.accepted:
            print(f'Pointing to community cards')
        else:
            raise ValueError(f'Rejected: {response.error}')


class ShellPrompt(Enum):
    NONE = 0
    POINT_CC = 1
    PICK_AND_PLACE_CARD = 2


class DealerShell(cmd.Cmd):
    intro = 'Welcome to the dealer shell. Type help or ? to list commands.\n'
    prompt = '(dealer) '
    last_prompt = ShellPrompt.NONE

    def __init__(self, srv_client):
        super().__init__()
        self.srv_client = srv_client

    def reset(self):
            self.last_prompt = ShellPrompt.NONE

    def precmd(self, line):
        return line if line == 'EOF' else line.lower()

    def parse_arg(self, arg):
        position = arg.strip()
        if position not in POSITIONS:
            raise ValueError(f"'{position}' is not a valid location. Use one of: {', '.join(POSITIONS)}")
        return position

    def default(self, line):
        if line == 'EOF':
            print()
            return True

        print(f"Unkown command {line}")

    def do_point(self, arg):
        'Points to the community cards position on the table'
        try:
            self.srv_client.point_to_cc()
        except ValueError as ve: # Something wrong with the input
            print(f"An error occurred while servicing your request: {ve}")
            return False
        except RuntimeError as re:
            print(f"A runtime error was thrown. Exiting: {re}")
            self.reset()
            return True

        print("Place the card between the gripper's left and right fingers. Once that is done, call 'move <name of location>' and press enter")
        self.last_prompt = ShellPrompt.POINT_CC

    def do_move(self, arg):
        'Moves the card from the community cards position to a location: move <player1|player2>'
        if self.last_prompt == ShellPrompt.NONE:
            print("First run the point command")
            return False

        start_position = TablePosition.CC.value
        try:
            end_position = self.parse_arg(arg)
        except ValueError as ve: # Something wrong with the input
            print(f"An error occurred while processing the input provided. Try again: {ve}")
            return False

        try:
            self.srv_client.pick_and_place(start_position, end_position)
        except ValueError as ve: # The dealer node rejected the request
            print(f"An error occurred while servicing your request: {ve}")
            return False
        except RuntimeError as re:
            print(f"A runtime error was thrown. Exiting: {re}")
            self.reset()
            return True

        print("Once movement is done, re-run either the move or point commands")
        self.last_prompt = ShellPrompt.PICK_AND_PLACE_CARD

    def do_quit(self, arg):
        'Exits the dealer shell'
        print("Bye!")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = Node('rx200_dealer_cli')
    srv_client = ServicesClient(node)
    DealerShell(srv_client).cmdloop()


if __name__ == '__main__':
    main()
