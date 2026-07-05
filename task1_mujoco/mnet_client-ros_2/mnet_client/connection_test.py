#! /usr/bin/env python3
# This is the entry script for connection test with the server and check the qualification of the team
# DO NOT MODIFY THIS FILE
# Contact: support@manipulation-net.org

import os
import json
import time
import rclpy
import socket
import struct
from rclpy.node import Node
from mnet_client.base import (
    PACKAGE_NAME,
    SERVER_IP,
    SERVER_PORT,
    LoginRequest,
    LoginResponse,
    HEADER_FMT,
    HEADER_SIZE,
)
from ament_index_python.packages import get_package_share_directory


class ConnectionTest(Node):
    def __init__(self):
        super().__init__("connection_test")
        try:
            package_path = get_package_share_directory(PACKAGE_NAME)
            print(f"Package path: {package_path}")

            # Build config file path
            config_file = os.path.join(package_path, "config", "team_config.json")
            banner_file = os.path.join(package_path, "config", "banner.txt")
            # Check if file exists
            if os.path.exists(config_file):
                with open(config_file, "r") as f:
                    team_config = json.load(f)
                print(f"Loaded config from: {config_file}")
            else:
                print(f"Config file not found: {config_file}")
            os.system("clear")

            # Check if banner exists
            if os.path.exists(banner_file):
                with open(banner_file, "r") as f:
                    banner = f.read()
                print(banner)
            else:
                print(f"Banner loading error: {banner_file}")

        except Exception as e:
            print(f"Failed to get package path: {e}")
            exit()

        self.server_port = SERVER_PORT
        self.server_ip = SERVER_IP
        self.team_unique_code = team_config["team_unique_code"]
        self.autonomy_level = team_config["autonomy_level"]

        self.print_box(
            f"[NOTICE] This is ONLY for connection test and qualification check. "
        )
        self.print_box("No task submission is involved.")
        print(f"Team Unique Code: {self.team_unique_code}")
        print(
            f"Mode: {'Teleop' if self.autonomy_level==0 else 'Human-in-the-loop' if self.autonomy_level==1 else 'Autonomous'}"
        )
        print(
            f"Connecting to Server IP: {self.server_ip}, Server Port: {self.server_port}"
        )

        # Initialize socket
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.settimeout(5)
        try:
            self.client_socket.connect((self.server_ip, self.server_port))
        except socket.error as e:
            self.print_box(f"Connection failed: {e}")
            return
        self.test_team_qualification()

    def test_team_qualification(self):
        """
        Test if the team qualification is successful
        """
        start_time = time.time()
        self.send_login_request()
        response = self.receive_login_response()
        if response is None or not response.success:
            self.print_box(f"Connection failed: Failed to receive message from server.")
            return

        self.print_box("Connected to server successfully")
        print("Server response:")
        self.print_box(response.message)
        end_time = time.time()
        rtt = (end_time - start_time) * 1000  # Convert to milliseconds
        print(
            f"Estimated Round Trip Time (RTT) between client and server connection: {rtt} ms"
        )

    def print_box(self, text):
        length = len(text)
        print("┌" + "─" * (length + 2) + "┐")
        print("│ " + text + " │")
        print("└" + "─" * (length + 2) + "┘")

    def send_login_request(self):
        """
        Send a request as JSON with a 4-byte length prefix
        """
        try:
            payload = (
                LoginRequest(
                    type="login_request",
                    team_unique_code=self.team_unique_code,
                    autonomy_level=self.autonomy_level,
                    connection_test=True,
                )
                .model_dump_json()
                .encode("utf-8")
            )
            length_prefix = struct.pack(
                HEADER_FMT, len(payload)
            )  # 4-byte, network order
            self.client_socket.sendall(length_prefix + payload)
        except socket.error as e:
            print(f"Failed to send request: {e}")
            return False
        return True

    def _recvall(self, n: int) -> bytes:
        """
        Keep reading from the socket until the specified number of bytes are received
        """
        data = bytearray()
        while len(data) < n:
            chunk = self.client_socket.recv(n - len(data))
            if not chunk:
                return None
            data.extend(chunk)
        return bytes(data)

    def receive_login_response(self):
        """
        Receive a length-prefixed JSON response and parse it
        """
        try:
            raw_length = self._recvall(HEADER_SIZE)
            if not raw_length:
                print("Failed to receive message length prefix")
                return None
            msg_length = struct.unpack(HEADER_FMT, raw_length)[0]
            raw_data = self._recvall(msg_length)
            if not raw_data:
                print("Failed to receive data stream from the server")
                return None

            data = raw_data.decode("utf-8")
            return LoginResponse.model_validate_json(data)
        except socket.error as e:
            print(f"Failed to receive response: {e}")
            return None


def main():
    """
    Main function to run the connection test
    """
    try:
        rclpy.init()
        connection_test = ConnectionTest()
    except Exception as e:
        print(f"Error: {e}")
        return


if __name__ == "__main__":
    main()
