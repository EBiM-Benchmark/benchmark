#! /usr/bin/env python3
# This is the entry script for local test
# It is designed to experience the submission protocol locally without connecting to the server
# DO NOT MODIFY THIS FILE
# Contact: support@manipulation-net.org


try:
    import rclpy
    from mnet_client.clients import LocalTestClient
except Exception as e:
    print(f"Error importing modules: {e}")
    print("Please ensure all required modules are installed and properly configured.")
    exit()


def main():
    """
    Main function to run the local test client
    """
    try:
        rclpy.init()
        client = LocalTestClient()
        client.run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
