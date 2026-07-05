#! /usr/bin/env python3
# This is the entry script for performance submission
# It is used to record and send the performance video to the server
# DO NOT MODIFY THIS FILE
# Contact: support@manipulation-net.org


try:
    import rclpy
    from mnet_client.clients import SubmissionClient
except Exception as e:
    print(f"Error importing modules: {e}")
    print("Please ensure all required modules are installed and properly configured.")
    exit()


def main():
    """
    Main function to initialize the client
    """
    try:
        rclpy.init()
        client = SubmissionClient()
        client.run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
