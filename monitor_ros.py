# /// script
# requires-python = ">=3.8"
# dependencies = [
#     "roslibpy",
# ]
# ///

import time
import roslibpy
import sys

def main():
    print("Connecting to ROS bridge at ws://localhost:9090...")
    # Configure the ROS bridge client
    client = roslibpy.Ros(host='localhost', port=9090)

    try:
        client.run()
        if client.is_connected:
            print("Successfully connected to ROS bridge!")
        else:
            print("Failed to connect to ROS bridge. Is the container running?")
            sys.exit(1)

        # Set up a subscriber for the joint states topic
        # The topic type is sensor_msgs/msg/JointState in ROS 2, roslibpy handles this
        print("Subscribing to /joint_states...")
        listener = roslibpy.Topic(client, '/joint_states', 'sensor_msgs/msg/JointState')

        def callback(message):
            if 'name' in message and 'position' in message:
                names = message['name']
                positions = message['position']
                print(f"Received Joint States: {len(names)} joints.")
                for name, pos in zip(names[:3], positions[:3]):
                    print(f"  {name}: {pos:.4f}")
                print("  ...")

        listener.subscribe(callback)

        print("Listening for 10 seconds to verify monitoring...")
        try:
            for _ in range(10):
                if not client.is_connected:
                    print("Connection lost.")
                    break
                time.sleep(1)
        except KeyboardInterrupt:
            print("Interrupted by user.")

        listener.unsubscribe()
        print("Unsubscribed. Disconnecting...")

    except Exception as e:
        print(f"Error connecting to ROS bridge: {e}")
    finally:
        if client.is_connected:
            client.terminate()

if __name__ == '__main__':
    main()
