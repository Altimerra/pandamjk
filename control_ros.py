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
    client = roslibpy.Ros(host='localhost', port=9090)

    try:
        client.run()
        if not client.is_connected:
            print("Failed to connect to ROS bridge.")
            sys.exit(1)
        print("Connected!")

        # Subscribe to joint states to get current position
        current_positions = None
        
        def js_callback(msg):
            nonlocal current_positions
            if 'name' in msg and 'position' in msg and len(msg['position']) >= 7:
                # Assuming the first 7 are arm joints, but let's be safe
                # Extract indices of joint1 to joint7
                pos_dict = dict(zip(msg['name'], msg['position']))
                try:
                    current_positions = [pos_dict[f'joint{i}'] for i in range(1, 8)]
                except KeyError:
                    # Might have fer_joint1 etc if no_prefix=false, but we expect bare joints
                    pass

        js_listener = roslibpy.Topic(client, '/joint_states', 'sensor_msgs/msg/JointState')
        js_listener.subscribe(js_callback)

        print("Waiting for joint states...")
        for _ in range(20):
            if current_positions is not None:
                break
            time.sleep(0.5)

        if current_positions is None:
            print("Failed to receive joint states.")
            sys.exit(1)

        print(f"Current positions: {current_positions}")
        js_listener.unsubscribe()

        # Publisher for forward position controller
        pub = roslibpy.Topic(client, '/forward_position_controller/commands', 'std_msgs/msg/Float64MultiArray')
        
        # Modify joint 4 (index 3) by 0.3 rad as a test, keeping others same
        target_positions = list(current_positions)
        target_positions[3] += 0.3

        print(f"Commanding new positions: {target_positions}")
        
        # Create Float64MultiArray message
        # In ROS 2 / roslibpy, Float64MultiArray has a layout and data array
        msg = {
            'layout': {
                'dim': [],
                'data_offset': 0
            },
            'data': target_positions
        }

        # Publish a few times to ensure it's received
        for _ in range(5):
            pub.publish(roslibpy.Message(msg))
            time.sleep(0.1)
        
        print("Command sent! Robot should have moved.")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if client.is_connected:
            client.terminate()

if __name__ == '__main__':
    main()
