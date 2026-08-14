# /// script
# requires-python = ">=3.9"
# dependencies = [
#     "roslibpy",
# ]
# ///

import roslibpy
import time

def listener_callback(msg):
    positions = msg.get('position', [])
    names = msg.get('name', [])
    
    if not hasattr(listener_callback, "first"):
        print(f"✅ Connection successful! Receiving messages with {len(names)} joints.")
        listener_callback.first = True
        
    if len(positions) == 10:
        print("\n*** 🚨 FOUND MESSAGE WITH 10 POSITIONS! 🚨 ***")
        print(f"Names: {names}")
        print(f"Positions: {positions}")

print("Connecting to ROS bridge on ws://localhost:9090...")
client = roslibpy.Ros(host='localhost', port=9090)
client.run()

if client.is_connected:
    print("Connected! Subscribing to /joint_states...")
    listener = roslibpy.Topic(client, '/joint_states', 'sensor_msgs/JointState')
    listener.subscribe(listener_callback)

    try:
        while client.is_connected:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    
    listener.unsubscribe()
    client.terminate()
else:
    print("❌ Failed to connect to rosbridge on localhost:9090. Make sure the Docker container is mapping port 9090 to the host!")
