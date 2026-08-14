# /// script
# requires-python = ">=3.9"
# dependencies = [
#     "roslibpy",
# ]
# ///

import roslibpy

print("Connecting to ROS bridge on ws://localhost:9090...")
client = roslibpy.Ros(host='localhost', port=9090)
client.run()

if client.is_connected:
    print("Connected! Fetching publishers for /joint_states...\n")
    try:
        service = roslibpy.Service(client, '/rosapi/publishers', 'rosapi/Publishers')
        request = roslibpy.ServiceRequest({'topic': '/joint_states'})
        result = service.call(request)
        
        publishers = result.get('publishers', [])
        if publishers:
            print(f"✅ Found {len(publishers)} nodes publishing to /joint_states:")
            for pub in publishers:
                print(f"   - {pub}")
        else:
            print("❌ No nodes are currently publishing to /joint_states!")
    except Exception as e:
        print(f"Error calling /rosapi/publishers: {e}")
    
    client.terminate()
else:
    print("❌ Failed to connect to rosbridge on localhost:9090. Is rosbridge running inside the container?")
