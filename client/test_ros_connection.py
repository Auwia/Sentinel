import roslibpy

def main():
    # Update the host and port to match your ROS2 master
    ros = roslibpy.Ros(host='192.168.221.127', port=9090)  # Replace with your PC's IP
    
    print('Connecting to ROS master...')
    ros.run()

    if ros.is_connected:
        print('Successfully connected to ROS master!')
    else:
        print('Failed to connect to ROS master.')

    ros.terminate()

if __name__ == '__main__':
    main()
