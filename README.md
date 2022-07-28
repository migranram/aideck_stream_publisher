# aideck_stream_publisher
ROS2 Package to publish the wifi streamed images of the AI Deck of the Crazyflie.

It is based on the bitcraze wifi-stream example for the AI Deck. The original code can be found on their repository:
https://github.com/bitcraze/aideck-gap8-examples


## How to use:
```
ros2 launch aideck_stream_publisher aideck_stream.py ip:=192.168.2.21 port:=5000 show_flag:=True
```

To reconfigure the color balance point at a white object with the camera and press `b` in your keyboard.