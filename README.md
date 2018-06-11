# WebsocketROSClientPython
WebsocketROSClientPython is a lightweight ROS package and Python library to messaging between the machines with ROS via websocket. You can subscribe and publish and call service messages between rosbridge server and client.


Data is transmitted in JSON format and automatically converted into ROS messages using the library [rospy_message_converter](https://github.com/baalexander/rospy_message_converter) 

#### Server: ROS --> Bridge: JSON <-- Client: ROS/MATLAB/UNITY

Support:
* Publish
* Subscribe (messages are immediately published in the specified topic)
* Service (call only, the response didn't parse)


**Note that this needs install:**
  ```
  sudo pip install websocket-client
  sudo apt install ros-$ROS_DISTRO-rospy-message-converter
  sudo apt install ros-$ROS_DISTRO-rosbridge-server
  ```

**Stage:** In the development



Usage
-----

Init websocket client:

``` python
    from WebsocketROSClient import WebsocketROSClient as ros_ws
    connect = ros_ws('128.0.0.1', 9090) # ip, port, name of client
 ```
Subscribe to topics from server:

``` python
    from geometry_msgs.msg import  PoseStamped
    # 1. The name of the topic server;
    # 2. The type of topic
    # 3. The name of the topic where to publish
    connect.subscribe('/position', PoseStamped(), '/position')
```
Publish to server:

``` python
    
    from std_msgs.msg import String
    msg = String()
    msg.data = "Test message"
    # 1. the name of the topic on server where to publish; 
    # 2. the message
    connect.publish("/client_topic",msg)
```

Call service from server
```
   # 1. the name of the service on server where to call;
   # 2. the service request message 
   connect.call_service('/ropic_of_service', srv_pub)
```
