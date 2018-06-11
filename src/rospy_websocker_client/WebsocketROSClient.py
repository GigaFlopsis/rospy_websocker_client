#!/usr/bin/env python
# coding: utf-8

# Note that this needs:
# sudo pip install websocket-client
# sudo ros-$ROS_DISTRO-rospy-message-converter


import json
from uuid import uuid4
import websocket
import yaml
from threading import Thread
import time
import sys

import rospy
from rospy_message_converter import message_converter
from rospy import Header


"""

"""


class WebsocketROSClient(Thread, object):
    def __init__(self, websocket_ip, port=9090, name =''):
        """
        Class to manage publishing to ROS thru a rosbridge websocket.
        :param str websocket_ip: IP of the machine with the rosbridge server.
        :param int port: Port of the websocket server, defaults to 9090.
        """
        Thread.__init__(self)

        print("Connecting to websocket: {}:{}".format(websocket_ip, port))

        self.name = websocket_ip if name == '' else name
        print(self.name)

        self.initFlaf = False
        self.runFlag = True
        # List for for each sub topic.
        # Where: key = sub_topic; data = [type_data, pub_topic_name,rospy.Publisher(..)]
        self.sub_list = {}

        self.ws = websocket.create_connection(
            'ws://' + websocket_ip + ':' + str(port))
        self._advertise_dict = {}

        self.daemon = True
        self.start()

    def run(self):
        """Run of thread"""
        # print("ok")
        while self.runFlag:
            self._callback()

    def _advertise(self, topic_name, topic_type):
        """
        Advertise a topic with it's type in 'package/Message' format.
        :param str topic_name: ROS topic name.
        :param str topic_type: ROS topic type, e.g. std_msgs/String.
        :returns str: ID to de-advertise later on.
        """
        new_uuid = str(uuid4())
        self._advertise_dict[new_uuid] = {'topic_name': topic_name,
                                          'topic_type': topic_type}
        advertise_msg = {"op": "advertise",
                         "id": new_uuid,
                         "topic": topic_name,
                         "type": topic_type
                         }
        self.ws.send(json.dumps(advertise_msg))
        return new_uuid

    def _unadvertise(self, uuid):
        unad_msg = {"op": "unadvertise",
                    "id": uuid,
                    # "topic": topic_name
                    }
        self.ws.send(json.dumps(unad_msg))

    def __del__(self):
        """Cleanup all advertisings"""
        d = self._advertise_dict
        for k in d:
            self._unadvertise(k)
        self.runFlag = False
        self.join()
        print("stop")

    def _publish(self, topic_name, message):
        """
        Publish onto the already advertised topic the msg in the shape of
        a Python dict.
        :param str topic_name: ROS topic name.
        :param dict msg: Dictionary containing the definition of the message.
        """
        msg = {
            'op': 'publish',
            'topic': topic_name,
            'msg': message
        }
        json_msg = json.dumps(msg)
        self.ws.send(json_msg)

    def publish(self, topic_name, ros_message):
        """
        Publish on a topic given ROS message thru rosbridge.
        :param str topic_name: ROS topic name.
        :param * ros_message: Any ROS message instance, e.g. LaserScan()
            from sensor_msgs/LaserScan.
        """
        # First check if we already advertised the topic
        d = self._advertise_dict
        for k in d:
            if d[k]['topic_name'] == topic_name:
                # Already advertised, do nothing
                break
        else:
            # Not advertised, so we advertise
            topic_type = ros_message._type
            self._advertise(topic_name, topic_type)
        # Converting ROS message to a dictionary thru YAML
        ros_message_as_dict = yaml.load(ros_message.__str__())
        # Publishing
        self._publish(topic_name, ros_message_as_dict)

    def subscribe(self, topic_name, msgs_data, pub_topic_name =''):

        if pub_topic_name == '':
            pub_topic_name = topic_name

        # added to list
        pub = rospy.Publisher(pub_topic_name, msgs_data.__class__, queue_size=10)

        self.sub_list[topic_name] = [msgs_data, pub_topic_name, pub]

        pub_msg = {
            'op': 'subscribe',
            'topic': topic_name,
            'msgs_data' : msgs_data._type
        }

        # send to server
        json_msg = json.dumps(pub_msg)
        self.ws.send(json_msg)
        self.initFlaf = True
        print("%s | Sub to: %s msgs_data: %s" % (self.name, topic_name, msgs_data._type))

    def _callback(self):
        """
        Publish onto the already advertised topic the msg in the shape of
        a Python dict.
        :param str topic_name: ROS topic name.
        :param dict msg: Dictionary containing the definition of the message.
        """
        if not self.initFlaf:
            return

        json_message = self.ws.recv()
        # print(json_message)

        type_msg = json.loads(json_message)['op']
        print(type_msg)
        if type_msg == 'publish':
            print('publish msg')
            # conver json to ROS msgs
            msgs_conf =  self.sub_list[json.loads(json_message)['topic']]
            dictionary = json.loads(json_message)['msg']
            result =  message_converter.convert_dictionary_to_ros_message(msgs_conf[0]._type, dictionary)
            # print("Type: '%s' \n Received: '%s'" % (msgs_conf[0]._type, result))
            # pub to topic
            msgs_conf[2].publish(result)
        if type_msg == 'service_response':
            print("service_response:", json.loads(json_message)['result'])

        # return result

    def call_service(self, topic_name, msgs_data, pub_topic_name =''):
        self.initFlaf = True

        if pub_topic_name == '':
            pub_topic_name = topic_name

        ros_message_as_dict = yaml.load(msgs_data.__str__())

        pub_msg = {
            'op': 'call_service',
            'service': topic_name,
            "args": ros_message_as_dict
        }

        # send to server
        json_msg = json.dumps(pub_msg)
        self.ws.send(json_msg)
        print("%s | call_service: %s msgs_data: %s" % (self.name,topic_name, msgs_data))




#
# if __name__ == '__main__':
#     rospy.init_node('websicket_pub', anonymous=True)
#
#     rate = rospy.Rate(10)
#
#     # subscribe to servise and pub to ros pub
#     connect = WebsocketROSClient('192.168.128.10', 9090)
#     connect.subscribe('/mavros/local_position/pose', PoseStamped(), '/mavros/local_position/pose')
#     # connect.subscribe('/mavros/imu/data', Imu(),"/test2")
#     # call test service
#     srv_pub = AddTwoIntsRequest()
#     srv_pub.a = 4
#     srv_pub.b = 6
#     connect.call_service('add_two_ints', srv_pub)
#
#     try:
#         while not rospy.is_shutdown():
#             # Publish to server
#             pose = PoseStamped()
#             pose.header = Header()
#             pose.header.stamp = rospy.Time.now()
#             pose.header.frame_id = "map"
#             pose.pose.position.x = 1.5
#             connect.publish("/pose",pose)
#
#             rate.sleep()
#
#     except:
#         print("exit")
#         # connect.stop()
#         raise
