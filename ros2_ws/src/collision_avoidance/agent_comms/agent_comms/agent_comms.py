#!/usr/bin/env python3
"""
Per-agent communication node, independent of the message type it carries.

    verifier --my_rs--> agent_comms --> radio --> agent_comms --received_rs--> checker

Run it inside the agent namespace (drone1, hover1, fixedwing1, ...). The namespace only separates the topics.
SystemID is the unique numeric id of the agent (0..255). It must be different for every agent, packets with
the own ID are dropped. All topics are relative; only radio_in / radio_out are shared, through the launch remap.

The message type is not configured: it is discovered from the publisher of my_rs (the verifier). Until
that publisher shows up, nothing is sent or received. Packets of another type are dropped, so agents running
different checkers can share the radio.

Topics (relative)
    sub  radio_in     std_msgs/String  radio traffic; the launch file remaps radio_in and radio_out
    pub  radio_out    std_msgs/String  to the same /radio_traffic, shared by all drones
    sub  my_rs        <discovered>     reachable set computed by this agent's verifier
    pub  received_rs  <discovered>     reachable sets of the other agents, one per radio message
    pub  br_time      std_msgs/Float32  logging, transfer time (s) of each received message

Radio message: a JSON object

    {"target": 0, "sender": 1, "type": "probstar_reach/msg/ProbStar", "msg": {...}}

target  0 means everyone (BROADCAST). Agent ids (SystemID) are integers 0..255
msg     the message as rosidl_runtime_py.message_to_ordereddict, rebuilt with set_message_fields
"""
import json

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rosidl_runtime_py import message_to_ordereddict, set_message_fields
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Float32, String

BROADCAST = 0  # target of a message for everyone


class AgentComms(Node):

    def __init__(self):
        super().__init__('agent_comms')

        self.ID = self.check_system_id(self.declare_parameter('SystemID', 0).value)

        self.radio_pub = self.create_publisher(String, 'radio_out', 10)
        self.create_subscription(String, 'radio_in', self.radio_traffic_callback, 30)

        # Logging
        self.broadcast_time_pub = self.create_publisher(Float32, 'br_time', 10)

        # Set once the type of my_rs is known
        self.msg_type = None
        self.type_name = None
        self.rcvd_rs_pub = None
        self.reach_topic = self.resolve_topic_name('my_rs')
        self.discover_timer = self.create_timer(0.5, self.discover_type)

        self.get_logger().info(f'agent_comms running: SystemID={self.ID}, waiting for a publisher on '
                               f'{self.reach_topic}')

    def check_system_id(self, system_id):
        """SystemID has no default on purpose: two agents with the same id ignore each other's messages."""
        if not 0 <= system_id <= 255:
            raise RuntimeError(
                f'SystemID must be a value between 0 and 255 (got {system_id}): '
                'pass system_id:=N to the launch file or -p SystemID:=N')
        return system_id

    def discover_type(self):
        """Take the message type from the graph once my_rs is advertised, then set up the typed topics."""
        types = dict(self.get_topic_names_and_types()).get(self.reach_topic)
        if not types:
            return
        if len(types) > 1:
            self.get_logger().error(f'{self.reach_topic} has several types {types}, cannot pick one')
            return
        self.discover_timer.cancel()
        self.type_name = types[0]
        self.msg_type = get_message(self.type_name)
        self.create_subscription(self.msg_type, 'my_rs', self.my_rs_callback, 5)
        self.rcvd_rs_pub = self.create_publisher(self.msg_type, 'received_rs', 10)
        self.get_logger().info(f'Relaying {self.type_name}')

    # ---- outgoing reachable set ----
    def my_rs_callback(self, msg):
        text = json.dumps({
            'target': BROADCAST,
            'sender': self.ID,
            'type': self.type_name,
            'msg': message_to_ordereddict(msg),
        })
        self.radio_pub.publish(String(data=text))

    # ---- incoming radio traffic ----
    def radio_traffic_callback(self, msg):
        try:
            packet = json.loads(msg.data)
            if packet['target'] not in (BROADCAST, self.ID) or packet['sender'] == self.ID:
                return
            if packet['type'] != self.type_name:
                self.get_logger().debug(f'Dropped {packet["type"]} message, relaying {self.type_name}')
                return
            rcvd_rs = self.msg_type()
            set_message_fields(rcvd_rs, packet['msg'])
        except (ValueError, TypeError, KeyError, AttributeError) as e:
            self.get_logger().warn(f'Malformed radio message ({e}): {msg.data!r}')
            return

        # Transfer time: from the sender verifier's publish (header.stamp) to here, both conversions included
        sent = Time.from_msg(rcvd_rs.header.stamp)
        self.broadcast_time_pub.publish(Float32(data=(self.get_clock().now() - sent).nanoseconds / 1e9))
        self.rcvd_rs_pub.publish(rcvd_rs)


def main():
    rclpy.init()
    node = AgentComms()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
