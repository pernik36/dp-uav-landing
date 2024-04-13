from gz.msgs.clock_pb2 import Clock

from gz.transport import SubscribeOptions
from gz.transport import Node


class gzClock():
    def __init__(self) -> None:
        self.node = Node()

        # Subscribe to the clock topic by registering a callback
        self.topic = "/clock"
        msg_type_name = Clock.DESCRIPTOR.full_name
        sub_options = SubscribeOptions()

        if self.node.subscribe(self.topic, self.cb, msg_type_name, sub_options):
            print("Subscribing to topic [{}]".format(self.topic))
        else:
            print("Error subscribing to topic [{}]. Clock unavailable.".format(self.topic))

    def cb(self, msg: Clock) -> None:
        self.time = msg

    def stop(self):
        self.node.unsubscribe(self.topic)