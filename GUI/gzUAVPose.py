from gz.msgs.odometry_pb2 import Odometry

from gz.transport import SubscribeOptions
from gz.transport import Node


class gzUAVPose():
    def __init__(self, cfg) -> None:
        self.node = Node()


        self.topic = f"/model/{cfg['gz_model']}_0/odometry"
        msg_type_name = Odometry.DESCRIPTOR.full_name
        sub_options = SubscribeOptions()

        if self.node.subscribe(self.topic, self.cb, msg_type_name, sub_options):
            print("Subscribing to topic [{}]".format(self.topic))
        else:
            print("Error subscribing to topic [{}]".format(self.topic))

    def cb(self, msg: Odometry) -> None:
        self.pose = msg.pose

    def stop(self):
        self.node.unsubscribe(self.topic)