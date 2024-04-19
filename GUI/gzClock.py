from gz.msgs.clock_pb2 import Clock

from gz.transport import SubscribeOptions
from gz.transport import Node

from PySide6 import QtCore as qtc

class gzClock(qtc.QObject):
    time_elapsed = qtc.Signal(None)
    def __init__(self) -> None:
        super().__init__()
        self.node = Node()

        # Subscribe to the clock topic by registering a callback
        self.topic = "/clock"
        msg_type_name = Clock.DESCRIPTOR.full_name
        sub_options = SubscribeOptions()

        if self.node.subscribe(self.topic, self.cb, msg_type_name, sub_options):
            print("Subscribing to topic [{}]".format(self.topic))
        else:
            print("Error subscribing to topic [{}]. Clock unavailable.".format(self.topic))

        self.timer_on = False
        self.time_start = 0.0
        self.duration = 0.0

    def time_to_float(self, time):
        return time.sec + time.nsec/1000000000

    def cb(self, msg: Clock) -> None:
        self.time = msg
        if self.timer_on and self.time_to_float(self.time.sim) - self.time_start > self.duration:
            self.timer_on = False
            self.time_elapsed.emit()

    def start_timer(self, duration):
        if self.timer_on:
            return False
        self.time_start = self.time_to_float(self.time.sim)
        self.duration = duration
        self.timer_on = True
        return True

    def stop(self):
        self.node.unsubscribe(self.topic)