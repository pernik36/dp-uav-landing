import time
import numpy as np
from gz.msgs.image_pb2 import Image
from gz.transport import SubscribeOptions
from gz.transport import Node
import dt_apriltags

from PySide6 import QtCore as qtc
from PySide6 import QtGui as qtg

# import cv2


class gzCamera(qtc.QObject):
    new_frame = qtc.Signal(float, float, float, float, float, qtg.QImage) # x, y, z, yaw, time, frame
    def __init__(self, parent: qtc.QObject | None = ..., cfg: dict|None=...) -> None:
        super().__init__(parent)

        cam_cfg = cfg["camera"]
        # Camera calibration parameters
        self.fx, self.fy, self.cx, self.cy = cam_cfg["fx"], cam_cfg["fy"], cam_cfg["cx"], cam_cfg["cy"]

        self.cb_running = False

        # Initialize AprilTag detectors
        self.detector = dt_apriltags.Detector(families="tagCustom48h12 tag36h11")
        self.tag_sizes = {137: 80, 164: 300} # TODO: melo by byt zavisle na velikosti nastavene v GUI

        self.node = Node()

        # Subscribe to the camera topic by registering a callback
        topic = "/camera"
        msg_type_name = Image.DESCRIPTOR.full_name
        sub_options = SubscribeOptions()

        if self.node.subscribe(topic, self.cb, msg_type_name, sub_options):
            print("Subscribing to topic [{}]".format(topic))
        else:
            print("Error subscribing to topic [{}]. Camera unavailable.".format(topic))
        

    def detect_apriltags(self, frame):
        # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        # # Convert frame to grayscale
        # gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # results = self.detector.detect(gray_frame, estimate_tag_pose=True, camera_params=[self.fx,self.fy,self.cx,self.cy], tag_size=1)

        # x, y, z, yaw = (None, None, None, None)

        # for result in results:
        #     # print(result)
        #     # Draw the AprilTag boundaries
        #     pts = np.array(result.corners, dtype=np.int32)
        #     pts = pts.reshape((-1, 1, 2))
        #     isClosed = True
        #     color = (0, 0, 255)
        #     thickness = 2
        #     frame = cv2.polylines(frame, [pts], isClosed, color, thickness)

        #     # Draw the pose information
        #     pose = result.pose_R
        #     trans = result.pose_t

        #     tag_size = self.tag_sizes[result.tag_id]

        #     x = trans[0][0]*tag_size
        #     y = trans[1][0]*tag_size
        #     z = trans[2][0]*tag_size
        #     yaw = pose[2][0]
            
        #     frame = cv2.putText(frame, f"X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}", (pts[0][0][0], pts[0][0][1] - 10),
        #                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        #     frame = cv2.putText(frame, f"Yaw: {yaw:.2f}", (pts[0][0][0], pts[0][0][1] - 30),
        #                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        x = y = z = yaw = 69.0

        return frame, x, y, z, yaw

    def cb(self, msg: Image) -> None:
        if self.cb_running: # drop current frame if the previous is not processed yet
            return
        self.cb_running = True

        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))

        # Detect AprilTags in the frame with pose estimation
        frame_with_tags, x, y, z, yaw = self.detect_apriltags(frame)
        t = time.time()

        bytes_per_line = 3*frame_with_tags.shape[1]
        Qframe = qtg.QImage(frame_with_tags.data, frame_with_tags.shape[1], frame_with_tags.shape[0], bytes_per_line, qtg.QImage.Format_BGR888)

        self.new_frame.emit(x, y, z, yaw, t, Qframe)

        self.cb_running = False