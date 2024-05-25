import time
import numpy as np
from gz.msgs.image_pb2 import Image
from gz.msgs.camera_info_pb2 import CameraInfo
from gz.transport import SubscribeOptions
from gz.transport import Node
import dt_apriltags

from PySide6 import QtCore as qtc
from PySide6 import QtGui as qtg

import cv2


class gzCamera(qtc.QObject):
    new_frame = qtc.Signal(object, object, object, object, float, qtg.QImage, float) # x, y, z, yaw, time, frame, RT step start
    def __init__(self, parent: qtc.QObject | None = ..., cfg: dict|None=...) -> None:
        super().__init__(parent)

        self.cam_cfg = cfg["camera"]
        # Camera calibration parameters
        self.fx, self.fy, self.cx, self.cy = self.cam_cfg["fx"], self.cam_cfg["fy"], self.cam_cfg["cx"], self.cam_cfg["cy"]

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

        # Subscribe to the camera info topic by registering a callback
        self.info_topic = "/camera_info"
        msg_type_name = CameraInfo.DESCRIPTOR.full_name
        sub_options = SubscribeOptions()

        if self.node.subscribe(self.info_topic, self.set_real_instrinsics, msg_type_name, sub_options):
            print("Subscribing to topic [{}]".format(self.info_topic))
        else:
            print("Error subscribing to topic [{}]. Camera unavailable.".format(self.info_topic))
        

    def detect_apriltags(self, frame):
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        # Convert frame to grayscale
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        x, y, z, yaw = (None, None, None, None)
        try:
            results = self.detector.detect(gray_frame, estimate_tag_pose=True, camera_params=[self.fx,self.fy,self.cx,self.cy], tag_size=1)

            for result in results:
                # print(result)
                # Draw the AprilTag boundaries
                pts = np.array(result.corners, dtype=np.int32)
                pts = pts.reshape((-1, 1, 2))
                isClosed = True
                color = (0, 0, 255)
                thickness = 2
                frame = cv2.polylines(frame, [pts], isClosed, color, thickness)

                # Draw the pose information
                rotation = result.pose_R
                trans = result.pose_t

                tag_size = self.tag_sizes[result.tag_id]
                yaw = np.degrees(np.arctan2(rotation[1, 0], rotation[0, 0]))
                yaw = (yaw + 180)%360-180

                # ox, oy = self.rotate_point(self.cam_cfg["x_offset"]*1000, self.cam_cfg["y_offset"]*1000, yaw)
                ox,oy = 0,0

                x = trans[0][0]*tag_size - ox
                y = trans[1][0]*tag_size - oy
                z = trans[2][0]*tag_size
                
                frame = cv2.putText(frame, f"X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}", (pts[0][0][0], pts[0][0][1] - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                frame = cv2.putText(frame, f"Yaw: {yaw:.2f}", (pts[0][0][0], pts[0][0][1] - 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        except AttributeError:
            pass

        return frame, x, y, z, yaw
    
    def rotate_point(self, x, y, yaw):
        # Convert yaw angle to radians
        yaw_rad = np.radians(yaw)
        
        # Create rotation matrix
        R = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad)],
                    [np.sin(yaw_rad), np.cos(yaw_rad)]])
        
        # Create a column vector representing the original point
        point = np.array([[x], [y]])
        
        # Rotate the point using matrix multiplication
        rotated_point = np.dot(R, point)
        
        # Extract the rotated coordinates
        rotated_x = rotated_point[0, 0]
        rotated_y = rotated_point[1, 0]
        
        return rotated_x, rotated_y

    def cb(self, msg: Image) -> None:
        step_start = time.perf_counter()
        if self.cb_running: # drop current frame if the previous is not processed yet
            return
        self.cb_running = True

        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))

        # Detect AprilTags in the frame with pose estimation
        try:
            frame_with_tags, x, y, z, yaw = self.detect_apriltags(frame)
        except Exception:
            self.cb_running = False
            return
        t = msg.header.stamp.sec + msg.header.stamp.nsec/1000000000.0

        bytes_per_line = 3*frame_with_tags.shape[1]
        Qframe = qtg.QImage(frame_with_tags.data, frame_with_tags.shape[1], frame_with_tags.shape[0], bytes_per_line, qtg.QImage.Format_BGR888)

        self.new_frame.emit(x, y, z, yaw, t, Qframe, step_start)

        self.cb_running = False

    def set_real_instrinsics(self, msg: CameraInfo) -> None:
        intrinsics = msg.intrinsics
        self.fx = intrinsics.k[0]
        self.fy = intrinsics.k[4]
        self.cx = intrinsics.k[2]
        self.cy = intrinsics.k[5]
        self.node.unsubscribe(self.info_topic)

    def stop(self):
        time.sleep(0.5)
        del self.detector