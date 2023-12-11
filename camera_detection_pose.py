import time
import cv2
import numpy as np
from gz.msgs.image_pb2 import Image
from gz.transport import SubscribeOptions
from gz.transport import Node
import dt_apriltags

# Camera calibration parameters
fx, fy, cx, cy = (614.1182502063649, 614.2811984891936, 289.5307050411072, 239.48850240020735)

cb_running = False

# Initialize AprilTag detectors
detectors = (dt_apriltags.Detector(families='tag36h11'), dt_apriltags.Detector(families="tagCustom48h12"))
tag_sizes = (80, 300)

def detect_apriltags(frame):
    # Convert frame to grayscale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    for detector, size in zip(detectors, tag_sizes):
    # detector = detectors[0]
    # size = tag_sizes[0]
        # Detect AprilTags in the frame with pose estimation
        results = detector.detect(gray_frame, estimate_tag_pose=True, camera_params=[fx,fy,cx,cy], tag_size=size)

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
            pose = result.pose_R
            trans = result.pose_t
            
            frame = cv2.putText(frame, f"X: {trans[0][0]:.2f}, Y: {trans[1][0]:.2f}, Z: {trans[2][0]:.2f}", (pts[0][0][0], pts[0][0][1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
            frame = cv2.putText(frame, f"Yaw: {pose[2][0]:.2f}", (pts[0][0][0], pts[0][0][1] - 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

    return frame

def cb(msg: Image) -> None:
    global cb_running
    if cb_running:
        return
    cb_running = True

    frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))

    # Detect AprilTags in the frame with pose estimation
    frame_with_tags = detect_apriltags(frame)

    # Display the frame with AprilTags
    cv2.imshow("Camera", frame_with_tags)
    cb_running = False
    cv2.waitKey(25)

def main():
    node = Node()

    # Subscribe to the camera topic by registering a callback
    topic = "/camera"
    msg_type_name = Image.DESCRIPTOR.full_name
    sub_options = SubscribeOptions()

    if node.subscribe(topic, cb, msg_type_name, sub_options):
        print("Subscribing to topic [{}]".format(topic))
    else:
        print("Error subscribing to topic [{}]".format(topic))
        return

    # Wait for shutdown
    try:
        while True:
            time.sleep(0.001)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()