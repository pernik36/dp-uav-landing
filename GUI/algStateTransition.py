class algStateTransition():
    def __init__(self, src_state, target_state, time, uav_pose) -> None:
        self.src_state = src_state
        self.target_state = target_state
        self.time = time
        self.uav_pose = uav_pose

    def to_dict(self):
        result = dict()
        result["src_state_index"] = self.src_state
        result["target_state_index"] = self.target_state
        result["time"] = {"sec": self.time.sec, "nsec": self.time.nsec}
        result["pose"] = {"position": {"x": self.uav_pose.position.x, "y": self.uav_pose.position.y, "z": self.uav_pose.position.z}, "orientation": {"x": self.uav_pose.orientation.x, "y": self.uav_pose.orientation.y, "z": self.uav_pose.orientation.z}}

        return result