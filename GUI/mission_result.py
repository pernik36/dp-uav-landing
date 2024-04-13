class MissionResult():
    def __init__(self, mission_definition, start_time, end_time, end_pose, state_transitions = [], extra = {}) -> None:
        self.start_time = start_time
        self.end_time = end_time
        self.state_transitions = state_transitions
        self.mission_def = mission_definition
        self.end_pose = end_pose
        self.extra = extra

    def to_dict(self):
        result = dict()
        result["mission_definition"] = self.mission_def
        result["start_time"] = {"sec": self.start_time.sim.sec, "nsec": self.start_time.sim.nsec}
        result["end_time"] = {"sec": self.end_time.sim.sec, "nsec": self.end_time.sim.nsec}
        result["start_realtime"] = {"sec": self.start_time.real.sec, "nsec": self.start_time.real.nsec}
        result["end_realtime"] = {"sec": self.end_time.real.sec, "nsec": self.end_time.real.nsec}
        result["end_pose"] = {"position": {"x": self.end_pose.position.x, "y": self.end_pose.position.y, "z": self.end_pose.position.z}, "orientation": {"x": self.end_pose.orientation.x, "y": self.end_pose.orientation.y, "z": self.end_pose.orientation.z}}
        result["state_transitions"] = [st.to_dict() for st in self.state_transitions]
        for key in self.extra:
            result[key] = self.extra[key]

        return result