from math import atan, radians, tan, pi
from gzUAVPose import gzUAVPose
pipul = pi/2
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.telemetry import LandedState, FlightMode
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed)

from PySide6 import QtCore as qtc
from PySide6 import QtGui as qtg
from PySide6.QtCore import Qt

from abc import ABC, abstractmethod
from qasync import asyncSlot
import asyncio

from gzCamera import gzCamera
from gzClock import gzClock
from mission_result import MissionResult
from algStateTransition import algStateTransition as stateTransition

from grpc._channel import _MultiThreadedRendezvous

from simple_pid import PID

earth_circumference = 40074155.8892

class LAMeta(type(qtc.QObject), type(ABC)):
    pass

class landingAlg(ABC, qtc.QObject, metaclass=LAMeta):

    def __init__(self, cfg) -> None:
        self.cfg = cfg
        self.state_transitions = []

    @abstractmethod
    def run(self, mission, uav_pl_rel_x, uav_pl_rel_y, camera):
        pass

    @abstractmethod
    def stop():
        pass

    def log_state_transition(self, target_state_i):
        self.state_transitions.append(stateTransition(self.state.i, target_state_i, self.clock.time.sim, self.UAVPose.pose))

class AlgsModel(qtc.QAbstractListModel):
    def __init__(self, cfg, algsList: list[landingAlg] | None = None):
        super().__init__()
        self.list = algsList or [TakeoffAndLand(cfg), TakeoffAndLand(cfg, "Vzleť a Přistaň 3m", h=3), mavlink_test(cfg), OffboardPID(cfg, name="P0.5"), OffboardPID(cfg, name="P0.8 I0.01", k_p=0.8, k_i=0.01), OffboardPID(cfg, name="P2 I0.2", k_p=2, k_i=0.2)]

    def data(self, index, role):
        if role == Qt.DisplayRole:
            alg = self.list[index.row()]
            return alg.name

    def rowCount(self, index):
        return len(self.list)
    
class algState():
    def __init__(self, index, name, step) -> None:
        self.name = name
        self.step = step
        self.i = index
    
class mavlink_test(landingAlg, qtc.QObject):
    ended = qtc.Signal(MissionResult)
    def __init__(self, cfg) -> None:
        landingAlg.__init__(self, cfg)
        qtc.QObject.__init__(self)
        self.stopped = True
        self.drone = System()
        self.name = "MAVlink test"
        self.tasks = []

    @asyncSlot(dict, float, float, gzCamera)
    async def run(self, mission, uav_pl_rel_x, uav_pl_rel_y, camera: gzCamera):
        self.stopped = False
        self.clock = gzClock()
        self.UAVPose = gzUAVPose(self.cfg)
        self.mission = mission
        await self.drone.connect(system_address="udp://:14540")
        self.cam = camera

        self.tasks.append(asyncio.ensure_future(self.subscribe_attitude()))
        # altitude_task = asyncio.ensure_future(self.subscribe_altitude())
        self.start_time = self.clock.time

    @asyncSlot(None)
    async def stop(self):
        if self.stopped:
            return
        self.stopped = True
        self.clock.stop()
        self.UAVPose.stop()
        for task in self.tasks:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        self.drone = System() # reset the connection
        self.ended.emit(MissionResult(self.mission, self.start_time, self.clock.time, self.UAVPose.pose))

    async def subscribe_altitude(self):
        """ Prints the altitude when it changes """
        previous_altitude = None

        try:
            async for position in self.drone.telemetry.position():
                altitude = position.relative_altitude_m
                altitude_rounded = round(altitude)
                if altitude_rounded != previous_altitude:
                    previous_altitude = altitude_rounded
                    print(f"Altitude: {altitude} m")
        except _MultiThreadedRendezvous:
            pass


    async def subscribe_attitude(self):
        global pitch_deg, roll_deg
        try:
            async for attitude in self.drone.telemetry.attitude_euler():
                pitch_deg = attitude.pitch_deg
                roll_deg = attitude.roll_deg
                # print(f"Pitch: {pitch_deg:6.2f}, Roll: {roll_deg:6.2f}", end="\r")
        except _MultiThreadedRendezvous:
            pass

class OffboardPID(landingAlg, qtc.QObject):
    ended = qtc.Signal(MissionResult)
    def __init__(self, cfg, name = "Základní", k_p=0.5, k_i=0, k_d=0) -> None:
        landingAlg.__init__(self, cfg)
        qtc.QObject.__init__(self)
        self.stopped = True
        self.drone = System()
        self.name = name
        self.tasks = []
        self.states = [algState(0, "preflight", self.preflightStep)        # --> 0
                      ,algState(1, "takeoff", self.takeoffStep)            # --> 1
                      ,algState(2, "hover", self.hoverStep)                # --> 2
                      ,algState(3, "find_target", self.find_targetStep)    # --> 3
                      ,algState(4, "centering", self.centeringStep)        # --> 4
                      ,algState(5, "landing", self.landingStep)            # --> 5
                      ]
        self.state = self.states[0]
        self.stateTransitions = [[None for s in self.states] for t in self.states]
        self.stateTransitions[0][1] = self.trans_preflight2takeoff
        self.stateTransitions[1][3] = self.trans_takeoff2find_target
        self.stateTransitions[3][4] = self.trans_find_target2centering
        self.stateTransitions[4][3] = self.trans_centering2find_target

        self.lt = float("infinity")

        v_max = self.cfg["uav_max_v"]
        self.pid_r = PID(k_p, k_i, k_d, setpoint=0, auto_mode=False, output_limits=(-v_max, v_max))
        self.pid_p = PID(k_p, k_i, k_d, setpoint=0, auto_mode=False, output_limits=(-v_max, v_max))
        self.pid_y = PID(k_p, k_i, k_d, setpoint=0, auto_mode=False)

    @asyncSlot(dict, float, float, gzCamera)
    async def run(self, mission, uav_pl_rel_x, uav_pl_rel_y, camera: gzCamera):
        self.clock = gzClock()
        self.UAVPose = gzUAVPose(self.cfg)
        self.stopped = False
        self.mission = mission
        await self.drone.connect(system_address="udp://:14540")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                break

        self.last_t = None

        self.state = self.states[0]

        self.tasks.append(asyncio.ensure_future(self.subscribe_attitude()))
        self.tasks.append(asyncio.ensure_future(self.subscribe_altitude()))
        self.tasks.append(asyncio.ensure_future(self.subscribe_flight_mode()))
        self.tasks.append(asyncio.ensure_future(self.subscribe_landed_state()))

        self.cam = camera

        self.stepTimer = qtc.QTimer()
        self.stepTimer.timeout.connect(self.step)
        self.stepTimer.start(100)

        self.uav_pl_rel_x = uav_pl_rel_x
        self.uav_pl_rel_y = uav_pl_rel_y

        self.pid_p.set_auto_mode(False)
        self.pid_r.set_auto_mode(False)
        self.pid_y.set_auto_mode(False)

    @asyncSlot(None)
    async def step(self):
        await self.state.step()

    @asyncSlot(None)
    async def stop(self):
        if self.stopped:
            return
        self.stopped = True
        self.clock.stop()
        for task in self.tasks:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        self.drone = System() # reset the connection
        self.stepTimer.stop()
        try:
            self.cam.new_frame.disconnect()
        except RuntimeError: # Already disconnected
            pass
        self.ended.emit(MissionResult(self.mission, self.start_time, self.clock.time, self.UAVPose.pose, self.state_transitions))

    async def subscribe_altitude(self):
        try:
            async for position in self.drone.telemetry.position():
                self.altitude = position.relative_altitude_m
                # print(f"h = {self.altitude} m", end = "\r")
        except _MultiThreadedRendezvous:
            pass

    async def subscribe_attitude(self):
        try:
            async for attitude in self.drone.telemetry.attitude_euler():
                self.pitch_deg = attitude.pitch_deg
                self.roll_deg = attitude.roll_deg
                # print(f"Pitch: {self.pitch_deg:6.2f}, Roll: {self.roll_deg:6.2f}", end="\r")
        except _MultiThreadedRendezvous:
            pass

    async def subscribe_flight_mode(self):
        try:
            async for flight_mode in self.drone.telemetry.flight_mode():
                self.flight_mode = flight_mode
        except _MultiThreadedRendezvous:
            pass
    async def subscribe_landed_state(self):
        try:
            async for state in self.drone.telemetry.landed_state():
                self.landed_state = state
        except _MultiThreadedRendezvous:
            pass

    async def preflightStep(self):
        self.stepTimer.blockSignals(True)
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                break
            else:
                self.stepTimer.blockSignals(False)
                return
            
        print("-- Global position state is good enough for flying.")
        await self.trans_preflight2takeoff()
        self.stepTimer.blockSignals(False)

    async def trans_preflight2takeoff(self):
        new_state = self.states[1]
        print(f"-- Switching the state from {self.state.name} to {new_state.name}")
        print("-- Arming")
        retry = True
        delay = 2
        while retry:
            try:
                await self.drone.action.arm()
                retry = False
            except ActionError as e:
                print(e)
                print("   Retrying in", delay)
                await asyncio.sleep(delay)
                delay *= 1.5

        print("-- Taking off")
        await self.drone.action.set_takeoff_altitude(10.0)
        await self.drone.action.takeoff()
        self.start_time = self.clock.time

        self.log_state_transition(new_state.i)
        self.state = new_state
        print(f"-- State switched to {new_state.name}")

    async def takeoffStep(self):
        if not self.landed_state == LandedState.IN_AIR:
            return
        self.stepTimer.blockSignals(True)
        await self.trans_takeoff2find_target()

    async def trans_takeoff2find_target(self):
        await self.trans2find_target()

    async def trans2find_target(self):
        new_state = self.states[3]
        print(f"-- Switching the state from {self.state.name} to {new_state.name}")
        self.log_state_transition(new_state.i)
        self.state = new_state
        print(f"-- State switched to {new_state.name}")

        async for terrain_info in self.drone.telemetry.home():
            absolute_altitude = terrain_info.absolute_altitude_m
            home_lat_deg = terrain_info.latitude_deg
            home_lon_deg = terrain_info.longitude_deg
            break
        pl_lat_deg = home_lat_deg + (self.uav_pl_rel_y*360)/earth_circumference
        pl_lon_deg = home_lon_deg + (self.uav_pl_rel_x*360)/earth_circumference

        h = self.cfg["uav_h"]

        print(f"-- Flying over to the target {h} m above the ground.")
        try:
            await self.drone.action.goto_location(pl_lat_deg, pl_lon_deg, absolute_altitude + h, 0)
        except ActionError as e:
            print(e)                                # TODO: handle error

        try:
            async for position in self.drone.telemetry.position():
                if abs(position.latitude_deg - pl_lat_deg) < 0.000045 and abs(position.longitude_deg - pl_lon_deg) < 0.000045: # cca 5m
                    break
        except _MultiThreadedRendezvous:
            pass
        print("-- Over the target. Trying to localize AprilTag.")

        self.tasks[1].cancel()                                              # reset task after subscribing to position
        try:
                await self.tasks[1]
        except asyncio.CancelledError:
            pass
        self.tasks[1] = asyncio.ensure_future(self.subscribe_altitude()) 

        self.cam.new_frame.connect(self.wait_for_apriltag)

    async def hoverStep(self):
        print(f"Pitch: {self.pitch_deg:6.2f}, Roll: {self.roll_deg:6.2f}", end="\r")

    async def find_targetStep(self):
        pass

    # @asyncSlot(float, float, float, float, float, qtg.QImage)
    # async def newCameraFrame(self, x, y, z, yaw, t, frame):
    #     print(x)

    @asyncSlot(object, object, object, object, float, qtg.QImage)
    async def wait_for_apriltag(self, x, y, z, yaw, t, frame):
        if x is not None:
            self.cam.new_frame.disconnect(self.wait_for_apriltag)
            print("-- Found an AprilTag.")
            await self.trans_find_target2centering()
            return
        print("-- AprilTag not found in the image.", end="\r")

    async def trans_find_target2centering(self):
        new_state = self.states[4]
        print(f"-- Switching the state from {self.state.name} to {new_state.name}")

        print("-- Setting initial offboard control setpoint")
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

        print("-- Starting offboard")
        delay = 2
        max_retries = 5
        for i in range(max_retries):
            try:
                await self.drone.offboard.start()
                await asyncio.sleep(delay)
                print("-- Offboard started")

                self.pid_p.set_auto_mode(True, 0)
                self.pid_r.set_auto_mode(True, 0)
                self.pid_y.set_auto_mode(True, 0)

                self.cam.new_frame.connect(self.centeringCamStep)
                return
            except OffboardError as error:
                print(f"   Starting offboard mode failed with error code: {error._result.result}")
                print("   Retrying in", delay)
                await asyncio.sleep(delay)
                delay *= 1.5

        print("   Failed to start the offboard mode") # TODO: Osetrit, co v tomto pripade delat

        self.log_state_transition(new_state.i)
        self.state = new_state
        print("-- State switched to centering")

    @asyncSlot(object, object, object, object, float, qtg.QImage)
    async def centeringCamStep(self, x, y, z, yaw, t, frame):
        if self.last_t is None:
            self.last_t = t
            return
        if self.altitude < self.cfg["landing_trigger_alt"]:
            await self.trans_centering2land()
            return
        sp_v_p = 0
        sp_v_r = 0
        sp_v_d = -0.3

        if x is not None:
            self.steps_wo_vis = 0
            d_r = self.altitude * tan(pipul - atan(z/x) - radians(self.roll_deg))
            d_p = -self.altitude * tan(pipul - atan(z/y) - radians(self.pitch_deg))

            # self.last_d_r = d_r
            # self.last_d_p = d_p

            dt = t - self.last_t
            self.last_t = t

            # sp_v_r = self.k_p * d_r
            # sp_v_p = self.k_p * d_p

            sp_v_r = self.pid_r(-d_r, dt)
            sp_v_p = self.pid_p(-d_p, dt)
            sp_v_d = 0.5/(1+12.0/self.altitude*(d_r*d_r + d_p*d_p))

            print(f"v_p: {sp_v_p:6.2f}, v_r: {sp_v_r:6.2f}", end="\r")

        else:
            self.steps_wo_vis += 1
            if self.steps_wo_vis>=self.cfg["max_steps_wo_visual"]:
                await self.trans_centering2find_target()

        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(sp_v_p, sp_v_r, sp_v_d, 0.0))

    async def trans_centering2find_target(self):
        self.cam.new_frame.disconnect(self.centeringCamStep)
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0.0))
        new_state = self.states[3]
        print(f"-- Switching the state from {self.state.name} to {new_state.name}")
        print("-- Stopping offboard")
        delay = 0.05
        max_retries = 5
        for i in range(max_retries):
            try:
                await self.drone.offboard.stop()
                await asyncio.sleep(delay)
                print("-- Offboard stopped")
                self.state = new_state
                print(f"-- State switched to {new_state.name}")
                self.pid_p.set_auto_mode(False)
                self.pid_r.set_auto_mode(False)
                self.pid_y.set_auto_mode(False)
                self.pid_p.reset()
                self.pid_r.reset()
                self.pid_y.reset()
                self.last_t = None
                await self.trans2find_target()
                return
            except OffboardError as error:
                print(f"   Stopping offboard mode failed with error code: {error._result.result}")
                print("   Retrying in", delay)
                await asyncio.sleep(delay)
                delay *= 1.5

        print("   Failed to stop the offboard mode")
        

    async def trans_centering2land(self):
        self.cam.new_frame.disconnect(self.centeringCamStep)
        new_state = self.states[5]
        print(f"-- Switching the state from {self.state.name} to {new_state.name}")
        print(f"-- State switched to {new_state.name}")
        self.log_state_transition(new_state.i)
        self.state = new_state
        print(f"-- Switching the state from {self.state.name} to {new_state.name}")
        print(f"-- State switched to {new_state.name}")
        await self.drone.action.land()
        self.stepTimer.blockSignals(False)

    async def centeringStep(self):
        pass

    async def landingStep(self):
        if not self.landed_state == LandedState.ON_GROUND:
            return
        self.stepTimer.blockSignals(True)
        print("-- Landed.")
        await self.stop()

class TakeoffAndLand(landingAlg, qtc.QObject):
    ended = qtc.Signal(MissionResult)
    def __init__(self, cfg, name = "Vzleť a Přistaň", h = 2) -> None:
        landingAlg.__init__(self, cfg)
        qtc.QObject.__init__(self)
        self.stopped = True
        self.drone = System()
        self.name = name
        self.tasks = []
        self.states = [algState(0, "preflight", self.preflightStep)        # --> 0
                      ,algState(1, "takeoff", self.takeoffStep)            # --> 1
                      ,algState(2, "landing", self.landingStep)            # --> 2
                      ]
        self.state = self.states[0]
        self.stateTransitions = [[None for s in self.states] for t in self.states]
        self.stateTransitions[0][1] = self.trans_preflight2takeoff
        self.stateTransitions[1][2] = self.trans_takeoff2landing

        self.takeoff_h = h

    @asyncSlot(dict, float, float, gzCamera)
    async def run(self, mission, uav_pl_rel_x, uav_pl_rel_y, camera: gzCamera):
        self.clock = gzClock()
        self.UAVPose = gzUAVPose(self.cfg)
        self.stopped = False
        self.mission = mission
        await self.drone.connect(system_address="udp://:14540")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                break

        self.last_t = None

        self.state = self.states[0]

        self.tasks.append(asyncio.ensure_future(self.subscribe_altitude()))
        self.tasks.append(asyncio.ensure_future(self.subscribe_flight_mode()))
        self.tasks.append(asyncio.ensure_future(self.subscribe_landed_state()))

        self.cam = camera

        self.stepTimer = qtc.QTimer()
        self.stepTimer.timeout.connect(self.step)
        self.stepTimer.start(100)

        self.uav_pl_rel_x = uav_pl_rel_x
        self.uav_pl_rel_y = uav_pl_rel_y

    @asyncSlot(None)
    async def step(self):
        await self.state.step()

    @asyncSlot(None)
    async def stop(self):
        if self.stopped:
            return
        self.stopped = True
        self.clock.stop()
        for task in self.tasks:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        self.drone = System() # reset the connection
        self.stepTimer.stop()
        try:
            self.cam.new_frame.disconnect()
        except RuntimeError: # Already disconnected
            pass
        self.ended.emit(MissionResult(self.mission, self.start_time, self.clock.time, self.UAVPose.pose, self.state_transitions))
        print("Ended signal emitted")

    async def subscribe_altitude(self):
        try:
            async for position in self.drone.telemetry.position():
                self.altitude = position.relative_altitude_m
                # print(f"h = {self.altitude} m", end = "\r")
        except _MultiThreadedRendezvous:
            pass

    async def subscribe_flight_mode(self):
        try:
            async for flight_mode in self.drone.telemetry.flight_mode():
                self.flight_mode = flight_mode
        except _MultiThreadedRendezvous:
            pass
    async def subscribe_landed_state(self):
        try:
            async for state in self.drone.telemetry.landed_state():
                self.landed_state = state
        except _MultiThreadedRendezvous:
            pass

    async def preflightStep(self):
        self.stepTimer.blockSignals(True)
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                break
            else:
                self.stepTimer.blockSignals(False)
                return
            
        print("-- Global position state is good enough for flying.")
        await self.trans_preflight2takeoff()
        self.stepTimer.blockSignals(False)

    async def trans_preflight2takeoff(self):
        new_state = self.states[1]
        print(f"-- Switching the state from {self.state.name} to {new_state.name}")
        print("-- Arming")
        retry = True
        delay = 2
        while retry:
            try:
                await self.drone.action.arm()
                retry = False
            except ActionError as e:
                print(e)
                print("   Retrying in", delay)
                await asyncio.sleep(delay)
                delay *= 1.5

        print("-- Taking off")
        await self.drone.action.set_takeoff_altitude(self.takeoff_h)
        await self.drone.action.takeoff()
        self.start_time = self.clock.time

        self.log_state_transition(new_state.i)
        self.state = new_state # change state to taking off
        print(f"-- State switched to {new_state.name}")

    async def takeoffStep(self):
        if not self.landed_state == LandedState.IN_AIR:
            return
        self.stepTimer.blockSignals(True)
        await self.stateTransitions[self.states.index(self.state)][2]()
        

    async def trans_takeoff2landing(self):
        new_state = self.states[2]
        print(f"-- Switching the state from {self.state.name} to {new_state.name}")
        self.log_state_transition(new_state.i)
        self.state = new_state
        print(f"-- State switched to {new_state.name}")
        await self.drone.action.land()
        self.stepTimer.blockSignals(False)

    async def landingStep(self):
        if not self.landed_state == LandedState.ON_GROUND:
            return
        self.stepTimer.blockSignals(True)
        print("-- Landed.")
        await self.stop()