from math import atan, radians, tan, pi
import math
import random

import numpy as np
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

from geopy.distance import distance
from geopy.point import Point

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

    def add_relative_distance(self, latitude, longitude, altitude, distance_north, distance_east):
        current_location = Point(latitude, longitude, altitude)
        north_destination = distance(meters=distance_north).destination(current_location, 0)
        east_destination = distance(meters=distance_east).destination(north_destination, 90)
        
        return east_destination.latitude, east_destination.longitude

class AlgsModel(qtc.QAbstractListModel):
    def __init__(self, cfg, algsList: list[landingAlg] | None = None):
        super().__init__()
        self.list = algsList or [FixMeasurement(cfg), EstimateMeasurementCovariance(cfg, N=500), EstimateCovariance(cfg, N=200), TakeoffAndLand(cfg), TakeoffAndLand(cfg, "Vzleť a Přistaň 3m", h=3), mavlink_test(cfg), OffboardPID(cfg, name="P0.5"), OffboardPID(cfg, name="P0.8 I0.01", k_p=0.8, k_i=0.01), OffboardPID(cfg, name="P2 I0.2", k_p=2, k_i=0.2)]

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
        pl_lat_deg, pl_lon_deg = self.add_relative_distance(home_lat_deg, home_lon_deg, absolute_altitude, self.uav_pl_rel_y, self.uav_pl_rel_x)

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


class EstimateCovariance(landingAlg, qtc.QObject):
    ended = qtc.Signal(MissionResult)
    def __init__(self, cfg, name = "Odhad kovariance x_0", N = 1000) -> None:
        landingAlg.__init__(self, cfg)
        qtc.QObject.__init__(self)
        self.stopped = True
        self.drone = System()
        self.name = name
        self.tasks = []
        self.states = [algState(0, "preflight", self.preflightStep)        # --> 0
                      ,algState(1, "takeoff", self.takeoffStep)            # --> 1
                      ,algState(2, "hover", self.hoverStep)                # --> 2
                      ,algState(3, "new_position", self.new_positionStep)    # --> 3
                      ]
        self.state = self.states[0]
        self.stateTransitions = [[None for s in self.states] for t in self.states]
        self.stateTransitions[0][1] = self.trans_preflight2takeoff
        self.stateTransitions[1][3] = self.trans_takeoff2new_position
        self.stateTransitions[3][2] = self.trans_new_position2hover

        self.N = N

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

        self.real_positions = []
        self.requested_positions = []

        self.i = 0

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
            try:
                self.cam.new_frame.disconnect()
            except RuntimeError: # Already disconnected
                pass
            self.cam = None
            self.ended.emit(MissionResult(self.mission, self.start_time, self.clock.time, self.UAVPose.pose, self.state_transitions, {"requested_postions": self.requested_positions, "real_positions": self.real_positions}))
        except AttributeError as e:
            print (e)

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
                self.yaw_deg = attitude.yaw_deg
                current_yaw = self.quat_to_yaw(self.UAVPose.pose.orientation)

                print(f"Pitch: {self.pitch_deg:6.2f}, Roll: {self.roll_deg:6.2f}, Yaw: {self.yaw_deg:6.2f}, Yaw(Q): {current_yaw:6.2f}", end="\r")
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

        async for terrain_info in self.drone.telemetry.home():
            absolute_altitude = terrain_info.absolute_altitude_m
            break

        print("-- Taking off")
        # await self.drone.action.set_takeoff_altitude(10.0)
        # await self.drone.action.takeoff()

        x = self.UAVPose.pose.position.x
        y = self.UAVPose.pose.position.y    
        yaw = 90

        lat_deg, lon_deg = self.add_relative_distance(self.cfg["origin"]["lat_deg"], self.cfg["origin"]["lon_deg"], self.cfg["origin"]["elevation"], y, x)
        try:
            await self.drone.action.goto_location(lat_deg, lon_deg, 10.0+absolute_altitude, yaw)
        except ActionError as e:
            print(e)                                # TODO: handle error


        self.start_time = self.clock.time

        self.log_state_transition(new_state.i)
        self.state = new_state
        print(f"-- State switched to {new_state.name}")

    async def takeoffStep(self):
        # if not self.landed_state == LandedState.IN_AIR:
        #     return
        if not self.altitude > 9.5:
            return
        self.stepTimer.blockSignals(True)
        await self.trans_takeoff2new_position()
        self.stepTimer.blockSignals(False)

    async def trans_takeoff2new_position(self):
        await self.trans2new_position()

    def random_point(self):
        angle = random.uniform(0, 2 * math.pi)

        yaw = random.uniform(-180, 180)
        
        x = 10 * math.cos(angle)
        y = 10 * math.sin(angle)
        h = self.cfg["uav_h"]
        
        return (x, y, h, yaw)

    async def trans2new_position(self):
        new_state = self.states[3]
        print(f"-- Switching the state from {self.state.name} to {new_state.name}")
        self.log_state_transition(new_state.i)
        self.state = new_state
        print(f"-- State switched to {new_state.name}")

        async for terrain_info in self.drone.telemetry.home():
            absolute_altitude = terrain_info.absolute_altitude_m
            break

        new_point_rel_x, new_point_rel_y, h, new_point_yaw = self.random_point()   
        x = self.UAVPose.pose.position.x + new_point_rel_x
        y = self.UAVPose.pose.position.y + new_point_rel_y        
        yaw = new_point_yaw
        self.requested_positions.append((x, y, h, yaw))

        lat_deg, lon_deg = self.add_relative_distance(self.cfg["origin"]["lat_deg"], self.cfg["origin"]["lon_deg"], self.cfg["origin"]["elevation"], y, x)

        print(f"-- Flying over to the target {h} m above the ground.")
        try:
            await self.drone.action.goto_location(lat_deg, lon_deg, h+absolute_altitude, yaw)
        except ActionError as e:
            print(e)                                # TODO: handle error

        while not self.clock.start_timer(10.0):
            await asyncio.sleep(0.1)
        self.clock.time_elapsed.connect(self.new_positionClockStep)

    async def hoverStep(self):
        self.stepTimer.blockSignals(True)
        await self.trans_hover2new_position()

    @asyncSlot(None)
    async def new_positionClockStep(self):
        self.clock.time_elapsed.disconnect()
        await self.trans_new_position2hover()
        self.stepTimer.blockSignals(False)

    async def new_positionStep(self):
        pass

    async def trans_new_position2hover(self):
        new_state = self.states[2]
        print(f"-- Switching the state from {self.state.name} to {new_state.name}")
        self.log_state_transition(new_state.i)
        self.state = new_state
        print(f"-- State switched to {new_state.name}")

        x = self.UAVPose.pose.position.x
        y = self.UAVPose.pose.position.y
        z = self.UAVPose.pose.position.z
        yaw = self.quat_to_yaw(self.UAVPose.pose.orientation)
        self.real_positions.append((x, y, z, yaw))

        self.i += 1
        print(f"-- Measured position {self.i}/{self.N}")
        if self.i >= self.N:
            await self.stop()

    async def trans_hover2new_position(self):
        await self.trans2new_position()

    def quat_to_yaw(self, q):
        yaw = (270 - math.degrees(math.atan2(2.0 * (q.z * q.w + q.x * q.y) , - 1.0 + 2.0 * (q.w * q.w + q.x * q.x))))%360 - 180
        return yaw
    
class EstimateMeasurementCovariance(landingAlg, qtc.QObject):
    ended = qtc.Signal(MissionResult)
    def __init__(self, cfg, name = "Odhad kovariance x_k", N = 1000) -> None:
        landingAlg.__init__(self, cfg)
        qtc.QObject.__init__(self)
        self.stopped = True
        self.drone = System()
        self.name = name
        self.tasks = []
        self.states = [algState(0, "preflight", self.preflightStep)        # --> 0
                      ,algState(1, "takeoff", self.takeoffStep)            # --> 1
                      ,algState(2, "hover", self.hoverStep)                # --> 2
                      ,algState(3, "new_position", self.new_positionStep)    # --> 3
                      ]
        self.state = self.states[0]
        self.stateTransitions = [[None for s in self.states] for t in self.states]
        self.stateTransitions[0][1] = self.trans_preflight2takeoff
        self.stateTransitions[1][3] = self.trans_takeoff2new_position
        self.stateTransitions[3][2] = self.trans_new_position2hover

        self.N = N

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

        self.real_positions = []
        self.measured_positions = []

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
            try:
                self.cam.new_frame.disconnect()
            except RuntimeError: # Already disconnected
                pass
            self.cam = None
            self.ended.emit(MissionResult(self.mission, self.start_time, self.clock.time, self.UAVPose.pose, self.state_transitions, {"measured_postions": self.measured_positions, "real_positions": self.real_positions}))
        except AttributeError as e:
            print (e)

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
                self.yaw_deg = attitude.yaw_deg
                current_yaw = self.quat_to_yaw(self.UAVPose.pose.orientation)

                print(f"Pitch: {self.pitch_deg:6.2f}, Roll: {self.roll_deg:6.2f}, Yaw: {self.yaw_deg:6.2f}, Yaw(Q): {current_yaw:6.2f}", end="\r")
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

        async for terrain_info in self.drone.telemetry.home():
            absolute_altitude = terrain_info.absolute_altitude_m
            break

        print("-- Taking off")
        # await self.drone.action.set_takeoff_altitude(10.0)
        # await self.drone.action.takeoff()

        x = self.UAVPose.pose.position.x
        y = self.UAVPose.pose.position.y    
        yaw = 90

        lat_deg, lon_deg = self.add_relative_distance(self.cfg["origin"]["lat_deg"], self.cfg["origin"]["lon_deg"], self.cfg["origin"]["elevation"], y, x)
        try:
            await self.drone.action.goto_location(lat_deg, lon_deg, 10.0+absolute_altitude, yaw)
        except ActionError as e:
            print(e)                                # TODO: handle error


        self.start_time = self.clock.time

        self.log_state_transition(new_state.i)
        self.state = new_state
        print(f"-- State switched to {new_state.name}")

    async def takeoffStep(self):
        # if not self.landed_state == LandedState.IN_AIR:
        #     return
        if not self.altitude > 9.5:
            return
        self.stepTimer.blockSignals(True)
        await self.trans_takeoff2new_position()
        self.stepTimer.blockSignals(False)

    async def trans_takeoff2new_position(self):
        await self.trans2new_position()

    def random_point(self):
        angle = random.uniform(0, 2 * math.pi)

        yaw = random.uniform(-180, 180)
        
        h = random.uniform(0.5, 11)

        a = 0.35 * h
        x = a * math.cos(angle)
        y = a * math.sin(angle)
        
        return (x, y, h, yaw)

    async def trans2new_position(self):
        new_state = self.states[3]
        print(f"-- Switching the state from {self.state.name} to {new_state.name}")
        self.log_state_transition(new_state.i)
        self.state = new_state
        print(f"-- State switched to {new_state.name}")

        async for terrain_info in self.drone.telemetry.home():
            absolute_altitude = terrain_info.absolute_altitude_m
            break

        new_point_rel_x, new_point_rel_y, h, new_point_yaw = self.random_point()   
        x = self.mission["plosina"]["x"] + new_point_rel_x
        y = self.mission["plosina"]["y"] + new_point_rel_y        
        yaw = new_point_yaw

        lat_deg, lon_deg = self.add_relative_distance(self.cfg["origin"]["lat_deg"], self.cfg["origin"]["lon_deg"], self.cfg["origin"]["elevation"], y, x)

        print(f"-- Flying over to the target {h} m above the ground.")
        try:
            await self.drone.action.goto_location(lat_deg, lon_deg, h+absolute_altitude, yaw)
        except ActionError as e:
            print(e)                                # TODO: handle error

        while not self.clock.start_timer(5.0):
            await asyncio.sleep(0.1)
        self.clock.time_elapsed.connect(self.new_positionClockStep)
        self.cam.new_frame.connect(self.new_positionCameraStep)

    async def hoverStep(self):
        self.stepTimer.blockSignals(True)
        await self.trans_hover2new_position()

    @asyncSlot(None)
    async def new_positionClockStep(self):
        self.clock.time_elapsed.disconnect()
        await self.trans_new_position2hover()
        self.stepTimer.blockSignals(False)

    def compute_heading(self, roll, pitch, yaw):
        # Convert angles to radians
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_rad = math.radians(yaw)

        # Compute heading (rotation in ground plane)
        heading_rad = yaw_rad + math.atan2(math.sin(roll_rad) * math.sin(yaw_rad), 
                                        math.cos(roll_rad)) * math.cos(yaw_rad)

        # Convert heading to degrees
        heading_deg = math.degrees(heading_rad)

        # Normalize heading to be within [0, 360) degrees
        heading_deg = (heading_deg + 180) % 360 - 180

        return heading_deg
    
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

    @asyncSlot(object, object, object, object, float, qtg.QImage)
    async def new_positionCameraStep(self, x, y, z, yaw, t, frame):
        if x is None:
            return
        if random.uniform(0,1) <= 0.95: # take only 5% of images
            return
        
        yaw = (-yaw - 90 + 180)%360-180
        x = +self.altitude * tan(pipul - atan(z/x) - radians(self.roll_deg))
        y = -self.altitude * tan(pipul - atan(z/y) - radians(self.pitch_deg))
        x, y = self.rotate_point(x, y, -self.compute_heading(self.roll_deg, self.pitch_deg, yaw))
        x = self.mission["plosina"]["x"] - x
        y = self.mission["plosina"]["y"] - y
        z = self.altitude + self.cfg["camera"]["height_offset"]
        

        self.measured_positions.append((float(x), float(y), float(z), float(yaw)))

        x = self.UAVPose.pose.position.x
        y = self.UAVPose.pose.position.y
        z = self.UAVPose.pose.position.z
        yaw = self.quat_to_yaw(self.UAVPose.pose.orientation)
        self.real_positions.append((x, y, z, yaw))
        
        i = len(self.measured_positions)
        print(f"-- Measured position {i}/{self.N}")
        if i >= self.N:
            await self.stop()

    async def new_positionStep(self):
        pass

    async def trans_new_position2hover(self):
        new_state = self.states[2]
        print(f"-- Switching the state from {self.state.name} to {new_state.name}")
        self.log_state_transition(new_state.i)
        self.state = new_state
        print(f"-- State switched to {new_state.name}")

    async def trans_hover2new_position(self):
        await self.trans2new_position()

    def quat_to_yaw(self, q):
        yaw = (270 - math.degrees(math.atan2(2.0 * (q.z * q.w + q.x * q.y) , - 1.0 + 2.0 * (q.w * q.w + q.x * q.x))))%360 - 180
        return yaw
    
class FixMeasurement(EstimateMeasurementCovariance):
    def __init__(self, cfg, name="Oprava mereni kamerou", N=400) -> None:
        super().__init__(cfg, name, N)
        self.p_i = 0
        self.points = [[2, 2], [2, -2], [-2, -2], [-2, 2]]

    def random_point(self):
        # yaw = random.uniform(-180, 180)
        yaw = 90
        h = 6
        x, y = tuple(self.points[self.p_i])
        self.p_i+=1
        self.p_i%=4
        
        return (x, y, h, yaw)