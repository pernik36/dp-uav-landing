from math import atan, radians, tan, pi
import math
import os
import random
import sys
import time

import grpc
import numpy as np
from gzUAVPose import gzUAVPose
pipul = pi/2
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.telemetry import LandedState, FlightMode, TelemetryError
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

from filterpy.kalman import KalmanFilter

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
    
    def quat_to_yaw(self, q):
        yaw = (270 - math.degrees(math.atan2(2.0 * (q.z * q.w - q.x * q.y) , (q.w * q.w + q.x * q.x - q.y*q.y - q.z*q.z))))%360 - 180
        return yaw
    
    def convert_time(self, time):
        return time.sec + time.nsec/1000000000

class AlgsModel(qtc.QAbstractListModel):
    def __init__(self, cfg, algsList: list[landingAlg] | None = None):
        super().__init__()
        self.list = algsList or [DetectionTest(cfg), KalmanOffboardPIDAngled(cfg, "KAP1", 1), OffboardPIDAngled(cfg, "_AP1", 1), KalmanOffboardPID(cfg, "K_P1", 1), OffboardPID(cfg, "__P1", 1), KalmanOffboardPIDAngled(cfg), OffboardPIDAngled(cfg), KalmanOffboardPID(cfg), FixMeasurement(cfg), EstimateMeasurementCovariance(cfg, N=400), EstimateCovariance(cfg, N=200), TakeoffAndLand(cfg), TakeoffAndLand(cfg, "Vzleť a Přistaň 3m", h=3), mavlink_test(cfg), OffboardPID(cfg, name="P0.5"), OffboardPID(cfg, name="P0.8 I0.01", k_p=0.8, k_i=0.01), OffboardPID(cfg, name="P2 I0.2", k_p=2, k_i=0.2)]

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
    step_signal = qtc.Signal(str)
    ae_signal = qtc.Signal(object, float)
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
        await self.run_preparation(mission, uav_pl_rel_x, uav_pl_rel_y, camera)
        self.start_timer()

    def start_timer(self):
        self.stepTimer = qtc.QTimer()
        self.stepTimer.timeout.connect(self.step)
        self.stepTimer.start(100)

    async def run_preparation(self, mission, uav_pl_rel_x, uav_pl_rel_y, camera: gzCamera):
        self.step_signal.emit("Příprava.")
        await asyncio.sleep(4)
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
        self.state_transitions = []

        try:
            await self.drone.telemetry.set_rate_attitude(30)
            await self.drone.telemetry.set_rate_position(30)
        except TelemetryError as e:
            print("-- Failed to set update rates.")
            print(e)

        self.tasks.append(asyncio.ensure_future(self.subscribe_attitude()))
        self.tasks.append(asyncio.ensure_future(self.subscribe_altitude()))
        self.tasks.append(asyncio.ensure_future(self.subscribe_flight_mode()))
        self.tasks.append(asyncio.ensure_future(self.subscribe_landed_state()))

        self.cam = camera

        self.uav_pl_rel_x = uav_pl_rel_x
        self.uav_pl_rel_y = uav_pl_rel_y

        self.pid_p.set_auto_mode(False)
        self.pid_r.set_auto_mode(False)
        self.pid_y.set_auto_mode(False)

        self.mae = np.zeros(3)
        self.steps_wo_vis = 0
        self.total_steps = 0
        self.total_steps_wo_vision = 0
        self.average_duration = 0

    @asyncSlot(None)
    async def step(self):
        if (len(self.state_transitions) > 0 and self.clock.time.sim.sec - self.state_transitions[-1].time.sec > 60) or (len(self.state_transitions) == 0 and self.clock.time.sim.sec > 60):
            os.execl(sys.executable, 'python3', __file__, *sys.argv[1:])
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
        extras = {"mae": self.mae.tolist(), "mean_duration": self.average_duration, "Nsteps": self.total_steps, "Nsteps_wo_vision": self.total_steps_wo_vision}
        self.ended.emit(MissionResult(self.mission, self.start_time, self.clock.time, self.UAVPose.pose, self.state_transitions, extras))
        self.step_signal.emit(f"Konec. \nMAE:\n  mae_x: {self.mae[0]:6.2f} m,\n  mae_y: {self.mae[1]:6.2f} m,\n  mae_yaw: {self.mae[2]:6.2f} °\nPočet kroků: {self.total_steps}\n  z toho bez detekce: {self.total_steps_wo_vision}\nStřední doba kroku: {self.average_duration:8.4f} s")
        self.step_signal.disconnect()

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
        self.step_signal.emit("UAV zjišťuje polohu.")
        self.stepTimer.blockSignals(True)
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                break
            else:
                self.stepTimer.blockSignals(False)
                return
            
        print("-- Global position state is good enough for flying.")
        self.step_signal.emit("Poloha zjištěna.")
        await self.trans_preflight2takeoff()
        self.stepTimer.blockSignals(False)

    async def trans_preflight2takeoff(self):
        new_state = self.states[1]
        print(f"-- Switching the state from {self.state.name} to {new_state.name}")
        print("-- Arming")
        self.step_signal.emit("Armování UAV.")
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
        await self.drone.action.set_takeoff_altitude(self.cfg["uav_h"])
        await self.drone.action.takeoff()
        self.start_time = self.clock.time

        self.log_state_transition(new_state.i)
        self.state = new_state
        print(f"-- State switched to {new_state.name}")

    async def takeoffStep(self):
        self.step_signal.emit(f"Vzlet. Čeká se na výšku {self.cfg['uav_h']} m\nSoučasná výška: {self.altitude:6.2f} m")
        if not self.landed_state == LandedState.IN_AIR:
            return
        self.stepTimer.blockSignals(True)
        await self.trans_takeoff2find_target()

    async def trans_takeoff2find_target(self):
        await self.trans2find_target()

    async def get_position_over_tag(self):
        async for terrain_info in self.drone.telemetry.home():
            absolute_altitude = terrain_info.absolute_altitude_m
            home_lat_deg = terrain_info.latitude_deg
            home_lon_deg = terrain_info.longitude_deg
            break
        pl_lat_deg, pl_lon_deg = self.add_relative_distance(home_lat_deg, home_lon_deg, absolute_altitude, self.uav_pl_rel_y, self.uav_pl_rel_x)
        return pl_lat_deg, pl_lon_deg, absolute_altitude + self.cfg["uav_h"]

    async def trans2find_target(self):
        new_state = self.states[3]
        print(f"-- Switching the state from {self.state.name} to {new_state.name}")
        self.log_state_transition(new_state.i)
        self.state = new_state
        print(f"-- State switched to {new_state.name}")

        pl_lat_deg, pl_lon_deg, absolute_altitude = await self.get_position_over_tag()

        h = self.cfg["uav_h"]

        print(f"-- Flying over to the target {h} m above the ground.")
        self.step_signal.emit("Přelet do blízkosti plošiny.")
        try:
            await self.drone.action.goto_location(pl_lat_deg, pl_lon_deg, absolute_altitude, self.mission["uav"]["phi"])
        except ActionError as e:
            print(e)                                # TODO: handle error

        try:
            async for position in self.drone.telemetry.position():
                if abs(position.latitude_deg - pl_lat_deg) < 0.000009 and abs(position.longitude_deg - pl_lon_deg) < 0.000009: # cca 1m
                    break
        except _MultiThreadedRendezvous:
            pass
        print("-- Over the target. Trying to localize AprilTag.")
        self.step_signal.emit("V blízkosti plošiny. Hledání značky.")

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

    @asyncSlot(object, object, object, object, float, qtg.QImage, float)
    async def wait_for_apriltag(self, x, y, z, yaw, t, frame, step_start):
        if x is not None:
            self.cam.new_frame.disconnect(self.wait_for_apriltag)
            self.steps_wo_vis = 0
            self.start_searching = None
            print("-- Found an AprilTag.")
            self.step_signal.emit("V blízkosti plošiny, značka nalezena.")
            await self.trans_find_target2centering()
            return
        else:
            self.steps_wo_vis += 1
            if self.steps_wo_vis>=6*self.cfg["max_steps_wo_visual"]:
                self.step_signal.emit("Značka nenalezena. Přistávání selhalo.")
                await self.stop()
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
                self.log_state_transition(new_state.i)
                self.state = new_state
                print("-- State switched to centering")
                return
            except OffboardError as error:
                print(f"   Starting offboard mode failed with error code: {error._result.result}")
                print("   Retrying in", delay)
                await asyncio.sleep(delay)
                delay *= 1.5

        print("   Failed to start the offboard mode") # TODO: Osetrit, co v tomto pripade delat

    @asyncSlot(object, object, object, object, float, qtg.QImage, float)
    async def centeringCamStep(self, x, y, z, yaw, t, frame, step_start):
        if self.last_t is None:
            self.last_t = t
            return
        if self.altitude < self.cfg["landing_trigger_alt"]:
            await self.trans_centering2land()
            return
        
        self.total_steps += 1

        sp_v_p = 0
        sp_v_r = 0
        sp_v_d = -0.3
        sp_v_y = 0

        if x is not None:
            dt = t - self.last_t
            self.steps_wo_vis = 0
            d_r, d_p, d_h, d_yaw = await self.relative_dist(x, y, z, yaw, t)

            self.last_t = t

            sp_v_r = self.pid_r(-d_r, dt)
            sp_v_p = self.pid_p(-d_p, dt)
            sp_v_d = 0.5/(1+12.0/d_h*(d_r*d_r + d_p*d_p))
            sp_v_y = self.pid_y(d_yaw, dt)

            print(f"v_p: {sp_v_p:6.2f}, v_r: {sp_v_r:6.2f}, v_yaw: {sp_v_y:6.2f}", end="\r")

        else:
            self.step_signal.emit(f"Přistávání. Značka nedetekována.\nPožadované rychlosti:\n  v_r: {sp_v_r:6.2f} m/s,\n  v_p: {sp_v_p:6.2f} m/s,\n v_yaw: {sp_v_y:6.2f} °/s\nMAE:\n  mae_x: {self.mae[0]:6.2f} m,\n  mae_y: {self.mae[1]:6.2f} m,\n  mae_yaw: {self.mae[2]:6.2f} °\nPočet kroků: {self.total_steps}\n  z toho bez detekce: {self.total_steps_wo_vision}\nStřední doba kroku: {self.average_duration:8.4f} s")
            self.steps_wo_vis += 1
            self.total_steps_wo_vision += 1
            if self.steps_wo_vis>=self.cfg["max_steps_wo_visual"]:
                await self.trans_centering2find_target()
                return

        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(sp_v_p, sp_v_r, sp_v_d, sp_v_y))
        step_end = time.perf_counter()
        duration = step_end - step_start
        self.average_duration = (self.average_duration * (self.total_steps - 1) + duration)/self.total_steps
        if x is not None:
            valid_steps = (self.total_steps - self.total_steps_wo_vision)
            error = self.absolute_distance(d_h)
            self.ae_signal.emit(error, self.clock.time_to_float(self.clock.time.sim))
            absolute_error = np.abs(error)
            self.mae = (self.mae * (valid_steps - 1) + absolute_error)/valid_steps
            self.step_signal.emit(f"Přistávání.\nPožadované rychlosti:\n  v_r: {sp_v_r:6.2f} m/s,\n  v_p: {sp_v_p:6.2f} m/s,\n v_yaw: {sp_v_y:6.2f} °/s\nMAE:\n  mae_x: {self.mae[0]:6.2f} m,\n  mae_y: {self.mae[1]:6.2f} m,\n  mae_yaw: {self.mae[2]:6.2f} °\nPočet kroků: {self.total_steps}\n  z toho bez detekce: {self.total_steps_wo_vision}\nStřední doba kroku: {self.average_duration:8.4f} s")

    async def relative_dist(self, x, y, z, yaw, t):
        d_r = self.altitude * tan(pipul - atan(z/x) - radians(self.roll_deg))
        d_p = -self.altitude * tan(pipul - atan(z/y) - radians(self.pitch_deg))
        d_yaw = (-yaw - 90 + 180)%360-180

        return d_r, d_p, self.altitude, d_yaw
    
    def absolute_distance(self, h):
        e_x = self.UAVPose.pose.position.x - self.mission["plosina"]["x"]
        e_y = self.UAVPose.pose.position.y - self.mission["plosina"]["y"]
        e_yaw = (self.quat_to_yaw(self.UAVPose.pose.orientation) - self.mission["plosina"]["phi"] + 180)%360 - 180

        return np.array([e_x, e_y, e_yaw])

    async def trans_centering2find_target(self):
        self.cam.new_frame.disconnect(self.centeringCamStep)
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0.0))
        new_state = self.states[3]
        self.log_state_transition(new_state.i)
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
        self.log_state_transition(new_state.i)
        self.state = new_state
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0.3, 0.0))
        self.stepTimer.blockSignals(False)
        print(f"-- State switched to {new_state.name}")

    async def centeringStep(self):
        pass

    async def landingStep(self):
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 2, 0.0))
        print(self.altitude, end="\r")
        self.step_signal.emit(f"Dosedávání. \nMAE:\n  mae_x: {self.mae[0]:6.2f} m,\n  mae_y: {self.mae[1]:6.2f} m,\n  mae_yaw: {self.mae[2]:6.2f} °\nPočet kroků: {self.total_steps}\n  z toho bez detekce: {self.total_steps_wo_vision}\nStřední doba kroku: {self.average_duration} s")
        if not self.landed_state == LandedState.ON_GROUND:
            return
        self.stepTimer.blockSignals(True)
        print("-- Landed.")
        self.step_signal.emit(f"Přistáno. \nMAE:\n  mae_x: {self.mae[0]:6.2f} m,\n  mae_y: {self.mae[1]:6.2f} m,\n  mae_yaw: {self.mae[2]:6.2f} °\nPočet kroků: {self.total_steps}\n  z toho bez detekce: {self.total_steps_wo_vision}\nStřední doba kroku: {self.average_duration} s")
        await self.stop()

class KalmanOffboardPID(OffboardPID):
    def __init__(self, cfg, name="Kálmánův filtr P=0.5", k_p=0.5, k_i=0, k_d=0, P = None, Q=None, R=None) -> None:
        """
        * `cfg` config dictionary
        * `name` name of the alg for UI
        * `k_p` PID proportional gain
        * `k_i` PID integral gain
        * `k_d` PID derivative gain
        * `P` state covariance
        * `Q` process noise covariance
        * `R` measurement noise covariance
        """
        super().__init__(cfg, name, k_p, k_i, k_d)
        self.P = P if P is not None else self.cfg["kalman"]["P"]
        self.Q = Q if Q is not None else self.cfg["kalman"]["Q"]
        self.R = R if R is not None else self.cfg["kalman"]["R"]
        self.F = np.array(self.cfg["kalman"]["F"])
        self.H = np.array(self.cfg["kalman"]["H"])
        self.kf = KalmanFilter(8, 4, 4)

    async def subscribe_angular_velocity(self):
        try:
            async for velo in self.drone.telemetry.attitude_angular_velocity_body():
                v_yaw = math.degrees(velo.yaw_rad_s)
                self.a_yaw = self.v_yaw - v_yaw
                self.v_yaw = v_yaw
        except _MultiThreadedRendezvous:
            pass

    async def subscribe_imu(self):
        try:
            async for imu in self.drone.telemetry.imu():
                self.imu = imu
        except _MultiThreadedRendezvous:
            pass

    @asyncSlot(dict, float, float, gzCamera)
    async def run(self, mission, uav_pl_rel_x, uav_pl_rel_y, camera: gzCamera):
        await self.run_preparation(mission, uav_pl_rel_x, uav_pl_rel_y, camera)
        self.v_yaw = 0
        self.tasks.append(asyncio.ensure_future(self.subscribe_angular_velocity()))
        self.tasks.append(asyncio.ensure_future(self.subscribe_imu()))
        try:
            await self.drone.telemetry.set_rate_imu(30)
        except TelemetryError as e:
            print("-- Failed to set IMU update rate.")
            print(e)
        self.set_kalman_params()
        self.start_timer()

    def set_kalman_params(self):
        self.kf.P = np.array(self.P)
        self.kf.Q = np.array(self.Q)
        self.kf.R = np.array(self.R)
        self.kf.F = np.array(self.F)
        self.kf.H = np.array(self.H)

        self.kf.x = np.array([0, 0, 0, 0, 0, 0, 0, 0])

    async def relative_dist(self, x, y, z, yaw, t):
        d_r = self.altitude * tan(pipul - atan(z/x) - radians(self.roll_deg))
        d_p = -self.altitude * tan(pipul - atan(z/y) - radians(self.pitch_deg))
        d_yaw = (-yaw - 90 + 180)%360-180
        
        accel = self.imu.acceleration_frd
        u = np.array([accel.right_m_s2, accel.forward_m_s2, -accel.down_m_s2, self.a_yaw])

        z = np.array([d_r, d_p, self.altitude, d_yaw])

        self.kf.predict(u)
        self.kf.update(z)

        return self.get_filtered_state()
    
    def get_filtered_state(self):
        x = self.kf.x[0]
        y = self.kf.x[2]
        z = self.kf.x[4]
        yaw = self.kf.x[6]

        return x, y, z, yaw
    
class OffboardPIDAngled(OffboardPID):
    def __init__(self, cfg, name="Se sklonem P=0.5", k_p=0.5, k_i=0, k_d=0, N_samples=150) -> None:
        self.N = N_samples
        super().__init__(cfg, name, k_p, k_i, k_d)

    async def trans_takeoff2find_target(self):
        pitch_measurements = []
        roll_measurements = []
        async for pos in self.drone.telemetry.position():
            lat = pos.latitude_deg
            lon = pos.longitude_deg
            alt = pos.absolute_altitude_m
            break
        yaw = 0
        try:
            await self.drone.action.goto_location(lat, lon, alt, yaw)
        except ActionError as e:
            print(e)                                # TODO: handle error
        await asyncio.sleep(2)
        print("--Measuring mean roll and pitch needed to hold position in this wind.")

        while len(pitch_measurements) < self.N:
            pitch_measurements.append(self.pitch_deg)
            roll_measurements.append(self.roll_deg)
            await asyncio.sleep(0.03)

        pitch_wind = np.array(pitch_measurements).mean()
        roll_wind = np.array(roll_measurements).mean()

        self.rate_x = math.tan(math.radians(roll_wind))
        self.rate_y = math.tan(math.radians(pitch_wind))
        print(f"  Measured rates: x: {self.rate_x}, y: {self.rate_y}")
        return await super().trans_takeoff2find_target()
    
    async def relative_dist(self, x, y, z, yaw, t):
        d_r, d_p, h, d_yaw = await super().relative_dist(x, y, z, yaw, t)
        r_t, p_t = await self.rotate_point(self.rate_x, self.rate_y, (self.yaw_deg + 180)%360-180)
        d_r -= max(h-1.2*self.cfg["landing_trigger_alt"],0)*r_t
        d_p -= max(h-1.2*self.cfg["landing_trigger_alt"],0)*p_t

        return d_r, d_p, h, d_yaw
    
    def absolute_distance(self, h):
        e_x = self.UAVPose.pose.position.x - (self.mission["plosina"]["x"] - self.rate_x*max(h-1.2*self.cfg["landing_trigger_alt"],0))
        e_y = self.UAVPose.pose.position.y - (self.mission["plosina"]["y"] - self.rate_y*max(h-1.2*self.cfg["landing_trigger_alt"],0))
        e_yaw = (self.quat_to_yaw(self.UAVPose.pose.orientation) - self.mission["plosina"]["phi"] + 180)%360 - 180

        return np.array([e_x, e_y, e_yaw])
    
    async def rotate_point(self, x, y, angle):
        angle = math.radians(angle)
        sin_angle = math.sin(angle)
        cos_angle = math.cos(angle)
        
        x_rotated = x * cos_angle - y * sin_angle
        y_rotated = x * sin_angle + y * cos_angle
        
        return x_rotated, y_rotated
    
    async def get_position_over_tag(self):
        pl_lat_deg, pl_lon_deg, alt = await super().get_position_over_tag()
        lat_deg, lon_deg = self.add_relative_distance(pl_lat_deg, pl_lon_deg, alt, -self.rate_y*self.cfg["uav_h"], -self.rate_x*self.cfg["uav_h"])
        return lat_deg, lon_deg, alt
    
class KalmanOffboardPIDAngled(KalmanOffboardPID):
    def __init__(self, cfg, name="Kálmánův filtr se sklonem P=0.5", k_p=0.5, k_i=0, k_d=0, P=None, Q=None, R=None, N_samples=150) -> None:
        self.N = N_samples
        super().__init__(cfg, name, k_p, k_i, k_d, P, Q, R)

    async def trans_takeoff2find_target(self):
        pitch_measurements = []
        roll_measurements = []
        async for pos in self.drone.telemetry.position():
            lat = pos.latitude_deg
            lon = pos.longitude_deg
            alt = pos.absolute_altitude_m
            break
        yaw = 0
        try:
            await self.drone.action.goto_location(lat, lon, alt, yaw)
        except ActionError as e:
            print(e)                                # TODO: handle error
        await asyncio.sleep(2)
        print("--Measuring mean roll and pitch needed to hold position in this wind.")

        while len(pitch_measurements) < self.N:
            pitch_measurements.append(self.pitch_deg)
            roll_measurements.append(self.roll_deg)
            await asyncio.sleep(0.03)

        pitch_wind = np.array(pitch_measurements).mean()
        roll_wind = np.array(roll_measurements).mean()

        self.rate_x = math.tan(math.radians(roll_wind))
        self.rate_y = math.tan(math.radians(pitch_wind))
        print(f"  Measured rates: x: {self.rate_x}, y: {self.rate_y}")
        return await super().trans_takeoff2find_target()
    
    async def relative_dist(self, x, y, z, yaw, t):
        d_r, d_p, h, d_y = await super().relative_dist(x, y, z, yaw, t)
    
        r_t, p_t = await self.rotate_point(self.rate_x, self.rate_y, (self.yaw_deg + 180)%360-180)
        d_r -= max(h-1.2*self.cfg["landing_trigger_alt"],0)*r_t
        d_p -= max(h-1.2*self.cfg["landing_trigger_alt"],0)*p_t

        return d_r, d_p, h, d_y
    
    def absolute_distance(self, h):
        e_x = self.UAVPose.pose.position.x - (self.mission["plosina"]["x"] - self.rate_x*max(h-1.2*self.cfg["landing_trigger_alt"],0))
        e_y = self.UAVPose.pose.position.y - (self.mission["plosina"]["y"] - self.rate_y*max(h-1.2*self.cfg["landing_trigger_alt"],0))
        e_yaw = (self.quat_to_yaw(self.UAVPose.pose.orientation) - self.mission["plosina"]["phi"] + 180)%360 - 180

        return np.array([e_x, e_y, e_yaw])
    
    async def rotate_point(self, x, y, angle):
        angle = math.radians(angle)
        sin_angle = math.sin(angle)
        cos_angle = math.cos(angle)
        
        x_rotated = x * cos_angle - y * sin_angle
        y_rotated = x * sin_angle + y * cos_angle
        
        return x_rotated, y_rotated
    
    async def get_position_over_tag(self):
        pl_lat_deg, pl_lon_deg, alt = await super().get_position_over_tag()
        lat_deg, lon_deg = self.add_relative_distance(pl_lat_deg, pl_lon_deg, alt, -self.rate_y*self.cfg["uav_h"], -self.rate_x*self.cfg["uav_h"])
        return lat_deg, lon_deg, alt

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

                # print(f"Pitch: {self.pitch_deg:6.2f}, Roll: {self.roll_deg:6.2f}, Yaw: {self.yaw_deg:6.2f}", end="\r")
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

    @asyncSlot(object, object, object, object, float, qtg.QImage, float)
    async def new_positionCameraStep(self, x, y, z, yaw, t, frame, step_start):
        if x is None:
            return
        if random.uniform(0,1) <= 0.97: # take only 3% of images
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
        yaw = random.uniform(-180, 180)
        # yaw = 90
        h = 7.5
        x, y = tuple(self.points[self.p_i])
        self.p_i+=1
        self.p_i%=4
        
        return (x, y, h, yaw)
    
class CountUnseenTags(landingAlg, qtc.QObject):
    ended = qtc.Signal(MissionResult)
    def __init__(self, cfg, name = "Odhad podílu nedetekovaných") -> None:
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

    @asyncSlot(object, object, object, object, float, qtg.QImage, float)
    async def new_positionCameraStep(self, x, y, z, yaw, t, frame, step_start):
        if x is None:
            return
        if random.uniform(0,1) <= 0.97: # take only 3% of images
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
    
class DetectionTest(landingAlg, qtc.QObject):
    ended = qtc.Signal(MissionResult)
    step_signal = qtc.Signal(str)
    ae_signal = qtc.Signal(object, float)
    def __init__(self, cfg, name = "Test detekce", base_h = 9.5, step_h = 0.25, max_h = 15) -> None:
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
                      ,algState(4, "centering", self.testingStep)        # --> 4
                      ]
        self.state = self.states[0]
        self.stateTransitions = [[None for s in self.states] for t in self.states]
        self.stateTransitions[0][1] = self.trans_preflight2takeoff
        self.stateTransitions[1][3] = self.trans_takeoff2find_target
        self.stateTransitions[3][4] = self.trans_find_target2testing

        self.lt = float("infinity")
        self.heights = list(np.linspace(base_h, max_h, int((max_h-base_h)/step_h + 1)))
        print(self.heights)

    @asyncSlot(dict, float, float, gzCamera)
    async def run(self, mission, uav_pl_rel_x, uav_pl_rel_y, camera: gzCamera):
        await self.run_preparation(mission, uav_pl_rel_x, uav_pl_rel_y, camera)
        self.start_timer()

    def start_timer(self):
        self.stepTimer = qtc.QTimer()
        self.stepTimer.timeout.connect(self.step)
        self.stepTimer.start(100)
        self.watchdogTimer = qtc.QTimer()
        self.watchdogTimer.timeout.connect(self.watchdog_step)
        self.watchdogTimer.start(10000)

    def watchdog_step(self):
        if ((len(self.state_transitions) > 0 and self.clock.time.sim.sec - self.state_transitions[-1].time.sec > 120) or (len(self.state_transitions) == 0 and self.clock.time.sim.sec > 60)):
            print("Končím.")
            os.execl(sys.executable, 'python3', __file__, *sys.argv[1:])

    async def run_preparation(self, mission, uav_pl_rel_x, uav_pl_rel_y, camera: gzCamera):
        self.step_signal.emit("Příprava.")
        await asyncio.sleep(4)
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
        self.state_transitions = []

        try:
            await self.drone.telemetry.set_rate_attitude(30)
            await self.drone.telemetry.set_rate_position(30)
        except TelemetryError as e:
            print("-- Failed to set update rates.")
            print(e)

        self.tasks.append(asyncio.ensure_future(self.subscribe_attitude()))
        self.tasks.append(asyncio.ensure_future(self.subscribe_altitude()))
        self.tasks.append(asyncio.ensure_future(self.subscribe_flight_mode()))
        self.tasks.append(asyncio.ensure_future(self.subscribe_landed_state()))

        self.cam = camera

        self.uav_pl_rel_x = uav_pl_rel_x
        self.uav_pl_rel_y = uav_pl_rel_y

        self.steps_wo_vis = 0
        self.total_steps = [0 for h in self.heights]
        self.total_steps_wo_vision = [0 for h in self.heights]
        self.hi = 0

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
        self.watchdogTimer.stop()
        try:
            self.cam.new_frame.disconnect()
        except RuntimeError: # Already disconnected
            pass
        extras = {"Nsteps": self.total_steps, "Nsteps_wo_vision": self.total_steps_wo_vision, "heights": self.heights}
        self.ended.emit(MissionResult(self.mission, self.start_time, self.clock.time, self.UAVPose.pose, self.state_transitions, extras))
        self.step_signal.emit(f"Konec. \nPočet kroků: {self.total_steps}\n  z toho bez detekce: {self.total_steps_wo_vision}")
        self.step_signal.disconnect()

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
        self.step_signal.emit("UAV zjišťuje polohu.")
        self.stepTimer.blockSignals(True)
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                break
            else:
                self.stepTimer.blockSignals(False)
                return
            
        print("-- Global position state is good enough for flying.")
        self.step_signal.emit("Poloha zjištěna.")
        await self.trans_preflight2takeoff()
        self.stepTimer.blockSignals(False)

    async def trans_preflight2takeoff(self):
        new_state = self.states[1]
        print(f"-- Switching the state from {self.state.name} to {new_state.name}")
        print("-- Arming")
        self.step_signal.emit("Armování UAV.")
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
        await self.drone.action.set_takeoff_altitude(self.cfg["uav_h"])
        await self.drone.action.takeoff()
        self.start_time = self.clock.time

        self.log_state_transition(new_state.i)
        self.state = new_state
        print(f"-- State switched to {new_state.name}")

    async def takeoffStep(self):
        self.step_signal.emit(f"Vzlet. Čeká se na výšku {self.cfg['uav_h']} m\nSoučasná výška: {self.altitude:6.2f} m")
        if not self.landed_state == LandedState.IN_AIR:
            return
        self.stepTimer.blockSignals(True)
        await self.trans_takeoff2find_target()

    async def trans_takeoff2find_target(self):
        await self.trans2find_target()

    async def get_position_over_tag(self):
        async for terrain_info in self.drone.telemetry.home():
            self.absolute_altitude = terrain_info.absolute_altitude_m
            home_lat_deg = terrain_info.latitude_deg
            home_lon_deg = terrain_info.longitude_deg
            break
        pl_lat_deg, pl_lon_deg = self.add_relative_distance(home_lat_deg, home_lon_deg, self.absolute_altitude, self.uav_pl_rel_y, self.uav_pl_rel_x)
        return pl_lat_deg, pl_lon_deg, self.absolute_altitude + self.heights[self.hi]

    async def trans2find_target(self):
        new_state = self.states[3]
        print(f"-- Switching the state from {self.state.name} to {new_state.name}")
        self.log_state_transition(new_state.i)
        self.state = new_state
        print(f"-- State switched to {new_state.name}")

        self.pl_lat_deg, self.pl_lon_deg, absolute_altitude = await self.get_position_over_tag()

        h = self.cfg["uav_h"]

        print(f"-- Flying over to the target {h} m above the ground.")
        self.step_signal.emit("Přelet do blízkosti plošiny.")
        try:
            await self.drone.action.goto_location(self.pl_lat_deg, self.pl_lon_deg, absolute_altitude, self.mission["uav"]["phi"])
        except ActionError as e:
            print(e)                                # TODO: handle error

        try:
            async for position in self.drone.telemetry.position():
                if abs(position.latitude_deg - self.pl_lat_deg) < 0.000009 and abs(position.longitude_deg - self.pl_lon_deg) < 0.000009: # cca 1m
                    break
        except _MultiThreadedRendezvous:
            pass
        print("-- Over the target. Trying to localize AprilTag.")
        self.step_signal.emit("V blízkosti plošiny. Hledání značky.")

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

    @asyncSlot(object, object, object, object, float, qtg.QImage, float)
    async def wait_for_apriltag(self, x, y, z, yaw, t, frame, step_start):
        if x is not None:
            self.cam.new_frame.disconnect(self.wait_for_apriltag)
            self.steps_wo_vis = 0
            self.start_searching = None
            print("-- Found an AprilTag.")
            self.step_signal.emit("V blízkosti plošiny, značka nalezena.")
            await self.trans_find_target2testing()
            return
        else:
            self.steps_wo_vis += 1
            if self.steps_wo_vis>=6*self.cfg["max_steps_wo_visual"]:
                self.step_signal.emit("Značka nenalezena. Přistávání selhalo.")
                await self.stop()
                return
        print("-- AprilTag not found in the image.", end="\r")

    async def trans_find_target2testing(self):
        new_state = self.states[4]
        print(f"-- Switching the state from {self.state.name} to {new_state.name}")
        print("-- Setting initial offboard control setpoint")
        
        self.cam.new_frame.connect(self.testingCamStep)
        self.log_state_transition(new_state.i)
        self.state = new_state
        self.stepTimer.blockSignals(False)
        print("-- State switched to centering")

    @asyncSlot(object, object, object, object, float, qtg.QImage, float)
    async def testingCamStep(self, x, y, z, yaw, t, frame, step_start):
        if random.random() < 0.7:
            return
        if self.last_t is None:
            self.last_t = t
            return
        
        self.total_steps[self.hi] += 1

        if x is not None:
            dt = t - self.last_t
            self.steps_wo_vis = 0
            self.last_t = t

        else:
            self.step_signal.emit(f"Značka nedetekována. Ve výšce {self.altitude} m:\n  Počet kroků: {self.total_steps[self.hi]}\n  z toho bez detekce: {self.total_steps_wo_vision[self.hi]}")
            self.steps_wo_vis += 1
            self.total_steps_wo_vision[self.hi] += 1
            if self.steps_wo_vis>=4*self.cfg["max_steps_wo_visual"]:
                self.stepTimer.blockSignals(True)
                self.step_signal.emit(f"Konec testu.")
                await self.stop()
                return

        if x is not None:
            self.step_signal.emit(f"Značka detekována. Ve výšce {self.altitude} m:\n  Počet kroků: {self.total_steps[self.hi]}\n  z toho bez detekce: {self.total_steps_wo_vision[self.hi]}")

    async def testingStep(self):
        self.stepTimer.blockSignals(True)
        if self.total_steps[self.hi] > 400:
            self.cam.new_frame.disconnect(self.testingCamStep)
            self.hi+=1
            if self.hi >= len(self.heights) - 1:
                self.stepTimer.blockSignals(True)
                self.step_signal.emit(f"Konec testu.")
                await self.stop()
                return
            h = self.heights[self.hi]
            self.log_state_transition(4)
            try:
                await self.drone.action.goto_location(self.pl_lat_deg, self.pl_lon_deg, self.absolute_altitude + h, self.mission["uav"]["phi"])
            except ActionError as e:
                print(e)                                # TODO: handle error
            await asyncio.sleep(5)
            self.cam.new_frame.connect(self.testingCamStep)
            
        h = self.heights[self.hi]
        new_lat_deg, new_lon_deg = self.add_relative_distance(self.pl_lat_deg, self.pl_lon_deg, self.absolute_altitude, random.uniform(-1,1), random.uniform(-1,1))
        try:
            await self.drone.action.goto_location(new_lat_deg, new_lon_deg, self.absolute_altitude + h, random.uniform(-180,180))
        except ActionError as e:
            print(e)                                # TODO: handle error
        await asyncio.sleep(5)
        self.stepTimer.blockSignals(False)


            
