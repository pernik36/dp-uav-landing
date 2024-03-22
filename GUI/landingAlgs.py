from mavsdk import System

from PySide6 import QtCore as qtc
from PySide6.QtCore import Qt

from abc import ABC, abstractmethod
from qasync import asyncSlot
import asyncio

class LAMeta(type(qtc.QObject), type(ABC)):
    pass

class landingAlg(ABC, qtc.QObject, metaclass=LAMeta):
    @abstractmethod
    def __init__(self) -> None:
        pass

    @abstractmethod
    def run():
        pass

    @abstractmethod
    def stop():
        pass

class AlgsModel(qtc.QAbstractListModel):
    def __init__(self, algsList: list[landingAlg] | None = None):
        super().__init__()
        self.list = algsList or [mavlink_test()]

    def data(self, index, role):
        if role == Qt.DisplayRole:
            alg = self.list[index.row()]
            return alg.name

    def rowCount(self, index):
        return len(self.list)
    
class mavlink_test(landingAlg, qtc.QObject):
    def __init__(self) -> None:
        super().__init__()
        self.drone = System()
        self.name = "MAVlink test"
        self.tasks = []

    @asyncSlot(None)
    async def run(self):
        await self.drone.connect(system_address="udp://:14540")

        self.tasks.append(asyncio.ensure_future(self.subscribe_attitude()))
        # altitude_task = asyncio.ensure_future(self.print_altitude())

    @asyncSlot(None)
    async def stop(self):
        for task in self.tasks:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        self.drone = System() # reset the connection

    async def print_altitude(self):
        """ Prints the altitude when it changes """
        previous_altitude = None

        async for position in self.drone.telemetry.position():
            altitude = position.relative_altitude_m
            altitude_rounded = round(altitude)
            if altitude_rounded != previous_altitude:
                previous_altitude = altitude_rounded
                print(f"Altitude: {altitude} m")


    async def subscribe_attitude(self):
        global pitch_deg, roll_deg
        async for attitude in self.drone.telemetry.attitude_euler():
            pitch_deg = attitude.pitch_deg
            roll_deg = attitude.roll_deg
            # print(f"Pitch: {pitch_deg:6.2f}, Roll: {roll_deg:6.2f}", end="\r")