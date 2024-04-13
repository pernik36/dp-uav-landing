import asyncio
from datetime import datetime
from PySide6 import QtCore as qtc
import yaml
from mission_result import MissionResult
from qasync import asyncSlot

class ExperimentRunner(qtc.QObject):
    run_mission = qtc.Signal(str)
    stop_mission = qtc.Signal(None)
    def __init__(self, cfg, app) -> None:
        super().__init__()
        self.running = False
        self.cfg = cfg
        self.app = app

    def run(self, experiment, name, start=0):
        self.results = []
        self.running = True
        self.experiment = experiment
        self.experiment_name = name
        self.i = start
        self.N = 0
        self.step()

    def step(self):
        try:
            if self.experiment.Ns[self.i] == 0:
                self.i += 1
                self.step()
                return
            self.run_mission.emit(self.experiment.names[self.i])
            print("Launched a mission")
            self.N += 1
            if self.N >= self.experiment.Ns[self.i]:
                self.i += 1
                self.N = 0

        except IndexError:
            print("Reached the end of an experiment")
            self.stop()

    def register_alg(self, alg):
        alg.ended.connect(self.mission_eval)

    @asyncSlot(MissionResult)
    async def mission_eval(self, result):
        if not self.running:
            return
        print("Ended signal received")
        self.stop_mission.emit()
        while self.app.px4 is not None:
            print("Waiting for PX4 to be stoppped.", end="\r")
            await asyncio.sleep(0.5)
        print("Saving results")
        self.results.append(result)
        self.step()

    def stop(self):
        self.running = False
        current_datetime = datetime.now().isoformat(timespec='seconds')
        filename = f"experiment_results/{current_datetime}_{self.experiment_name}.yaml"
        with open(filename, 'w') as file:
            yaml.dump([result.to_dict() for result in self.results], file)
        for result in self.results:
            print(result.to_dict())
        # vymyslet něco s vyhodnocením

    