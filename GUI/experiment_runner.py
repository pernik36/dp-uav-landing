import asyncio
from datetime import datetime
import os
import sys
from PySide6 import QtCore as qtc
import yaml
from mission_result import MissionResult
from qasync import asyncSlot

class ExperimentRunner(qtc.QObject):
    run_mission = qtc.Signal(str)
    stop_mission = qtc.Signal(None)
    stop_experiment = qtc.Signal(None)
    restart = qtc.Signal(None)
    def __init__(self, cfg, app) -> None:
        super().__init__()
        self.running = False
        self.cfg = cfg
        self.app = app
        self.timer = qtc.QTimer()
        self.timer.timeout.connect(self.reset_program)

    def run(self, experiment, name, start=0, N=0):
        self.experiments_ran = 0
        self.results = []
        self.running = True
        self.experiment = experiment
        self.experiment_name = name
        self.i = start
        self.N = N
        current_datetime = datetime.now().isoformat(timespec='seconds')
        self.filename = f"experiment_results/{current_datetime}_{self.experiment_name}.yaml"
        self.step()

    def step(self):
        if self.experiments_ran >= self.cfg["restart_after_experiments"]:
            self.reset_program()
        try:
            if self.experiment.Ns[self.i] == 0:
                self.i += 1
                self.step()
                return
            if "stin" in self.experiment.names[self.i]:
                pass
            else:
                self.timer.start(180000)
            self.run_mission.emit(self.experiment.names[self.i])
            print("Launched a mission")
            self.N += 1
            self.experiments_ran += 1
            if self.N >= self.experiment.Ns[self.i]:
                self.i += 1
                self.N = 0

        except IndexError:
            print("Reached the end of an experiment")
            self.stop()

    def register_alg(self, alg):
        alg.ended.connect(self.mission_eval)

    def reset_program(self):
        os.execl(sys.executable, 'python3', __file__, *sys.argv[1:])

    @asyncSlot(MissionResult)
    async def mission_eval(self, result):
        if not self.running:
            return
        self.timer.stop()
        print("Ended signal received")
        print("Saving results of mission ", result.mission_def["nazev"], "repetition: ", self.N)
        self.results.append(result)
        with open(self.filename, 'w') as file:
            yaml.dump([result.to_dict() for result in self.results], file)
        self.save_resume_file()
        self.stop_mission.emit()
        await asyncio.sleep(3)
        while self.app.px4 is not None:
            print("Waiting for PX4 to be stoppped.", end="\r")
            await asyncio.sleep(0.5)
        self.step()

    def save_resume_file(self):
        with open("GUI/pokracovani.yaml", "w") as file:
            yaml.dump({"nazev": self.experiment_name, "index": self.i, "N": self.N}, file)

    def stop(self):
        self.running = False
        try:
            os.remove("GUI/pokracovani.yaml")
        except FileNotFoundError:
            pass
        self.stop_experiment.emit()
        for result in self.results:
            print(result.to_dict())

    