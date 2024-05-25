from copy import copy
import os
import signal
import subprocess
import sys
import time
from types import NoneType
from typing import Optional
from PySide6 import QtCore as qtc
from PySide6 import QtWidgets as qtw
from PySide6 import QtGui as qtg
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QWidget
import numpy as np
import pyqtgraph as pg
import psutil

from qasync import QEventLoop, QApplication, asyncClose, asyncSlot
import asyncio

from landingAlgs import AlgsModel
from missions import Ui_MainWindow
from mission_result import MissionResult
from experiment_runner import ExperimentRunner

import yaml

from gzCamera import gzCamera

import xml.etree.ElementTree as ET
import math

# class RotatableContainer(qtw.QGraphicsView):
#     def __init__(self, widget: QWidget, rotation: float):
#         super(qtw.QGraphicsView, self).__init__()

#         scene = qtw.QGraphicsScene(self)
#         self.setScene(scene)

#         self.proxy = qtw.QGraphicsProxyWidget()
#         self.proxy.setWidget(widget)
#         self.proxy.setTransformOriginPoint(self.proxy.boundingRect().center())
#         self.proxy.setRotation(rotation)
#         scene.addItem(self.proxy)

#     def rotate(self, rotation: float):
#         self.proxy.setRotation(rotation)
class PositionUpdater():
    def __init__(self, label:qtw.QLabel, xDSB:qtw.QDoubleSpinBox, yDSB:qtw.QDoubleSpinBox, phiDSB:qtw.QDoubleSpinBox) -> None:
        super().__init__()
        xDSB.valueChanged.connect(self.update_pose)
        yDSB.valueChanged.connect(self.update_pose)
        # phiDSB.valueChanged.connect(self.update_pose)
        self.xDSB = xDSB
        self.yDSB = yDSB
        # self.phiDSB = phiDSB
        self.label = label
        self.update_pose()

    def update_pose(self):
        spanX = self.xDSB.maximum() - self.xDSB.minimum()
        spanY = self.yDSB.maximum() - self.yDSB.minimum()
        self.label.move(int(round(self.label.parent().geometry().width()*(1/2 + self.xDSB.value()/spanX))), int(round(self.label.parent().geometry().height()*(1/2 - self.yDSB.value()/spanY))))


class Missions(qtw.QMainWindow):
    run_alg_relay_signal = qtc.Signal(dict, float, float, gzCamera)
    def __init__(self, parent=None):
        super(Missions, self).__init__(parent=parent)
        self.load_config()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.mise_uav_label_updater = PositionUpdater(self.ui.label_uav, self.ui.uav_x, self.ui.uav_y, self.ui.uav_phi)
        self.mise_target_label_updater = PositionUpdater(self.ui.label_tag, self.ui.pl_x, self.ui.pl_y, self.ui.pl_phi)
        self.setup_states()
        self.setup_signals_slots()
        self.resume()

    def resume(self):
        try:
            with open("GUI/pokracovani.yaml", "r") as file:
                resume = yaml.load(file, Loader=yaml.SafeLoader)
                self.ui.textBox_nazev_experimentu.setPlainText(resume["nazev"])
                self.load_experiment()
                self.run_stop_experiment(resume["index"], resume["N"])
        except FileNotFoundError:
            print("Resume file not found.")    

    def load_config(self):
        try:
            with open("GUI/config.yaml", "r") as file:
                self.cfg = yaml.load(file, Loader=yaml.SafeLoader)
        except FileNotFoundError:
            print("Config not found.")      

    def setup_signals_slots(self):
        def sss_mise():
            self.ui.but_save.clicked.connect(self.save_mission)
            self.ui.but_delete.clicked.connect(self.delete_mission)
            self.ui.but_load.clicked.connect(self.load_mission)
            self.ui.list_mise.clicked.connect(self.mission_selected)
            self.ui.list_mise.activated.connect(self.mission_selected)
            self.ui.list_mise.doubleClicked.connect(self.load_mission)
            self.ui.but_start.clicked.connect(self.run_stop_mission)
            self.ui.comboBox_alg.currentIndexChanged.connect(self.on_algChange)

        def sss_experimenty():
            self.ui.but_add_mise.clicked.connect(self.add_mission_to_experiment)
            self.ui.but_remove_mise.clicked.connect(self.remove_mission_from_experiment)

            self.ui.but_save_experiment.clicked.connect(self.save_experiment)
            self.ui.but_delete_experiment.clicked.connect(self.delete_experiment)
            self.ui.but_load_experiment.clicked.connect(self.load_experiment)
            self.ui.but_start_experiment.clicked.connect(self.run_stop_experiment)
            self.ui.list_experimenty.clicked.connect(self.experiment_selected)
            self.ui.list_experimenty.activated.connect(self.experiment_selected)
            self.ui.list_experimenty.doubleClicked.connect(self.load_experiment)

            self.experiment_runner.run_mission.connect(self.run_mission_from_experiment)
            self.experiment_runner.stop_mission.connect(self.stop_mission)
            self.experiment_runner.stop_experiment.connect(self.experiment_stopped)
            for alg in self.algs.list:
                self.experiment_runner.register_alg(alg)

        sss_mise()
        sss_experimenty()

        self.run_alg_relay_signal.connect(self.algs.list[self.ui.comboBox_alg.currentIndex()].run)

    def setup_states(self):
        self.ui.tabWidget.setTabEnabled(1, True)
        self.ui.tabWidget.setTabEnabled(2, True)
        self.mise = MissionsModel()  
        self.experimenty = ExperimentsModel(self.cfg, self.mise)
        self.current_experiment = ExperimentModel(self.cfg, ["pokus1", "pokus3"], [1, 2], all_missions=self.mise)
        self.experiment_runner = ExperimentRunner(self.cfg, self)
        self.ui.list_mise.setModel(self.mise)
        self.ui.list_mise_2.setModel(self.mise)
        self.ui.list_experimenty.setModel(self.experimenty)
        self.px4 = None
        self.algs = AlgsModel(self.cfg)
        self.ui.comboBox_alg.setModel(self.algs)
        self.camera = None
        self.ui.label_cam.setScaledContents(True)
        # self.alg = self.algs.list[self.ui.comboBox_alg.currentIndex()]

        self.setup_plots()

        self.ui.table_experiment.setModel(self.current_experiment)
        for i, c in enumerate(self.current_experiment.columns):
            if c in self.cfg["exclude_mission_columns_in_experiment_table"]:
                self.ui.table_experiment.setColumnHidden(i, True)

    def setup_plots(self):
        self.lines = []
        labelY=["Chyba x [m]", "Chyba y [m]", "Chyba yaw [°]"]
        nazvy=["Chyba x", "Chyba y", "Chyba yaw"]
        plots = []
        for i in range(3):
            pl = self.ui.grafChyby.addPlot(row=0, col=i)
            plots.append(pl)
            self.ui.grafChyby.setBackground((45,45,45))
            pen = pg.mkPen(color=(255, 110, 0), width=2)
            styles = {"color": "white", "font-size": "18px"}
            pl.setLabel("left", labelY[i], **styles)
            pl.setLabel("bottom", "Čas [s]", **styles)
            # pl.addLegend()
            pl.showGrid(x=True, y=True)
            # Get a line reference
            self.lines.append(pl.plot(
                [0,1],[0,1],
                name=nazvy[i],
                pen=pen
            ))
        plots[1].setXLink(plots[0])
        plots[2].setXLink(plots[0])
        self.plotData=[[],[],[]]
        self.plotTimes=[]

    def add_mission_to_experiment(self):
        for index in self.ui.list_mise_2.selectedIndexes():
            self.current_experiment.add(self.mise.names[index.row()])

    def remove_mission_from_experiment(self):
        rows_removed = []
        for index in self.ui.table_experiment.selectedIndexes():
            row = index.row()
            if row in rows_removed:
                continue
            rows_removed.append(row)
            self.current_experiment.remove(self.current_experiment.names[row])

    @qtc.Slot(int)
    def on_algChange(self, index):
        self.run_alg_relay_signal.disconnect()
        self.run_alg_relay_signal.connect(self.algs.list[index].run)

    def on_algStep(self, message):
        self.ui.plainTextEdit.setPlainText(message)

    def on_trajectoryError(self, err, time):
        self.plotTimes.append(time)
        for i in range(3):
            self.plotData[i].append(err[i])

    def updatePlots(self):
        for i in range(3):
            self.lines[i].setData(self.plotTimes, self.plotData[i])

    def save_mission(self):
        nazev = self.ui.textBox_nazev_mise.toPlainText()
        if not nazev: return
        self.mise.save(self.tabMise_mission_dict())
    
    def save_experiment(self):
        nazev = self.ui.textBox_nazev_experimentu.toPlainText()
        if not nazev: return
        self.current_experiment = self.experimenty.save(self.current_experiment, nazev)
        self.ui.table_experiment.setModel(self.current_experiment)

    def delete_mission(self):
        nazev = self.ui.textBox_nazev_mise.toPlainText()
        self.mise.remove(nazev)

    def delete_experiment(self):
        nazev = self.ui.textBox_nazev_experimentu.toPlainText()
        self.experimenty.remove(nazev)

    def load_mission(self, index = None):
        try:
            if index:
                nazev = self.mise.names[index.row()]
                self.ui.textBox_nazev_mise.setPlainText(nazev)
                mise = self.mise.list[index.row()]
            else:
                nazev = self.ui.textBox_nazev_mise.toPlainText()
                mise = self.mise.get(nazev)

            pl_x = mise["plosina"]["x"]
            self.ui.pl_x.setValue(pl_x)
            pl_y = mise["plosina"]["y"]
            self.ui.pl_y.setValue(pl_y)
            pl_phi = mise["plosina"]["phi"]
            self.ui.pl_phi.setValue(pl_phi)

            uav_x = mise["uav"]["x"]
            self.ui.uav_x.setValue(uav_x)
            uav_y = mise["uav"]["y"]
            self.ui.uav_y.setValue(uav_y)
            uav_phi = mise["uav"]["phi"]
            self.ui.uav_phi.setValue(uav_phi)

            vitr_v = mise["vitr"]["v"]
            self.ui.vitr_v.setValue(vitr_v)
            vitr_s_v = mise["vitr"]["s_v"]
            self.ui.vitr_s_v.setValue(vitr_s_v)
            vitr_phi = mise["vitr"]["phi"]
            self.ui.vitr_phi.setValue(vitr_phi)
            vitr_s_phi = mise["vitr"]["s_phi"]
            self.ui.vitr_s_phi.setValue(vitr_s_phi)

            pl_a = mise["plosina"]["a"]
            self.ui.pl_a.setValue(pl_a)
            pl_stin = mise["plosina"]["stin"]
            self.ui.pl_stin.setValue(pl_stin)
            pl_stin_sklon = mise["plosina"]["stin_sklon"]
            self.ui.pl_stin_sklon.setValue(pl_stin_sklon)

            alg = mise["algoritmus"]
            try:
                self.ui.comboBox_alg.setCurrentIndex([alg.name for alg in self.algs.list].index(alg))
            except ValueError:
                self.ui.comboBox_alg.setCurrentIndex(0)
        except ValueError:
            pass

    def load_experiment(self, index:qtc.QModelIndex|NoneType=None):
        try:
            if index:
                nazev = self.experimenty.names[index.row()]
                self.ui.textBox_nazev_experimentu.setPlainText(nazev)
                experiment = self.experimenty.list[index.row()]
            else:
                nazev = self.ui.textBox_nazev_experimentu.toPlainText()
                experiment = self.experimenty.get(nazev)
            self.current_experiment = experiment.copy()

        except ValueError:
            pass
        
        self.ui.table_experiment.setModel(self.current_experiment)

    def tabMise_mission_dict(self):
        pl_x = self.ui.pl_x.value()
        pl_y = self.ui.pl_y.value()
        pl_phi = self.ui.pl_phi.value()

        uav_x = self.ui.uav_x.value()
        uav_y = self.ui.uav_y.value()
        uav_phi = self.ui.uav_phi.value()

        vitr_v = self.ui.vitr_v.value()
        vitr_s_v = self.ui.vitr_s_v.value()
        vitr_phi = self.ui.vitr_phi.value()
        vitr_s_phi = self.ui.vitr_s_phi.value()

        pl_a = self.ui.pl_a.value()
        pl_stin = self.ui.pl_stin.value()
        pl_stin_sklon = self.ui.pl_stin_sklon.value()

        alg = self.ui.comboBox_alg.currentText()

        nazev = self.ui.textBox_nazev_mise.toPlainText()

        mise = {"nazev": nazev,
                "plosina": {"x": pl_x, "y": pl_y, "phi": pl_phi, "a": pl_a, "stin": pl_stin, "stin_sklon": pl_stin_sklon},
                "uav": {"x": uav_x, "y": uav_y, "phi": uav_phi},
                "vitr": {"v": vitr_v, "s_v": vitr_s_v, "phi": vitr_phi, "s_phi": vitr_s_phi},
                "algoritmus": alg}
        
        return mise
    
    def mission_selected(self, index):
        self.ui.textBox_nazev_mise.setPlainText(self.mise.names[index.row()])

    def experiment_selected(self, index:qtc.QModelIndex):
        self.ui.textBox_nazev_experimentu.setPlainText(self.experimenty.names[index.row()])

    def stop_mission(self):
        for proc in psutil.process_iter():
            if "ruby" in proc.name() or "px4" in proc.name():
                proc.send_signal(signal.SIGKILL)
        self.px4 = None
        try:
            try:
                self.camera.new_frame.disconnect()
            except RuntimeError: # Already disconnected
                pass
            self.camera.stop()
            self.camera = None
        except AttributeError:
            pass                # Already None
        self.ui.comboBox_alg.setEnabled(True)
        self.ui.tabWidget.setCurrentIndex(0)
        self.ui.tabWidget.setTabEnabled(1, False)
        self.ui.plainTextEdit.setPlainText("-")

    def experiment_stopped(self):
        self.ui.label_exp_i.setText("-")
        self.ui.label_exp_N.setText("-")

    def run_mission(self):
        if not self.ui.textBox_nazev_mise.toPlainText():
            self.ui.textBox_nazev_mise.setPlainText("BezNazvu")

        command = [self.cfg["px4_bin_path"]]
        env_vars = {
            'PX4_SYS_AUTOSTART': self.cfg["px4_sys_autostart"],
            'PX4_GZ_MODEL': self.cfg["gz_model"],
            'PX4_GZ_WORLD': self.ui.textBox_nazev_mise.toPlainText(),
            'PX4_GZ_MODEL_POSE': " ".join((str(self.ui.uav_x.value()), str(self.ui.uav_y.value()), "0", "0", "0", str(self.ui.uav_phi.value()))),
            'PX4_HOME_LAT': str(self.cfg["origin"]["lat_deg"]),
            'PX4_HOME_LON': str(self.cfg["origin"]["lon_deg"]),
            'PX4_HOME_ALT': str(self.cfg["origin"]["elevation"])
        }

        if self.cfg["headless"]:
            env_vars["HEADLESS"] = str(1)

        self.update_sdf_with_mission(self.ui.textBox_nazev_mise.toPlainText()+".sdf")

        original_env = os.environ.copy()

        # Update environment with the new variables
        env = original_env.copy()
        env.update(env_vars)

        self.px4 = subprocess.Popen(command, env=env, stdin=subprocess.PIPE, shell=True)

        os.environ.clear()
        os.environ.update(original_env)

        self.ui.comboBox_alg.setEnabled(False)

        self.camera = gzCamera(self, self.cfg)
        self.camera.new_frame.connect(self.updateCameraFrame)
        self.camera.new_frame.connect(self.run_alg_relay_slot)

        self.ui.tabWidget.setTabEnabled(1, True)
        self.ui.tabWidget.setCurrentIndex(1)

        try:
            self.algs.list[self.ui.comboBox_alg.currentIndex()].step_signal.connect(self.on_algStep)
        except AttributeError: # doesnt have step signal
            pass

        try:
            self.algs.list[self.ui.comboBox_alg.currentIndex()].ae_signal.connect(self.on_trajectoryError)
        except AttributeError: # doesnt have trajectory error signal
            pass
        
        self.plotTimes = []
        self.plotData = [[],[],[]]
        self.updatePlots()
        self.stepTimer = qtc.QTimer()
        self.stepTimer.timeout.connect(self.updatePlots)
        self.stepTimer.start(5000)

    def run_alg_relay_slot(self, x, y, z, yaw, t, frame, step_start):
        self.camera.new_frame.disconnect(self.run_alg_relay_slot)
        time.sleep(2)
        uav_pl_rel_x = self.ui.pl_x.value() - self.ui.uav_x.value()
        uav_pl_rel_y = self.ui.pl_y.value() - self.ui.uav_y.value()
        self.run_alg_relay_signal.emit(self.tabMise_mission_dict(), uav_pl_rel_x, uav_pl_rel_y, self.camera)

    def run_stop_mission(self):
        if self.px4 is None:
            self.ui.but_start.setText("■ Stop")
            self.run_mission()
            return
        self.ui.but_start.setText("▶ Start")
        self.algs.list[self.ui.comboBox_alg.currentIndex()].stop()
        self.stop_mission()

    def run_mission_from_experiment(self, name):
        self.ui.textBox_nazev_mise.setPlainText(name)
        self.ui.label_exp_i.setText(str(self.experiment_runner.i))
        self.ui.label_exp_N.setText(str(self.experiment_runner.N))
        self.load_mission()
        self.run_stop_mission()

    def current_experiment_name(self):
        return self.ui.textBox_nazev_experimentu.toPlainText()

    def run_stop_experiment(self, i=0, N=0):
        if not self.experiment_runner.running:
            if self.px4 is not None:
                return
            self.ui.but_start_experiment.setText("■ Stop")
            self.experiment_runner.run(self.current_experiment, self.current_experiment_name(), i, N)
            return
        if self.px4 is not None:
            self.run_stop_mission()
        self.ui.but_start_experiment.setText("▶ Start")
        self.experiment_runner.stop()

    def update_sdf_with_mission(self, output_file):
        tree = ET.parse(self.cfg["base_world_path"])
        root = tree.getroot()

        mission = self.tabMise_mission_dict()

        include = root.find(".//include")           # umisteni tagu
        pose_element = include.find("pose")
        pose_yaw = -math.radians(mission['plosina']['phi'])
        pose_x = mission['plosina']['x'] - math.cos(pose_yaw) * mission["plosina"]["a"]/200.0
        pose_y = mission['plosina']['y'] - math.sin(pose_yaw) * mission["plosina"]["a"]/200.0
        pose_values = f"{pose_x} {pose_y} 0 0 1.57 {pose_yaw}"
        pose_element.text = pose_values

        wind = root.find(".//wind")                 # vitr
        linear_velocity_element = wind.find("linear_velocity")
        wind_phi = mission['vitr']['phi']
        wind_v = mission['vitr']['v']
        wind_speed_x = wind_v * math.cos(wind_phi)
        wind_speed_y = wind_v * math.sin(wind_phi)
        linear_velocity_values = f"{wind_speed_x} {wind_speed_y} 0"
        linear_velocity_element.text = linear_velocity_values

        plugin = root.find(".//plugin[@name='gz::sim::systems::WindEffects']")              # sum vetru
        horizontal = plugin.find("./horizontal")
        direction = horizontal.find("./direction")
        noise_s_phi = mission['vitr']['s_phi']
        noise_s_v = mission['vitr']['s_v']
        direction_noise = direction.find("./noise")
        direction_noise_stddev = direction_noise.find("./stddev")
        direction_noise_stddev.text = str(noise_s_phi)

        magnitude = horizontal.find("./magnitude")
        magnitude_noise = magnitude.find("./noise")
        magnitude_noise_stddev = magnitude_noise.find("./stddev")
        magnitude_noise_stddev.text = str(noise_s_v)

        world = root.find(".//world")
        world.set("name", mission["nazev"])

        coordinates = world.find("./spherical_coordinates")
        lat = coordinates.find("./latitude_deg")
        lon = coordinates.find("./longitude_deg")
        elev = coordinates.find("./elevation")
        lat.text = str(self.cfg["origin"]["lat_deg"])
        lon.text = str(self.cfg["origin"]["lon_deg"])
        elev.text = str(self.cfg["origin"]["elevation"])

        sun = root.find(".//light[@name='sun']")
        sun.find("./direction").text = " ".join([str(d) for d in self.cfg["sun_direction"]])

        shade = root.find(".//model[@name='shade']")
        if mission["plosina"]["stin"]>0:
            shade_x = pose_x + ((mission["plosina"]["stin"]/100 - 1) * (mission["plosina"]["a"]/100) * math.copysign(1,float(self.cfg["sun_direction"][0])))
            shade_y = pose_y
            shade_xyz = np.array([shade_x, shade_y, -2.5]) - 400*np.array(self.cfg["sun_direction"])
            shade.find("pose").text = f"{shade_xyz[0]} {shade_xyz[1]} {shade_xyz[2]} 0 0 {math.radians(mission['plosina']['stin_sklon'])}"
        else:
            world.remove(shade)

        tree.write(os.path.join(self.cfg["gz_worlds_dir"], output_file))

    @asyncClose
    async def closeEvent(self, event):
        pass

    @asyncSlot()
    async def onMyEvent(self):
        pass

    @qtc.Slot(object, object, object, object, float, qtg.QImage, float)
    def updateCameraFrame(self, x, y, z, yaw, t, frame, step_start):
        self.ui.label_cam.setPixmap(qtg.QPixmap(frame))
    
class MissionsModel(qtc.QAbstractListModel):
    def __init__(self, missionList=None):
        super().__init__()
        self.list = missionList or []
        self.names = [m["nazev"] for m in self.list]
        self.load_from_file()

    def data(self, index, role):
        if role == Qt.DisplayRole:
            mission = self.list[index.row()]
            return mission["nazev"]

    def rowCount(self, index):
        return len(self.list)
    
    def save_to_file(self):
        with open("GUI/mise.yaml", "w") as file:
            yaml.dump(self.list, file)

    def load_from_file(self):
        try:
            with open("GUI/mise.yaml", "r") as file:
                self.list = yaml.load(file, Loader=yaml.SafeLoader)
                self.names = [m["nazev"] for m in self.list]
                self.layoutChanged.emit()
        except FileNotFoundError:
            pass
    
    def save(self, mission):
        try:
            index = self.names.index(mission["nazev"])
            self.list[index] = mission
        except ValueError:
            self.list.append(mission)
            self.names.append(mission["nazev"])
            self.layoutChanged.emit()
        self.save_to_file()

    def remove(self, missionName):
        try:
            index = self.names.index(missionName)
            self.list.pop(index)
            self.names.pop(index)
            self.layoutChanged.emit()
            self.save_to_file()
        except ValueError:
            pass

    def get(self, missionName):
        index = self.names.index(missionName)
        return self.list[index]
    
class ExperimentModel(qtc.QAbstractTableModel):
    def __init__(self, cfg, names, Ns, all_missions: MissionsModel):
        super().__init__()
        self.cfg=cfg
        self.names = names or []
        self.Ns = Ns or [0 for _ in self.names]
        self.missions = all_missions
        self.remove_nonexistent_missions()
        self.columns = self.extract_column_names(self.cfg["sample_mission"])

    def remove_nonexistent_missions(self):
        i = 0
        while i<len(self.names):
            name = self.names[i]
            if name not in self.missions.names:
                self.names.pop(i)
                self.Ns.pop(i)
                continue
            i+=1

    def extract_column_names(self, data, parent_key='', sep='.'):
        column_names = []
        for k, v in data.items():
            new_key = f"{parent_key}{sep}{k}" if parent_key else k
            if isinstance(v, dict):
                column_names.extend(self.extract_column_names(v, new_key, sep))
            else:
                column_names.append(new_key)
        return column_names
    
    def get_value_for_column(self, mission_name, column_name):
        keys = column_name.split('.')
        value = self.missions.get(mission_name)
        try:
            for key in keys:
                value = value[key]
            return value
        except KeyError:
            return None

    def data(self, index, role):
        if role == Qt.DisplayRole or role == Qt.ItemDataRole.EditRole:
            if index.column() == len(self.columns):
                return self.Ns[index.row()]
            mission_name = self.names[index.row()]
            return self.get_value_for_column(mission_name, self.columns[index.column()])
        
    def setData(self, index: qtc.QModelIndex | qtc.QPersistentModelIndex, value, role: int = ...) -> bool:
        if role == Qt.EditRole:
            if index.column() == len(self.columns):
                try:
                    value = int(value)
                    if value<0: return False
                    self.Ns[index.row()] = value
                    return True
                except ValueError:
                    pass
            return False
        
    def flags(self, index):
        default_flags = super().flags(index)
        if index.column() == len(self.columns):
            return Qt.ItemFlag.ItemIsEnabled|Qt.ItemFlag.ItemIsSelectable|Qt.ItemFlag.ItemIsEditable
        return default_flags
        
    def headerData(self, section: int, orientation: Qt.Orientation, role: int = ...):
        if role == Qt.DisplayRole and orientation == Qt.Orientation.Horizontal:
            if section == len(self.columns):
                return "N" # last column is the number of times the mission should be run
            current = self.columns[section]
            # try:
            #     previous = self.columns[section-1]
            #     current_split = current.split('.')
            #     previous_split = previous.split('.')
            #     if previous not in self.cfg["exclude_mission_columns_in_experiment_table"] and current_split[0] == previous_split[0]:
            #         return "."+current_split[-1]
            # except IndexError:
            #     pass
            return current
        return super().headerData(section, orientation, role)

    def rowCount(self, index):
        return len(self.names)
    
    def columnCount(self, index):
        return len(self.columns) + 1

    def remove(self, missionName):
        index = self.names.index(missionName)
        self.Ns.pop(index)
        self.names.pop(index)
        self.layoutChanged.emit()

    def add(self, missionName):
        try:
            i = self.names.index(missionName)
            self.Ns[i] += self.cfg["default_mission_repeats"]
        except ValueError:
            self.names.append(missionName)
            self.Ns.append(self.cfg["default_mission_repeats"])
        self.layoutChanged.emit()

    def get(self, missionName):
        index = self.names.index(missionName)
        return self.list[index]
    
    def dict(self):
        return {"mise": {mission_name: reps for mission_name, reps in zip(self.names, self.Ns)}}
    
    def copy(self):
        return ExperimentModel(self.cfg, copy(self.names), copy(self.Ns), self.missions)
    
class ExperimentsModel(qtc.QAbstractListModel):
    def __init__(self, cfg, missions: MissionsModel):
        super().__init__()
        self.cfg=cfg
        self.list = []
        self.names = []
        self.missions = missions
        self.load_from_file()

    def data(self, index, role):
        if role == Qt.DisplayRole:
            return self.names[index.row()]

    def rowCount(self, index):
        return len(self.list)
    
    def save_to_file(self):
        dict_list = []
        for name, e in zip(self.names, self.list):
            dict_list.append(e.dict())
            dict_list[-1]["nazev"] = name

        with open(self.cfg["experiments_file"], "w") as file:
            yaml.dump(dict_list, file)

    def load_from_file(self):
        self.names = []
        self.list = []
        try:
            with open(self.cfg["experiments_file"], "r") as file:
                dict_list = yaml.load(file, Loader=yaml.SafeLoader)
                for e in dict_list:
                    self.names.append(e["nazev"])
                    self.list.append(ExperimentModel(self.cfg, list(e["mise"].keys()), list(e["mise"].values()), self.missions))
                self.layoutChanged.emit()
        except FileNotFoundError:
            pass
    
    def save(self, experiment: ExperimentModel, name):
        try:
            index = self.names.index(name)
            self.list[index] = experiment
        except ValueError:
            self.list.append(experiment)
            self.names.append(name)
            self.layoutChanged.emit()
        self.save_to_file()
        return experiment.copy()

    def remove(self, experimentName):
        try:
            index = self.names.index(experimentName)
            self.list.pop(index)
            self.names.pop(index)
            self.layoutChanged.emit()
            self.save_to_file()
        except ValueError:
            pass

    def get(self, experimentName) -> ExperimentModel:
        index = self.names.index(experimentName)
        return self.list[index]
        

if __name__ == "__main__":
    for proc in psutil.process_iter():
        if "ruby" in proc.name() or "px4" in proc.name():
            proc.send_signal(signal.SIGKILL)
    app = QApplication(sys.argv)

    event_loop = QEventLoop(app)
    asyncio.set_event_loop(event_loop)

    app_close_event = asyncio.Event()
    app.aboutToQuit.connect(app_close_event.set)

    window = Missions()
    window.show()

    with event_loop:
        event_loop.run_until_complete(app_close_event.wait())