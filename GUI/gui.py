import sys
from typing import Optional
from PySide6 import QtCore as qtc
from PySide6 import QtWidgets as qtw
from PySide6 import QtGui as qtg
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QWidget

from missions import Ui_MainWindow
import yaml

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
    def __init__(self, parent=None):
        super(Missions, self).__init__(parent=parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.mise_uav_label_updater = PositionUpdater(self.ui.label_uav, self.ui.uav_x, self.ui.uav_y, self.ui.uav_phi)
        self.mise_target_label_updater = PositionUpdater(self.ui.label_tag, self.ui.pl_x, self.ui.pl_y, self.ui.pl_phi)
        self.setup_signals_slots()
        self.setup_states()



    def setup_signals_slots(self):
        def sss_mise():
            self.ui.but_save.clicked.connect(self.save_mission)
            self.ui.but_delete.clicked.connect(self.delete_mission)
            self.ui.but_load.clicked.connect(self.load_mission)
            self.ui.list_mise.clicked.connect(self.mission_selected)
            self.ui.list_mise.activated.connect(self.mission_selected)
            self.ui.list_mise.doubleClicked.connect(self.load_mission)

        sss_mise()
        pass

    def setup_states(self):
        self.ui.tabWidget.setTabEnabled(1, False)   # tab not ready -> disable
        self.ui.tabWidget.setTabEnabled(2, False)   # tab not ready -> disable
        self.mise = MissionsModel()                            # dict of missions, mission name as key
        self.ui.list_mise.setModel(self.mise)

    def save_mission(self):
        nazev = self.ui.textBox_nazev_mise.toPlainText()
        if not nazev: return
        self.mise.save(self.tabMise_mission())

    def delete_mission(self):
        nazev = self.ui.textBox_nazev_mise.toPlainText()
        self.mise.remove(nazev)

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

            alg = self.ui.comboBox_alg.currentText()
        except ValueError:
            pass

    def tabMise_mission(self):
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
        with open("mise.yaml", "w") as file:
            yaml.dump(self.list, file)

    def load_from_file(self):
        try:
            with open("mise.yaml", "r") as file:
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
        

if __name__ == "__main__":
    app = qtw.QApplication(sys.argv)

    window = Missions()
    window.show()

    sys.exit(app.exec())