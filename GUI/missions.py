# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'missions.ui'
##
## Created by: Qt User Interface Compiler version 6.6.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QAbstractItemView, QAbstractSpinBox, QApplication, QColumnView,
    QComboBox, QDoubleSpinBox, QFrame, QGroupBox,
    QHeaderView, QLabel, QListView, QMainWindow,
    QMenuBar, QPlainTextEdit, QPushButton, QSizePolicy,
    QStatusBar, QTabWidget, QTableView, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1166, 603)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.tabWidget = QTabWidget(self.centralwidget)
        self.tabWidget.setObjectName(u"tabWidget")
        self.tabWidget.setEnabled(True)
        self.tabWidget.setGeometry(QRect(0, 0, 1161, 561))
        self.tabWidget.setDocumentMode(False)
        self.tabWidget.setTabsClosable(False)
        self.tabWidget.setMovable(False)
        self.tabWidget.setTabBarAutoHide(False)
        self.tab_mise = QWidget()
        self.tab_mise.setObjectName(u"tab_mise")
        self.mapa = QWidget(self.tab_mise)
        self.mapa.setObjectName(u"mapa")
        self.mapa.setGeometry(QRect(10, 10, 371, 371))
        self.mapa.setAcceptDrops(True)
        self.mapa.setStyleSheet(u"border-image : url(GUI/assets/img/borcak1.png)")
        self.label_uav = QLabel(self.mapa)
        self.label_uav.setObjectName(u"label_uav")
        self.label_uav.setGeometry(QRect(70, 110, 15, 15))
        font = QFont()
        font.setPointSize(14)
        font.setBold(False)
        font.setItalic(False)
        self.label_uav.setFont(font)
        self.label_uav.setAutoFillBackground(False)
        self.label_uav.setStyleSheet(u"border-image : none; color: white;")
        self.label_uav.setFrameShape(QFrame.NoFrame)
        self.label_tag = QLabel(self.mapa)
        self.label_tag.setObjectName(u"label_tag")
        self.label_tag.setGeometry(QRect(280, 190, 15, 15))
        self.label_tag.setStyleSheet(u"border-image: none; background-image : url(GUI/assets/img/tag.png); color: red")
        self.label_tag.setFrameShape(QFrame.Panel)
        self.label_tag.setLineWidth(1)
        self.groupBox = QGroupBox(self.tab_mise)
        self.groupBox.setObjectName(u"groupBox")
        self.groupBox.setGeometry(QRect(400, 10, 331, 71))
        self.pl_x = QDoubleSpinBox(self.groupBox)
        self.pl_x.setObjectName(u"pl_x")
        self.pl_x.setGeometry(QRect(30, 30, 71, 26))
        self.pl_x.setWrapping(True)
        self.pl_x.setProperty("showGroupSeparator", False)
        self.pl_x.setDecimals(1)
        self.pl_x.setMinimum(-100.000000000000000)
        self.pl_x.setMaximum(100.000000000000000)
        self.pl_x.setSingleStep(0.500000000000000)
        self.pl_x.setStepType(QAbstractSpinBox.AdaptiveDecimalStepType)
        self.pl_x.setValue(23.000000000000000)
        self.label = QLabel(self.groupBox)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(10, 31, 21, 20))
        self.label.setAlignment(Qt.AlignBottom|Qt.AlignLeading|Qt.AlignLeft)
        self.label_2 = QLabel(self.groupBox)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(120, 30, 21, 20))
        self.label_2.setAlignment(Qt.AlignBottom|Qt.AlignLeading|Qt.AlignLeft)
        self.pl_y = QDoubleSpinBox(self.groupBox)
        self.pl_y.setObjectName(u"pl_y")
        self.pl_y.setGeometry(QRect(140, 29, 71, 26))
        self.pl_y.setWrapping(True)
        self.pl_y.setProperty("showGroupSeparator", False)
        self.pl_y.setDecimals(1)
        self.pl_y.setMinimum(-100.000000000000000)
        self.pl_y.setMaximum(100.000000000000000)
        self.pl_y.setSingleStep(0.500000000000000)
        self.pl_y.setStepType(QAbstractSpinBox.AdaptiveDecimalStepType)
        self.pl_y.setValue(-11.199999999999999)
        self.pl_phi = QDoubleSpinBox(self.groupBox)
        self.pl_phi.setObjectName(u"pl_phi")
        self.pl_phi.setEnabled(False)
        self.pl_phi.setGeometry(QRect(250, 29, 71, 26))
        self.pl_phi.setWrapping(True)
        self.pl_phi.setProperty("showGroupSeparator", False)
        self.pl_phi.setDecimals(0)
        self.pl_phi.setMinimum(0.000000000000000)
        self.pl_phi.setMaximum(359.000000000000000)
        self.pl_phi.setSingleStep(1.000000000000000)
        self.pl_phi.setStepType(QAbstractSpinBox.DefaultStepType)
        self.pl_phi.setValue(0.000000000000000)
        self.label_3 = QLabel(self.groupBox)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setEnabled(False)
        self.label_3.setGeometry(QRect(230, 30, 21, 20))
        self.label_3.setAlignment(Qt.AlignBottom|Qt.AlignLeading|Qt.AlignLeft)
        self.groupBox_2 = QGroupBox(self.tab_mise)
        self.groupBox_2.setObjectName(u"groupBox_2")
        self.groupBox_2.setGeometry(QRect(400, 90, 331, 71))
        self.uav_x = QDoubleSpinBox(self.groupBox_2)
        self.uav_x.setObjectName(u"uav_x")
        self.uav_x.setGeometry(QRect(30, 30, 71, 26))
        self.uav_x.setWrapping(True)
        self.uav_x.setProperty("showGroupSeparator", False)
        self.uav_x.setDecimals(1)
        self.uav_x.setMinimum(-100.000000000000000)
        self.uav_x.setMaximum(100.000000000000000)
        self.uav_x.setSingleStep(0.500000000000000)
        self.uav_x.setStepType(QAbstractSpinBox.AdaptiveDecimalStepType)
        self.uav_x.setValue(-5.000000000000000)
        self.label_4 = QLabel(self.groupBox_2)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setGeometry(QRect(10, 31, 21, 20))
        self.label_4.setAlignment(Qt.AlignBottom|Qt.AlignLeading|Qt.AlignLeft)
        self.label_5 = QLabel(self.groupBox_2)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setGeometry(QRect(120, 30, 21, 20))
        self.label_5.setAlignment(Qt.AlignBottom|Qt.AlignLeading|Qt.AlignLeft)
        self.uav_y = QDoubleSpinBox(self.groupBox_2)
        self.uav_y.setObjectName(u"uav_y")
        self.uav_y.setGeometry(QRect(140, 29, 71, 26))
        self.uav_y.setWrapping(True)
        self.uav_y.setProperty("showGroupSeparator", False)
        self.uav_y.setDecimals(1)
        self.uav_y.setMinimum(-100.000000000000000)
        self.uav_y.setMaximum(100.000000000000000)
        self.uav_y.setSingleStep(0.500000000000000)
        self.uav_y.setStepType(QAbstractSpinBox.AdaptiveDecimalStepType)
        self.uav_y.setValue(3.000000000000000)
        self.uav_phi = QDoubleSpinBox(self.groupBox_2)
        self.uav_phi.setObjectName(u"uav_phi")
        self.uav_phi.setEnabled(False)
        self.uav_phi.setGeometry(QRect(250, 29, 71, 26))
        self.uav_phi.setWrapping(True)
        self.uav_phi.setProperty("showGroupSeparator", False)
        self.uav_phi.setDecimals(0)
        self.uav_phi.setMinimum(0.000000000000000)
        self.uav_phi.setMaximum(359.000000000000000)
        self.uav_phi.setSingleStep(1.000000000000000)
        self.uav_phi.setStepType(QAbstractSpinBox.DefaultStepType)
        self.uav_phi.setValue(0.000000000000000)
        self.label_6 = QLabel(self.groupBox_2)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setEnabled(False)
        self.label_6.setGeometry(QRect(230, 30, 21, 20))
        self.label_6.setAlignment(Qt.AlignBottom|Qt.AlignLeading|Qt.AlignLeft)
        self.groupBox_3 = QGroupBox(self.tab_mise)
        self.groupBox_3.setObjectName(u"groupBox_3")
        self.groupBox_3.setGeometry(QRect(400, 170, 241, 101))
        self.vitr_v = QDoubleSpinBox(self.groupBox_3)
        self.vitr_v.setObjectName(u"vitr_v")
        self.vitr_v.setGeometry(QRect(40, 30, 81, 26))
        self.vitr_v.setWrapping(True)
        self.vitr_v.setProperty("showGroupSeparator", False)
        self.vitr_v.setDecimals(1)
        self.vitr_v.setMinimum(0.000000000000000)
        self.vitr_v.setMaximum(30.000000000000000)
        self.vitr_v.setSingleStep(0.100000000000000)
        self.vitr_v.setStepType(QAbstractSpinBox.AdaptiveDecimalStepType)
        self.vitr_v.setValue(0.000000000000000)
        self.label_7 = QLabel(self.groupBox_3)
        self.label_7.setObjectName(u"label_7")
        self.label_7.setGeometry(QRect(20, 31, 21, 20))
        self.label_7.setAlignment(Qt.AlignBottom|Qt.AlignLeading|Qt.AlignLeft)
        self.label_8 = QLabel(self.groupBox_3)
        self.label_8.setObjectName(u"label_8")
        self.label_8.setGeometry(QRect(10, 61, 21, 20))
        self.label_8.setAlignment(Qt.AlignBottom|Qt.AlignRight|Qt.AlignTrailing)
        self.vitr_s_v = QDoubleSpinBox(self.groupBox_3)
        self.vitr_s_v.setObjectName(u"vitr_s_v")
        self.vitr_s_v.setGeometry(QRect(40, 60, 81, 26))
        self.vitr_s_v.setWrapping(True)
        self.vitr_s_v.setProperty("showGroupSeparator", False)
        self.vitr_s_v.setDecimals(1)
        self.vitr_s_v.setMinimum(0.000000000000000)
        self.vitr_s_v.setMaximum(25.000000000000000)
        self.vitr_s_v.setSingleStep(0.100000000000000)
        self.vitr_s_v.setStepType(QAbstractSpinBox.AdaptiveDecimalStepType)
        self.vitr_s_v.setValue(0.000000000000000)
        self.vitr_phi = QDoubleSpinBox(self.groupBox_3)
        self.vitr_phi.setObjectName(u"vitr_phi")
        self.vitr_phi.setGeometry(QRect(160, 29, 71, 26))
        self.vitr_phi.setWrapping(True)
        self.vitr_phi.setProperty("showGroupSeparator", False)
        self.vitr_phi.setDecimals(0)
        self.vitr_phi.setMinimum(0.000000000000000)
        self.vitr_phi.setMaximum(359.000000000000000)
        self.vitr_phi.setSingleStep(1.000000000000000)
        self.vitr_phi.setStepType(QAbstractSpinBox.DefaultStepType)
        self.vitr_phi.setValue(0.000000000000000)
        self.label_9 = QLabel(self.groupBox_3)
        self.label_9.setObjectName(u"label_9")
        self.label_9.setGeometry(QRect(130, 30, 21, 20))
        self.label_9.setAlignment(Qt.AlignBottom|Qt.AlignRight|Qt.AlignTrailing)
        self.vitr_s_phi = QDoubleSpinBox(self.groupBox_3)
        self.vitr_s_phi.setObjectName(u"vitr_s_phi")
        self.vitr_s_phi.setGeometry(QRect(160, 60, 71, 26))
        self.vitr_s_phi.setWrapping(True)
        self.vitr_s_phi.setProperty("showGroupSeparator", False)
        self.vitr_s_phi.setDecimals(0)
        self.vitr_s_phi.setMinimum(0.000000000000000)
        self.vitr_s_phi.setMaximum(359.000000000000000)
        self.vitr_s_phi.setSingleStep(1.000000000000000)
        self.vitr_s_phi.setStepType(QAbstractSpinBox.DefaultStepType)
        self.vitr_s_phi.setValue(0.000000000000000)
        self.label_10 = QLabel(self.groupBox_3)
        self.label_10.setObjectName(u"label_10")
        self.label_10.setGeometry(QRect(130, 61, 21, 20))
        self.label_10.setAlignment(Qt.AlignBottom|Qt.AlignRight|Qt.AlignTrailing)
        self.groupBox_4 = QGroupBox(self.tab_mise)
        self.groupBox_4.setObjectName(u"groupBox_4")
        self.groupBox_4.setGeometry(QRect(400, 280, 331, 101))
        self.pl_a = QDoubleSpinBox(self.groupBox_4)
        self.pl_a.setObjectName(u"pl_a")
        self.pl_a.setGeometry(QRect(70, 30, 81, 26))
        self.pl_a.setWrapping(True)
        self.pl_a.setProperty("showGroupSeparator", False)
        self.pl_a.setDecimals(0)
        self.pl_a.setMinimum(10.000000000000000)
        self.pl_a.setMaximum(200.000000000000000)
        self.pl_a.setSingleStep(1.000000000000000)
        self.pl_a.setStepType(QAbstractSpinBox.AdaptiveDecimalStepType)
        self.pl_a.setValue(70.000000000000000)
        self.label_11 = QLabel(self.groupBox_4)
        self.label_11.setObjectName(u"label_11")
        self.label_11.setGeometry(QRect(10, 31, 51, 20))
        self.label_11.setAlignment(Qt.AlignBottom|Qt.AlignRight|Qt.AlignTrailing)
        self.label_12 = QLabel(self.groupBox_4)
        self.label_12.setObjectName(u"label_12")
        self.label_12.setGeometry(QRect(170, 31, 71, 20))
        self.label_12.setAlignment(Qt.AlignBottom|Qt.AlignRight|Qt.AlignTrailing)
        self.pl_stin = QDoubleSpinBox(self.groupBox_4)
        self.pl_stin.setObjectName(u"pl_stin")
        self.pl_stin.setGeometry(QRect(250, 30, 71, 26))
        self.pl_stin.setWrapping(True)
        self.pl_stin.setProperty("showGroupSeparator", False)
        self.pl_stin.setDecimals(0)
        self.pl_stin.setMinimum(0.000000000000000)
        self.pl_stin.setMaximum(100.000000000000000)
        self.pl_stin.setSingleStep(1.000000000000000)
        self.pl_stin.setStepType(QAbstractSpinBox.AdaptiveDecimalStepType)
        self.pl_stin.setValue(0.000000000000000)
        self.pl_stin_sklon = QDoubleSpinBox(self.groupBox_4)
        self.pl_stin_sklon.setObjectName(u"pl_stin_sklon")
        self.pl_stin_sklon.setGeometry(QRect(250, 60, 71, 26))
        self.pl_stin_sklon.setWrapping(True)
        self.pl_stin_sklon.setProperty("showGroupSeparator", False)
        self.pl_stin_sklon.setDecimals(0)
        self.pl_stin_sklon.setMinimum(0.000000000000000)
        self.pl_stin_sklon.setMaximum(359.000000000000000)
        self.pl_stin_sklon.setSingleStep(1.000000000000000)
        self.pl_stin_sklon.setStepType(QAbstractSpinBox.DefaultStepType)
        self.pl_stin_sklon.setValue(0.000000000000000)
        self.label_13 = QLabel(self.groupBox_4)
        self.label_13.setObjectName(u"label_13")
        self.label_13.setGeometry(QRect(160, 60, 81, 20))
        self.label_13.setAlignment(Qt.AlignBottom|Qt.AlignRight|Qt.AlignTrailing)
        self.list_mise = QListView(self.tab_mise)
        self.list_mise.setObjectName(u"list_mise")
        self.list_mise.setGeometry(QRect(750, 100, 391, 281))
        self.textBox_nazev_mise = QPlainTextEdit(self.tab_mise)
        self.textBox_nazev_mise.setObjectName(u"textBox_nazev_mise")
        self.textBox_nazev_mise.setGeometry(QRect(810, 20, 331, 31))
        self.textBox_nazev_mise.setTabChangesFocus(True)
        self.textBox_nazev_mise.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.label_14 = QLabel(self.tab_mise)
        self.label_14.setObjectName(u"label_14")
        self.label_14.setGeometry(QRect(750, 26, 51, 21))
        self.label_14.setAlignment(Qt.AlignRight|Qt.AlignTop|Qt.AlignTrailing)
        self.but_save = QPushButton(self.tab_mise)
        self.but_save.setObjectName(u"but_save")
        self.but_save.setGeometry(QRect(850, 60, 89, 25))
        self.but_load = QPushButton(self.tab_mise)
        self.but_load.setObjectName(u"but_load")
        self.but_load.setGeometry(QRect(950, 60, 89, 25))
        self.but_delete = QPushButton(self.tab_mise)
        self.but_delete.setObjectName(u"but_delete")
        self.but_delete.setGeometry(QRect(1050, 60, 89, 25))
        self.but_start = QPushButton(self.tab_mise)
        self.but_start.setObjectName(u"but_start")
        self.but_start.setGeometry(QRect(750, 60, 71, 25))
        self.label_15 = QLabel(self.tab_mise)
        self.label_15.setObjectName(u"label_15")
        self.label_15.setGeometry(QRect(650, 190, 81, 81))
        self.label_15.setAlignment(Qt.AlignCenter)
        self.comboBox_alg = QComboBox(self.tab_mise)
        self.comboBox_alg.addItem("")
        self.comboBox_alg.setObjectName(u"comboBox_alg")
        self.comboBox_alg.setGeometry(QRect(490, 400, 241, 25))
        self.label_16 = QLabel(self.tab_mise)
        self.label_16.setObjectName(u"label_16")
        self.label_16.setGeometry(QRect(400, 400, 81, 20))
        self.label_16.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)
        self.tabWidget.addTab(self.tab_mise, "")
        self.tab_zive = QWidget()
        self.tab_zive.setObjectName(u"tab_zive")
        self.tab_zive.setEnabled(False)
        self.label_cam = QLabel(self.tab_zive)
        self.label_cam.setObjectName(u"label_cam")
        self.label_cam.setGeometry(QRect(10, 10, 320, 240))
        self.label_18 = QLabel(self.tab_zive)
        self.label_18.setObjectName(u"label_18")
        self.label_18.setGeometry(QRect(340, 10, 171, 17))
        self.label_18.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)
        self.label_exp_i = QLabel(self.tab_zive)
        self.label_exp_i.setObjectName(u"label_exp_i")
        self.label_exp_i.setGeometry(QRect(520, 10, 51, 17))
        self.label_19 = QLabel(self.tab_zive)
        self.label_19.setObjectName(u"label_19")
        self.label_19.setGeometry(QRect(340, 30, 171, 17))
        self.label_19.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)
        self.label_exp_N = QLabel(self.tab_zive)
        self.label_exp_N.setObjectName(u"label_exp_N")
        self.label_exp_N.setGeometry(QRect(520, 30, 51, 17))
        self.tabWidget.addTab(self.tab_zive, "")
        self.tab_experiment = QWidget()
        self.tab_experiment.setObjectName(u"tab_experiment")
        self.but_remove_mise = QPushButton(self.tab_experiment)
        self.but_remove_mise.setObjectName(u"but_remove_mise")
        self.but_remove_mise.setGeometry(QRect(890, 210, 31, 25))
        self.but_add_mise = QPushButton(self.tab_experiment)
        self.but_add_mise.setObjectName(u"but_add_mise")
        self.but_add_mise.setGeometry(QRect(850, 210, 31, 25))
        self.but_add_all = QPushButton(self.tab_experiment)
        self.but_add_all.setObjectName(u"but_add_all")
        self.but_add_all.setGeometry(QRect(810, 210, 31, 25))
        self.but_remove_all = QPushButton(self.tab_experiment)
        self.but_remove_all.setObjectName(u"but_remove_all")
        self.but_remove_all.setGeometry(QRect(930, 210, 31, 25))
        self.groupBox_5 = QGroupBox(self.tab_experiment)
        self.groupBox_5.setObjectName(u"groupBox_5")
        self.groupBox_5.setGeometry(QRect(10, 10, 561, 191))
        self.list_experimenty = QColumnView(self.groupBox_5)
        self.list_experimenty.setObjectName(u"list_experimenty")
        self.list_experimenty.setGeometry(QRect(170, 70, 341, 111))
        self.but_delete_experiment = QPushButton(self.groupBox_5)
        self.but_delete_experiment.setObjectName(u"but_delete_experiment")
        self.but_delete_experiment.setGeometry(QRect(70, 160, 91, 25))
        self.but_load_experiment = QPushButton(self.groupBox_5)
        self.but_load_experiment.setObjectName(u"but_load_experiment")
        self.but_load_experiment.setGeometry(QRect(70, 130, 91, 25))
        self.but_save_experiment = QPushButton(self.groupBox_5)
        self.but_save_experiment.setObjectName(u"but_save_experiment")
        self.but_save_experiment.setGeometry(QRect(70, 100, 91, 25))
        self.but_start_experiment = QPushButton(self.groupBox_5)
        self.but_start_experiment.setObjectName(u"but_start_experiment")
        self.but_start_experiment.setGeometry(QRect(70, 70, 91, 25))
        self.textBox_nazev_experimentu = QPlainTextEdit(self.groupBox_5)
        self.textBox_nazev_experimentu.setObjectName(u"textBox_nazev_experimentu")
        self.textBox_nazev_experimentu.setGeometry(QRect(70, 30, 441, 31))
        self.textBox_nazev_experimentu.setTabChangesFocus(True)
        self.textBox_nazev_experimentu.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.label_17 = QLabel(self.groupBox_5)
        self.label_17.setObjectName(u"label_17")
        self.label_17.setGeometry(QRect(10, 40, 51, 21))
        self.label_17.setAlignment(Qt.AlignRight|Qt.AlignTop|Qt.AlignTrailing)
        self.groupBox_6 = QGroupBox(self.tab_experiment)
        self.groupBox_6.setObjectName(u"groupBox_6")
        self.groupBox_6.setGeometry(QRect(600, 10, 551, 191))
        self.list_mise_2 = QListView(self.groupBox_6)
        self.list_mise_2.setObjectName(u"list_mise_2")
        self.list_mise_2.setGeometry(QRect(10, 30, 531, 151))
        self.list_mise_2.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.groupBox_7 = QGroupBox(self.tab_experiment)
        self.groupBox_7.setObjectName(u"groupBox_7")
        self.groupBox_7.setGeometry(QRect(9, 219, 1141, 301))
        self.table_experiment = QTableView(self.groupBox_7)
        self.table_experiment.setObjectName(u"table_experiment")
        self.table_experiment.setGeometry(QRect(10, 30, 1121, 261))
        self.table_experiment.setEditTriggers(QAbstractItemView.AnyKeyPressed|QAbstractItemView.DoubleClicked|QAbstractItemView.SelectedClicked)
        self.table_experiment.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table_experiment.setSortingEnabled(False)
        self.table_experiment.horizontalHeader().setProperty("showSortIndicator", False)
        self.table_experiment.verticalHeader().setVisible(False)
        self.tabWidget.addTab(self.tab_experiment, "")
        self.groupBox_7.raise_()
        self.groupBox_5.raise_()
        self.but_remove_mise.raise_()
        self.but_add_mise.raise_()
        self.but_add_all.raise_()
        self.but_remove_all.raise_()
        self.groupBox_6.raise_()
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 1166, 22))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)
#if QT_CONFIG(shortcut)
        self.label_tag.setBuddy(self.pl_x)
        self.label.setBuddy(self.pl_x)
        self.label_2.setBuddy(self.pl_y)
        self.label_3.setBuddy(self.pl_phi)
        self.label_4.setBuddy(self.uav_x)
        self.label_5.setBuddy(self.uav_y)
        self.label_6.setBuddy(self.uav_phi)
        self.label_7.setBuddy(self.vitr_v)
        self.label_8.setBuddy(self.vitr_s_v)
        self.label_9.setBuddy(self.vitr_phi)
        self.label_10.setBuddy(self.vitr_s_phi)
        self.label_11.setBuddy(self.pl_a)
        self.label_12.setBuddy(self.pl_stin)
        self.label_13.setBuddy(self.pl_stin_sklon)
        self.label_14.setBuddy(self.textBox_nazev_mise)
        self.label_16.setBuddy(self.comboBox_alg)
        self.label_17.setBuddy(self.textBox_nazev_mise)
#endif // QT_CONFIG(shortcut)
        QWidget.setTabOrder(self.tabWidget, self.pl_x)
        QWidget.setTabOrder(self.pl_x, self.pl_y)
        QWidget.setTabOrder(self.pl_y, self.pl_phi)
        QWidget.setTabOrder(self.pl_phi, self.uav_x)
        QWidget.setTabOrder(self.uav_x, self.uav_y)
        QWidget.setTabOrder(self.uav_y, self.uav_phi)
        QWidget.setTabOrder(self.uav_phi, self.vitr_v)
        QWidget.setTabOrder(self.vitr_v, self.vitr_phi)
        QWidget.setTabOrder(self.vitr_phi, self.vitr_s_v)
        QWidget.setTabOrder(self.vitr_s_v, self.vitr_s_phi)
        QWidget.setTabOrder(self.vitr_s_phi, self.pl_a)
        QWidget.setTabOrder(self.pl_a, self.pl_stin)
        QWidget.setTabOrder(self.pl_stin, self.pl_stin_sklon)
        QWidget.setTabOrder(self.pl_stin_sklon, self.comboBox_alg)
        QWidget.setTabOrder(self.comboBox_alg, self.textBox_nazev_mise)
        QWidget.setTabOrder(self.textBox_nazev_mise, self.but_save)
        QWidget.setTabOrder(self.but_save, self.list_mise)
        QWidget.setTabOrder(self.list_mise, self.but_load)
        QWidget.setTabOrder(self.but_load, self.but_delete)
        QWidget.setTabOrder(self.but_delete, self.but_start)
        QWidget.setTabOrder(self.but_start, self.textBox_nazev_experimentu)
        QWidget.setTabOrder(self.textBox_nazev_experimentu, self.but_save_experiment)
        QWidget.setTabOrder(self.but_save_experiment, self.list_experimenty)
        QWidget.setTabOrder(self.list_experimenty, self.but_load_experiment)
        QWidget.setTabOrder(self.but_load_experiment, self.but_delete_experiment)
        QWidget.setTabOrder(self.but_delete_experiment, self.but_start_experiment)
        QWidget.setTabOrder(self.but_start_experiment, self.table_experiment)
        QWidget.setTabOrder(self.table_experiment, self.but_remove_mise)
        QWidget.setTabOrder(self.but_remove_mise, self.but_remove_all)
        QWidget.setTabOrder(self.but_remove_all, self.list_mise_2)
        QWidget.setTabOrder(self.list_mise_2, self.but_add_mise)
        QWidget.setTabOrder(self.but_add_mise, self.but_add_all)

        self.retranslateUi(MainWindow)

        self.tabWidget.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
#if QT_CONFIG(accessibility)
        self.mapa.setAccessibleName(QCoreApplication.translate("MainWindow", u"Mapa", None))
#endif // QT_CONFIG(accessibility)
#if QT_CONFIG(accessibility)
        self.mapa.setAccessibleDescription(QCoreApplication.translate("MainWindow", u"Mapa rozlo\u017een\u00ed objekt\u016f v misi", None))
#endif // QT_CONFIG(accessibility)
        self.label_uav.setText(QCoreApplication.translate("MainWindow", u"\u2723", None))
        self.label_tag.setText("")
        self.groupBox.setTitle(QCoreApplication.translate("MainWindow", u"Poloha plo\u0161iny", None))
#if QT_CONFIG(whatsthis)
        self.pl_x.setWhatsThis(QCoreApplication.translate("MainWindow", u"Poloha plo\u0161iny na ose X vzhledem k po\u010d\u00e1tku sou\u0159adnic sv\u011bta.", None))
#endif // QT_CONFIG(whatsthis)
#if QT_CONFIG(accessibility)
        self.pl_x.setAccessibleName(QCoreApplication.translate("MainWindow", u"X", None))
#endif // QT_CONFIG(accessibility)
        self.pl_x.setSuffix(QCoreApplication.translate("MainWindow", u" m", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"X:", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"Y:", None))
#if QT_CONFIG(whatsthis)
        self.pl_y.setWhatsThis(QCoreApplication.translate("MainWindow", u"Poloha plo\u0161iny na ose X vzhledem k po\u010d\u00e1tku sou\u0159adnic sv\u011bta.", None))
#endif // QT_CONFIG(whatsthis)
#if QT_CONFIG(accessibility)
        self.pl_y.setAccessibleName(QCoreApplication.translate("MainWindow", u"X", None))
#endif // QT_CONFIG(accessibility)
        self.pl_y.setSuffix(QCoreApplication.translate("MainWindow", u" m", None))
#if QT_CONFIG(whatsthis)
        self.pl_phi.setWhatsThis(QCoreApplication.translate("MainWindow", u"Poloha plo\u0161iny na ose X vzhledem k po\u010d\u00e1tku sou\u0159adnic sv\u011bta.", None))
#endif // QT_CONFIG(whatsthis)
#if QT_CONFIG(accessibility)
        self.pl_phi.setAccessibleName(QCoreApplication.translate("MainWindow", u"phi", None))
#endif // QT_CONFIG(accessibility)
        self.pl_phi.setSuffix(QCoreApplication.translate("MainWindow", u" \u02da", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"\u03c6:", None))
        self.groupBox_2.setTitle(QCoreApplication.translate("MainWindow", u"Poloha letadla", None))
#if QT_CONFIG(whatsthis)
        self.uav_x.setWhatsThis(QCoreApplication.translate("MainWindow", u"Poloha plo\u0161iny na ose X vzhledem k po\u010d\u00e1tku sou\u0159adnic sv\u011bta.", None))
#endif // QT_CONFIG(whatsthis)
#if QT_CONFIG(accessibility)
        self.uav_x.setAccessibleName(QCoreApplication.translate("MainWindow", u"X", None))
#endif // QT_CONFIG(accessibility)
        self.uav_x.setSuffix(QCoreApplication.translate("MainWindow", u" m", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"X:", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"Y:", None))
#if QT_CONFIG(whatsthis)
        self.uav_y.setWhatsThis(QCoreApplication.translate("MainWindow", u"Poloha plo\u0161iny na ose X vzhledem k po\u010d\u00e1tku sou\u0159adnic sv\u011bta.", None))
#endif // QT_CONFIG(whatsthis)
#if QT_CONFIG(accessibility)
        self.uav_y.setAccessibleName(QCoreApplication.translate("MainWindow", u"X", None))
#endif // QT_CONFIG(accessibility)
        self.uav_y.setSuffix(QCoreApplication.translate("MainWindow", u" m", None))
#if QT_CONFIG(whatsthis)
        self.uav_phi.setWhatsThis(QCoreApplication.translate("MainWindow", u"Poloha plo\u0161iny na ose X vzhledem k po\u010d\u00e1tku sou\u0159adnic sv\u011bta.", None))
#endif // QT_CONFIG(whatsthis)
#if QT_CONFIG(accessibility)
        self.uav_phi.setAccessibleName(QCoreApplication.translate("MainWindow", u"phi", None))
#endif // QT_CONFIG(accessibility)
        self.uav_phi.setSuffix(QCoreApplication.translate("MainWindow", u" \u02da", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"\u03c6:", None))
        self.groupBox_3.setTitle(QCoreApplication.translate("MainWindow", u"V\u00edtr", None))
#if QT_CONFIG(whatsthis)
        self.vitr_v.setWhatsThis(QCoreApplication.translate("MainWindow", u"Poloha plo\u0161iny na ose X vzhledem k po\u010d\u00e1tku sou\u0159adnic sv\u011bta.", None))
#endif // QT_CONFIG(whatsthis)
#if QT_CONFIG(accessibility)
        self.vitr_v.setAccessibleName(QCoreApplication.translate("MainWindow", u"X", None))
#endif // QT_CONFIG(accessibility)
        self.vitr_v.setSuffix(QCoreApplication.translate("MainWindow", u" m/s", None))
        self.label_7.setText(QCoreApplication.translate("MainWindow", u"v:", None))
        self.label_8.setText(QCoreApplication.translate("MainWindow", u"\u03c3<sub>v</sub>:", None))
#if QT_CONFIG(whatsthis)
        self.vitr_s_v.setWhatsThis(QCoreApplication.translate("MainWindow", u"Poloha plo\u0161iny na ose X vzhledem k po\u010d\u00e1tku sou\u0159adnic sv\u011bta.", None))
#endif // QT_CONFIG(whatsthis)
#if QT_CONFIG(accessibility)
        self.vitr_s_v.setAccessibleName(QCoreApplication.translate("MainWindow", u"X", None))
#endif // QT_CONFIG(accessibility)
        self.vitr_s_v.setSuffix(QCoreApplication.translate("MainWindow", u" m/s", None))
#if QT_CONFIG(whatsthis)
        self.vitr_phi.setWhatsThis(QCoreApplication.translate("MainWindow", u"Poloha plo\u0161iny na ose X vzhledem k po\u010d\u00e1tku sou\u0159adnic sv\u011bta.", None))
#endif // QT_CONFIG(whatsthis)
#if QT_CONFIG(accessibility)
        self.vitr_phi.setAccessibleName(QCoreApplication.translate("MainWindow", u"phi", None))
#endif // QT_CONFIG(accessibility)
        self.vitr_phi.setSuffix(QCoreApplication.translate("MainWindow", u" \u02da", None))
        self.label_9.setText(QCoreApplication.translate("MainWindow", u"\u03c6:", None))
#if QT_CONFIG(whatsthis)
        self.vitr_s_phi.setWhatsThis(QCoreApplication.translate("MainWindow", u"Poloha plo\u0161iny na ose X vzhledem k po\u010d\u00e1tku sou\u0159adnic sv\u011bta.", None))
#endif // QT_CONFIG(whatsthis)
#if QT_CONFIG(accessibility)
        self.vitr_s_phi.setAccessibleName(QCoreApplication.translate("MainWindow", u"phi", None))
#endif // QT_CONFIG(accessibility)
        self.vitr_s_phi.setSuffix(QCoreApplication.translate("MainWindow", u" \u02da", None))
        self.label_10.setText(QCoreApplication.translate("MainWindow", u"\u03c3<sub>\u03c6</sub>:", None))
        self.groupBox_4.setTitle(QCoreApplication.translate("MainWindow", u"Plo\u0161ina", None))
#if QT_CONFIG(whatsthis)
        self.pl_a.setWhatsThis(QCoreApplication.translate("MainWindow", u"Poloha plo\u0161iny na ose X vzhledem k po\u010d\u00e1tku sou\u0159adnic sv\u011bta.", None))
#endif // QT_CONFIG(whatsthis)
#if QT_CONFIG(accessibility)
        self.pl_a.setAccessibleName(QCoreApplication.translate("MainWindow", u"X", None))
#endif // QT_CONFIG(accessibility)
        self.pl_a.setSuffix(QCoreApplication.translate("MainWindow", u" cm", None))
        self.label_11.setText(QCoreApplication.translate("MainWindow", u"strana:", None))
        self.label_12.setText(QCoreApplication.translate("MainWindow", u"zast\u00edn\u011bn\u00ed:", None))
#if QT_CONFIG(whatsthis)
        self.pl_stin.setWhatsThis(QCoreApplication.translate("MainWindow", u"Poloha plo\u0161iny na ose X vzhledem k po\u010d\u00e1tku sou\u0159adnic sv\u011bta.", None))
#endif // QT_CONFIG(whatsthis)
#if QT_CONFIG(accessibility)
        self.pl_stin.setAccessibleName(QCoreApplication.translate("MainWindow", u"X", None))
#endif // QT_CONFIG(accessibility)
        self.pl_stin.setSuffix(QCoreApplication.translate("MainWindow", u" %", None))
#if QT_CONFIG(whatsthis)
        self.pl_stin_sklon.setWhatsThis(QCoreApplication.translate("MainWindow", u"Poloha plo\u0161iny na ose X vzhledem k po\u010d\u00e1tku sou\u0159adnic sv\u011bta.", None))
#endif // QT_CONFIG(whatsthis)
#if QT_CONFIG(accessibility)
        self.pl_stin_sklon.setAccessibleName(QCoreApplication.translate("MainWindow", u"phi", None))
#endif // QT_CONFIG(accessibility)
        self.pl_stin_sklon.setSuffix(QCoreApplication.translate("MainWindow", u" \u02da", None))
        self.label_13.setText(QCoreApplication.translate("MainWindow", u"sklon st\u00ednu:", None))
        self.textBox_nazev_mise.setPlaceholderText(QCoreApplication.translate("MainWindow", u"N\u00e1zev mise", None))
        self.label_14.setText(QCoreApplication.translate("MainWindow", u"N\u00e1zev:", None))
        self.but_save.setText(QCoreApplication.translate("MainWindow", u"Ulo\u017eit", None))
        self.but_load.setText(QCoreApplication.translate("MainWindow", u"Na\u010d\u00edst", None))
        self.but_delete.setText(QCoreApplication.translate("MainWindow", u"Odstranit", None))
        self.but_start.setText(QCoreApplication.translate("MainWindow", u"\u25b6 Start", None))
        self.label_15.setText(QCoreApplication.translate("MainWindow", u"Preview", None))
        self.comboBox_alg.setItemText(0, QCoreApplication.translate("MainWindow", u"Z\u00e1kladn\u00ed", None))

        self.label_16.setText(QCoreApplication.translate("MainWindow", u"Algoritmus:", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_mise), QCoreApplication.translate("MainWindow", u"Mise", None))
        self.label_cam.setText("")
        self.label_18.setText(QCoreApplication.translate("MainWindow", u"Experiment - index:", None))
        self.label_exp_i.setText(QCoreApplication.translate("MainWindow", u"-", None))
        self.label_19.setText(QCoreApplication.translate("MainWindow", u"Experiment - opakov\u00e1n\u00ed:", None))
        self.label_exp_N.setText(QCoreApplication.translate("MainWindow", u"-", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_zive), QCoreApplication.translate("MainWindow", u"\u017div\u011b", None))
        self.but_remove_mise.setText(QCoreApplication.translate("MainWindow", u"\u2191", None))
        self.but_add_mise.setText(QCoreApplication.translate("MainWindow", u"\u2193", None))
        self.but_add_all.setText(QCoreApplication.translate("MainWindow", u"\u21ca", None))
        self.but_remove_all.setText(QCoreApplication.translate("MainWindow", u"\u21c8", None))
        self.groupBox_5.setTitle(QCoreApplication.translate("MainWindow", u"Ulo\u017een\u00e9 experimenty", None))
        self.but_delete_experiment.setText(QCoreApplication.translate("MainWindow", u"Odstranit", None))
        self.but_load_experiment.setText(QCoreApplication.translate("MainWindow", u"Na\u010d\u00edst", None))
        self.but_save_experiment.setText(QCoreApplication.translate("MainWindow", u"Ulo\u017eit", None))
        self.but_start_experiment.setText(QCoreApplication.translate("MainWindow", u"\u25b6 Start", None))
        self.textBox_nazev_experimentu.setPlaceholderText(QCoreApplication.translate("MainWindow", u"N\u00e1zev experimentu", None))
        self.label_17.setText(QCoreApplication.translate("MainWindow", u"N\u00e1zev:", None))
        self.groupBox_6.setTitle(QCoreApplication.translate("MainWindow", u"V\u00fdb\u011br mis\u00ed", None))
        self.groupBox_7.setTitle(QCoreApplication.translate("MainWindow", u"Definice experimentu", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_experiment), QCoreApplication.translate("MainWindow", u"Experiment", None))
    # retranslateUi

