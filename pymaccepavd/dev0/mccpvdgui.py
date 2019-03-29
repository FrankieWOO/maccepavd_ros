# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mccpvdgui.ui'
#
# Created: Mon Jun  5 22:28:10 2017
#      by: pyside-uic 0.2.15 running on PySide 1.2.2
#
# WARNING! All changes made in this file will be lost!

import sys
from PySide import QtCore, QtGui
from pyqtgraph import PlotWidget

class Ui_mccpvdgui(object):
    def setupUi(self, mccpvdgui):
        mccpvdgui.setObjectName("mccpvdgui")
        mccpvdgui.resize(800, 600)
        self.centralWidget = QtGui.QWidget(mccpvdgui)
        self.centralWidget.setObjectName("centralWidget")
        self.viewJoint = PlotWidget(self.centralWidget)
        self.viewJoint.setGeometry(QtCore.QRect(10, 20, 381, 161))
        self.viewJoint.setObjectName("viewJoint")
        self.viewVelocity = PlotWidget(self.centralWidget)
        self.viewVelocity.setGeometry(QtCore.QRect(10, 190, 381, 161))
        self.viewVelocity.setObjectName("viewVelocity")
        self.viewAccel = PlotWidget(self.centralWidget)
        self.viewAccel.setGeometry(QtCore.QRect(10, 360, 381, 161))
        self.viewAccel.setObjectName("viewAccel")
        self.viewServo1 = PlotWidget(self.centralWidget)
        self.viewServo1.setGeometry(QtCore.QRect(410, 20, 381, 161))
        self.viewServo1.setObjectName("viewServo1")
        self.viewServo2 = PlotWidget(self.centralWidget)
        self.viewServo2.setGeometry(QtCore.QRect(410, 190, 381, 161))
        self.viewServo2.setObjectName("viewServo2")
        self.viewCurrent = PlotWidget(self.centralWidget)
        self.viewCurrent.setGeometry(QtCore.QRect(410, 360, 381, 161))
        self.viewCurrent.setObjectName("viewCurrent")
        mccpvdgui.setCentralWidget(self.centralWidget)
        self.menuBar = QtGui.QMenuBar(mccpvdgui)
        self.menuBar.setGeometry(QtCore.QRect(0, 0, 800, 22))
        self.menuBar.setObjectName("menuBar")
        mccpvdgui.setMenuBar(self.menuBar)
        self.mainToolBar = QtGui.QToolBar(mccpvdgui)
        self.mainToolBar.setObjectName("mainToolBar")
        mccpvdgui.addToolBar(QtCore.Qt.TopToolBarArea, self.mainToolBar)
        self.statusBar = QtGui.QStatusBar(mccpvdgui)
        self.statusBar.setObjectName("statusBar")
        mccpvdgui.setStatusBar(self.statusBar)

        self.retranslateUi(mccpvdgui)
        QtCore.QMetaObject.connectSlotsByName(mccpvdgui)

    def retranslateUi(self, mccpvdgui):
        mccpvdgui.setWindowTitle(QtGui.QApplication.translate("mccpvdgui", "mccpvdgui", None, QtGui.QApplication.UnicodeUTF8))


class ControlWindow(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(ControlWindow, self).__init__(parent)
        self.ui = Ui_mccpvdgui()
        self.ui.setupUi(self)
        self.ui.viewJoint.setYRange(-1,1)
        #mySW.ui.viewVelocity.setYRange(-1,1)
        #mySW.ui.viewAccel.setYRange(-1,1)
        self.ui.viewServo1.setYRange(-1,1)
        self.ui.viewServo2.setYRange(-0.2,1.5)
        self.ui.viewCurrent.setYRange(-4,4)

if __name__=="__main__":
    app = QtGui.QApplication(sys.argv)
    mySW = ControlWindow()
    mySW.show()
    sys.exit(app.exec_())
