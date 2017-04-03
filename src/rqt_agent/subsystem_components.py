#!/usr/bin/env python

# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import division
import os
import math
import subprocess

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot, QRectF, QPointF, QSize, QRect, QPoint
from python_qt_binding.QtWidgets import QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QListWidgetItem, QDialog, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem, QGraphicsPathItem, QTableWidgetItem, QHeaderView, QStyle, QCommonStyle
from python_qt_binding.QtGui import QColor, QPen, QBrush, QPainterPath, QPolygonF, QTransform, QPainter, QIcon, QPixmap, QPaintEvent, QPalette
from python_qt_binding.QtSvg import QSvgGenerator
import roslib
import rospkg
import rospy
from rospy.exceptions import ROSException

import xml.dom.minidom as minidom
import tempfile

from rqt_topic.topic_info import TopicInfo

from subsystem_msgs.srv import *

def getComponentBrush(state):
    b = QBrush(QColor(0,0,255)) # unconfigured/preoperational
    if state == 'S':      # stopped
        b = QBrush(QColor(255,255,255))
    elif state == 'R':    # running
        b = QBrush(QColor(0,255,0))
    elif state == 'E':    # error
        b = QBrush(QColor(255,0,0))
    elif state == 'F':    # fatal error
        b = QBrush(QColor(255,127,127))
    elif state == 'X':    # exception
        b = QBrush(QColor(0,255,255))
    return b

def getComponentTooltip(state):
    tooltip = 'unconfigured/preoperational'
    if state == 'S':      # stopped
        tooltip = 'stopped'
    elif state == 'R':    # running
        tooltip = 'running'
    elif state == 'E':    # error
        tooltip = 'error'
    elif state == 'F':    # fatal error
        tooltip = 'fatal error'
    elif state == 'X':    # exception
        tooltip = 'exception'
    return tooltip

class ComponentsDialog(QDialog):

    @Slot()
    def closeClick(self):
        self.close()

    def __init__(self, subsystem_name, parent=None):
        super(ComponentsDialog, self).__init__(parent)

        self.parent = parent

        self.setWindowFlags(Qt.Window)

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_agent'), 'resource', 'SubsystemComponents.ui')
        loadUi(ui_file, self)

        self.setWindowTitle(subsystem_name + " - state history")

        self.pushButton_close.clicked.connect(self.closeClick)

        self.components = {}

    def updateState(self, components_state, components_diag_msgs):
        # iterate through components and add them to QListWidget
        for comp in components_state:
            if not comp in self.components:
                item = QListWidgetItem()
                item.setText(comp)
                self.components[comp] = item
                self.listWidget.addItem(item)
            self.components[comp].setBackground( getComponentBrush(components_state[comp]) )
            self.components[comp].setToolTip( getComponentTooltip(components_state[comp]) )

        for comp in components_diag_msgs:
            if comp in self.components:
                self.components[comp].setText(comp + ' (' + components_diag_msgs[comp] + ')')

