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

class GraphScene(QGraphicsScene):
    def __init__(self, rect):
        super(GraphScene, self).__init__(rect)

class GraphView(QGraphicsView):

    comp_select_signal = Signal(str)

    def __init__ (self, parent = None):
        super (GraphView, self).__init__ (parent)
        self.parent = parent
        self.transform_press = None
        self.mousePressPos = None
        self.setTransformationAnchor(QGraphicsView.NoAnchor)
        self.setResizeAnchor(QGraphicsView.NoAnchor)

    def isPanActive(self):
        return (self.transform_press != None and self.mousePressPos != None)

    def wheelEvent (self, event):
        if self.isPanActive():
            return

        oldPos = self.mapToScene(event.pos())
        self.translate(oldPos.x(),oldPos.y())

        factor = 1.2
        if event.angleDelta().y() < 0 :
            factor = 1.0 / factor
        self.scale(factor, factor)

        # Get the new position
        newPos = self.mapToScene(event.pos())
        # Move scene to old position
        delta = newPos - oldPos
        self.translate(delta.x(), delta.y())

    def selectComponent(self, name):
#        print "selected component: ", name
        self.comp_select_signal.emit(name)

    def mousePressEvent(self, event):
        if event.button() == Qt.MidButton:
            self.transform_press = self.transform()
            self.mousePressPos = event.pos()
        else:
            super(GraphView, self).mousePressEvent(event)
            selected = False
            for item in self.items():
                if type(item) is QGraphicsEllipseItem:
                    if item.contains( self.mapToScene(event.pos()) ):
                        self.selectComponent(item.data(0))
                        selected = True
                        break
                elif type(item) is QGraphicsPathItem:
                    if item.contains( self.mapToScene(event.pos()) ):
                        print "connections: ", item.data(0)
            if not selected:
                self.selectComponent(None)

    def mouseMoveEvent(self, event):
        if event.buttons() == Qt.MidButton:
            mouseMovePos = event.pos()
            view_diff = (mouseMovePos - self.mousePressPos)
            tf = self.transform_press
            scene_diff = view_diff
            new_tf = QTransform(tf.m11(), tf.m12(), tf.m21(), tf.m22(), tf.dx()+view_diff.x(), tf.dy()+view_diff.y())
            self.setTransform(new_tf)
        else:
            super(GraphView, self).mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MidButton:
            self.transform_press = None
            self.mousePressPos = None
        else:
            super(GraphView, self).mouseReleaseEvent(event)

class GraphDialog(QDialog):

    def scX(self, x):
        return self.scale_factor * float(x)

    def scY(self, y):
        return self.scale_factor * float(y)

    def tfX(self, x):
        return self.scale_factor * float(x)

    def tfY(self, y):
        return self.scale_factor * (self.height - float(y))

    @Slot(int)
    def portSelected(self, index):
        for e in self.prev_selected_connections:
            e.setPen( QPen(QBrush(QColor(0,0,0)), 0) )
        self.prev_selected_connections = []

        if not self.component_selected:
            return

        for conn in self.parent.all_connections:
            if (conn[0] == self.component_selected and conn[1] == self.selected_component_port_names[index]) or \
                (conn[2] == self.component_selected and conn[3] == self.selected_component_port_names[index]):
                for graph_name in self.edges:
                    for e in self.edges[graph_name]:
                        data = e.data(0)
                        if (data[0] == conn[0] and data[1] == conn[2]) or \
                            (data[0] == conn[2] and data[1] == conn[0]):
                            e.setPen( QPen(QBrush(QColor(255,0,0)), 5) )
                            self.prev_selected_connections.append(e)

    @Slot(str)
    def componentSelected(self, name):
        self.comboBoxConnections.clear()
        self.component_selected = name

        if name == None:
            self.labelSelectedComponent.setText('')
            return

        self.labelSelectedComponent.setText(name)
        for comp in self.parent.subsystem_info.components:
            if comp.name == name:
                self.selected_component_port_names = []
                for p in comp.ports:
                    self.selected_component_port_names.append(p.name)
                    port_str = ''
                    if p.is_input:
                        port_str += '[IN]'
                    else:
                        port_str += '[OUT]'
                    port_str += ' ' + p.name
                    if p.is_connected:
                        port_str += ' <conncected>'
                    else:
                        port_str += ' <not conncected>'
                    type_str = ''
                    for tn in p.type_names:
                        type_str += ' ' + tn
                    if len(type_str) == 0:
                        type_str = 'unknown type'
                    port_str += ', type:' + type_str
                    self.comboBoxConnections.addItem(port_str)
                break

    @Slot(int)
    def graphSelected(self, index):
        graph_name = self.comboBoxGraphs.itemText(index)
        self.showGraph(graph_name)

    def __init__(self, subsystem_name, parent=None):
        super(GraphDialog, self).__init__(parent)

        self.parent = parent

        self.setWindowFlags(Qt.Window)

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_agent'), 'resource', 'GraphVis.ui')
        loadUi(ui_file, self)

        self.setWindowTitle(subsystem_name + " - transition function")
        self.components_state = None
        self.initialized = False

        self.pushButton_export.clicked.connect(self.exportClick)
        self.pushButton_reset_view.clicked.connect(self.reset_viewClick)

        self.pushButton_close.clicked.connect(self.closeClick)
        self.pushButton_zoom_in.clicked.connect(self.zoomInClick)
        self.pushButton_zoom_out.clicked.connect(self.zoomOutClick)

        self.prev_selected_connections = []
        self.comboBoxConnections.highlighted.connect(self.portSelected)
        self.comboBoxGraphs.highlighted.connect(self.graphSelected)

        self.componentSelected(None)
        self.scene = {}
        self.graphicsView = None
        self.nodes = {}
        self.edges = {}

    def showGraph(self, graph_name):
        if not graph_name in self.scene:
            print "could not show graph " + graph_name
            return

        if not self.graphicsView:
            self.graphicsView = GraphView()
            self.verticalLayout.insertWidget(0, self.graphicsView)

        self.graphicsView.setScene(self.scene[graph_name])

        self.graphicsView.comp_select_signal.connect(self.componentSelected)
        self.initialized = True

    def addGraph(self, graph_name, graph_str):

        self.comboBoxGraphs.addItem(graph_name)

        graph = graph_str.splitlines()

        header = graph[0].split()
        if header[0] != 'graph':
            raise Exception('wrong graph format', 'header is: ' + graph[0])

        self.scale_factor = 100.0
        self.width = float(header[2])
        self.height = float(header[3])
        print "QGraphicsScene size:", self.width, self.height

        self.scene[graph_name] = GraphScene(QRectF(0, 0, self.scX(self.width), self.scY(self.height)))

        self.nodes[graph_name] = {}
        self.edges[graph_name] = []

        for l in graph:
            items = l.split()
            if len(items) == 0:
                continue
            elif items[0] == 'stop':
                break
            elif items[0] == 'node':
                #node CImp 16.472 5.25 0.86659 0.5 CImp filled ellipse lightblue lightblue
                if len(items) != 11:
                    raise Exception('wrong number of items in line', 'line is: ' + l)
                name = items[6]
                if name == "\"\"":
                    name = ""
                w = self.scX(items[4])
                h = self.scY(items[5])
                x = self.tfX(items[2])
                y = self.tfY(items[3])

                self.nodes[graph_name][name] = self.scene[graph_name].addEllipse(x - w/2, y - h/2, w, h)
                self.nodes[graph_name][name].setData(0, name)
                text_item = self.scene[graph_name].addSimpleText(name)
                br = text_item.boundingRect()
                text_item.setPos(x - br.width()/2, y - br.height()/2)

            elif items[0] == 'edge':
                # without label:
                # edge CImp Ts 4 16.068 5.159 15.143 4.9826 12.876 4.5503 11.87 4.3583 solid black
                #
                # with label:
                # edge b_stSplit TorsoVelAggregate 7 7.5051 6.3954 7.7054 6.3043 7.9532 6.1899 8.1728 6.0833 8.4432 5.9522 8.7407 5.8012 8.9885 5.6735 aa 8.6798 5.9792 solid black
                line_len = int(items[3])
                label_text = None
                label_pos = None
                if (line_len * 2 + 6) == len(items):
                    # no label
                    pass
                elif (line_len * 2 + 9) == len(items):
                    # edge with label
                    label_text = items[4 + line_len*2]
                    label_pos = QPointF(self.tfX(items[4 + line_len*2 + 1]), self.tfY(items[4 + line_len*2 + 2]))
                else:
                    raise Exception('wrong number of items in line', 'should be: ' + str(line_len * 2 + 6) + " or " + str(line_len * 2 + 9) + ', line is: ' + l)
                    
                line = []
                for i in range(line_len):
                    line.append( (self.tfX(items[4+i*2]), self.tfY(items[5+i*2])) )
                control_points_idx = 1
                path = QPainterPath(QPointF(line[0][0], line[0][1]))
                while True:
                    q1 = line[control_points_idx]
                    q2 = line[control_points_idx+1]
                    p2 = line[control_points_idx+2]
                    path.cubicTo( q1[0], q1[1], q2[0], q2[1], p2[0], p2[1] )
                    control_points_idx = control_points_idx + 3
                    if control_points_idx >= len(line):
                        break
                edge = self.scene[graph_name].addPath(path)
                edge.setData(0, (items[1], items[2]))
                self.edges[graph_name].append(edge)

                end_p = QPointF(line[-1][0], line[-1][1])
                p0 = end_p - QPointF(line[-2][0], line[-2][1])
                p0_norm = math.sqrt(p0.x()*p0.x() + p0.y()*p0.y())
                p0 = p0 / p0_norm
                p0 = p0 * self.scale_factor * 0.15
                p1 = QPointF(p0.y(), -p0.x()) * 0.25
                p2 = -p1

                poly = QPolygonF()
                poly.append(p0+end_p)
                poly.append(p1+end_p)
                poly.append(p2+end_p)
                poly.append(p0+end_p)

#                poly_path = QPainterPath()
#                poly_path.addPolygon(poly)
#                painter = QPainter()
                self.scene[graph_name].addPolygon(poly)

                if label_text and label_pos:
                    if label_text[0] == "\"":
                        label_text = label_text[1:]
                    if label_text[-1] == "\"":
                        label_text = label_text[:-1]
                    label_text = label_text.replace("\\n", "\n")
                    label_item = self.scene[graph_name].addSimpleText(label_text)
                    br = label_item.boundingRect()
                    label_item.setPos(label_pos.x() - br.width()/2, label_pos.y() - br.height()/2)

#        svgGen = QSvgGenerator()
#        svgGen.setFileName( graph_name + ".svg" )
#        svgGen.setSize(QSize(self.scX(self.width), self.scY(self.height)))
#        svgGen.setViewBox(QRect(0, 0, self.scX(self.width), self.scY(self.height)))
#        svgGen.setTitle("SVG Generator Example Drawing")
#        svgGen.setDescription("An SVG drawing created by the SVG Generator Example provided with Qt.")
#        painter = QPainter( svgGen )
#        self.scene[graph_name].render( painter );
#        del painter

    @Slot()
    def exportClick(self):
        if not self.initialized:
            return
        self.parent.exportGraphs()

    @Slot()
    def reset_viewClick(self):
        if not self.initialized:
            return

        self.graphicsView.resetTransform()

    @Slot()
    def closeClick(self):
        self.close()

    @Slot()
    def zoomInClick(self):
        if not self.initialized:
            return

        scaleFactor = 1.1
        self.graphicsView.scale(scaleFactor, scaleFactor)

    @Slot()
    def zoomOutClick(self):
        if not self.initialized:
            return

        scaleFactor = 1.0/1.1
        self.graphicsView.scale(scaleFactor, scaleFactor)

    def updateState(self, components_state):
        if not self.initialized:
            return

        changed = False
        if self.components_state == None:
            changed = True
        else:
            for comp_name in self.components_state:
                if self.components_state[comp_name] != components_state[comp_name]:
                    changed = True
                    break

        if changed:
            self.components_state = components_state
            for graph_name in self.nodes:
                for comp_name in self.nodes[graph_name]:
                    if comp_name in self.components_state:
                        self.nodes[graph_name][comp_name].setBrush(getComponentBrush(self.components_state[comp_name]))

#    def wheelEvent(self, event):
#        print dir(event)
#        print event.delta()
#        ui->graphicsView->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
#        // Scale the view / do the zoom
#        double scaleFactor = 1.15;
#        if(event->delta() > 0) {
#            // Zoom in
#            ui->graphicsView-> scale(scaleFactor, scaleFactor);
# 
#        } else {
#            // Zooming out
#             ui->graphicsView->scale(1.0 / scaleFactor, 1.0 / scaleFactor);
#        }
#        //ui->graphicsView->setTransform(QTransform(h11, h12, h21, h22, 0, 0));

