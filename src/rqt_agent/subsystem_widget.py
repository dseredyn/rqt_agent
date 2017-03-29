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

class MyDialog(QDialog):

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
        super(MyDialog, self).__init__(parent)

        self.parent = parent

        self.setWindowFlags(Qt.Window)

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_agent'), 'resource', 'GraphVis.ui')
        loadUi(ui_file, self)

        self.setWindowTitle(subsystem_name + " - transition function")
        self.components_state = None
        self.initialized = False

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


class StateHistoryDialog(QDialog):

    @Slot()
    def closeClick(self):
        self.close()

    class MyStyle(QCommonStyle):
        def drawControl (self, element, opt, painter, widget=None):

#            print element
            if element == QStyle.CE_HeaderLabel:
                hv = widget
#                if not hv or hv.orientation() != Qt.Horizontal:
#                    return super(StateHistoryDialog.MyStyle, self).drawControl(element, opt, p, widget)
                header = opt

                if header.section < 3:
                    return super(StateHistoryDialog.MyStyle, self).drawControl(element, opt, painter, widget)

                painter.save()
                #// painter->translate(header->rect.topLeft())
                rect = header.rect.bottomLeft()
                painter.translate(QPoint(rect.x() + 10, rect.y() + 5))
#                print "drawControl"
                painter.rotate(-90)
                painter.drawText(0,0,header.text)
                painter.restore()
                return
#            return QProxyStyle::drawControl(element, option, painter, widget);
#            print "drawControl"
            return super(StateHistoryDialog.MyStyle, self).drawControl(element, opt, painter, widget)

#        def drawItemText( painter, rectangle, alignment, palette, enabled, text, textRole = QPalette.NoRole):
#            pass

    class MyHorizHeader(QHeaderView):
        def __init__(self, parent=None):
            super(StateHistoryDialog.MyHorizHeader, self).__init__(Qt.Horizontal, parent)
            style = StateHistoryDialog.MyStyle()
            self.setStyle(style)

        def sizeHint(self):
            # Get the base implementation size.
            baseSize = super(StateHistoryDialog.MyHorizHeader, self).sizeHint()
            # Override the height with a custom value.
            baseSize.setHeight( 150 );
#            baseSize.setWidth( 20 );
            return baseSize;

    def __init__(self, subsystem_name, parent=None):
        super(StateHistoryDialog, self).__init__(parent)

        self.parent = parent

        self.setWindowFlags(Qt.Window)

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_agent'), 'resource', 'StateHistory.ui')
        loadUi(ui_file, self)

        self.setWindowTitle(subsystem_name + " - state history")

        self.pushButton_close.clicked.connect(self.closeClick)

        hdr = self.MyHorizHeader(self.tableWidget)
        hdr.setMinimumSectionSize(40)
        hdr.setDefaultSectionSize(40)
        self.tableWidget.setHorizontalHeader(hdr)

        self.initialized = False
        item0 = QTableWidgetItem("behavior")
        item1 = QTableWidgetItem("reason")
        item2 = QTableWidgetItem("time")
        self.tableWidget.setHorizontalHeaderItem (0, item0)
        self.tableWidget.setHorizontalHeaderItem (1, item1)
        self.tableWidget.setHorizontalHeaderItem (2, item2)
        self.tableWidget.setColumnWidth(0, 100)
        self.tableWidget.setColumnWidth(1, 50)
        self.tableWidget.setColumnWidth(2, 75)

    def updateState(self, mcd):
        hist = mcd[0]
        if self.tableWidget.rowCount() != len(hist):
            self.tableWidget.setRowCount( len(hist) )
        row = 0
        curr_pred = mcd[1].split(",")
        curr_pred_v = []
        for cp in curr_pred:
            k_v = cp.split(":")
            if len(k_v) == 2:
                curr_pred_v.append(k_v[1])

        for ss in hist:
            for col in range(3):
                self.tableWidget.setItem(row, col, QTableWidgetItem(ss[col]))
            pred = ss[3].split(",")

            if not self.initialized:
                self.initialized = True
                predicates = []
                for p in pred:
                    k_v = p.split(":")
                    if len(k_v) != 2:
                        continue
                    predicates.append( k_v[0] )

                self.tableWidget.setColumnCount(len(predicates))
                idx = 3
                for p in predicates:
                    item = QTableWidgetItem(p)
                    self.tableWidget.setHorizontalHeaderItem(idx, item)
                    idx += 1

            idx = 0
            for p in pred:
                k_v = p.split(":")
                if len(k_v) != 2:
                    continue
                if row == 0:
                    self.tableWidget.setItem(row, 3+idx, QTableWidgetItem(k_v[1] + " ("+curr_pred_v[idx]+")"))
                else:
                    self.tableWidget.setItem(row, 3+idx, QTableWidgetItem(k_v[1]))
                idx += 1
                
            row = row + 1

class SubsystemWidget(QWidget):
    """
    main class inherits from the ui window class.

    You can specify the topics that the topic pane.

    SubsystemWidget.start must be called in order to update topic pane.
    """

    SELECT_BY_NAME = 0
    SELECT_BY_MSGTYPE = 1

    _column_names = ['topic', 'type', 'bandwidth', 'rate', 'value']

# ___________________________________________________________
# |[ipc_buffers]                                             |
# |                     subsystem_name                       |
# |  master_component                                        |
# |  subsystem_state       [components]                      |
# |                        [components]                      |
# |                        [components]                      |
# |                        [components]                      |
# |                        [components]                      |
# |                                                          |
# |                                                          |
# |                                                          |
# |                                                          |
# |[ipc_buffers]                                             |
# |__________________________________________________________|

    def layout_widgets(self, layout):
       return (layout.itemAt(i) for i in range(layout.count()))

    def resetBuffersLayout(self):
        self.buffer_groups = {}
        self.lower_subsystems = []

        print self.subsystem_name, ".resetBuffersLayout()"

        for buf in self.all_buffers:
            self.all_buffers[buf].hide()

        while True:
            w = self.lower_buffers_layout.takeAt(0)
            if w == None:
                break;
            del w

        while True:
            w = self.upper_buffers_layout.takeAt(0)
            if w == None:
                break;
            del w

    @Slot()
    def on_click(self):
        #self.dialogGraph.exec_()
        self.dialogGraph.show()

    @Slot()
    def on_click_showHistory(self):
        #self.dialogGraph.exec_()
        self.dialogHistory.show()

    def __init__(self, plugin=None, name=None):
        """
        @type selected_topics: list of tuples.
        @param selected_topics: [($NAME_TOPIC$, $TYPE_TOPIC$), ...]
        @type select_topic_type: int
        @param select_topic_type: Can specify either the name of topics or by
                                  the type of topic, to filter the topics to
                                  show. If 'select_topic_type' argument is
                                  None, this arg shouldn't be meaningful.
        """
        super(SubsystemWidget, self).__init__()

        self.initialized = False

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_agent'), 'resource', 'SubsystemWidget.ui')
        loadUi(ui_file, self)
        self._plugin = plugin

        print "created Subsystem widget"

        if name != None:
            self.SubsystemName.setText(name)
        else:
            self.SubsystemName.setText("<could not get subsystem name>")

        self.subsystem_name = name

        self.subsystem_info = None
        self.state = ''
        self.behavior = ''

        self.all_buffers = {}
        self.resetBuffersLayout()

        self.components = {}

        self.graph = None

        self.dialogGraph = MyDialog(self.subsystem_name, self)
        self.showGraph.clicked.connect(self.on_click)

        self.dialogHistory = StateHistoryDialog(self.subsystem_name, self)
        self.showHistory.clicked.connect(self.on_click_showHistory)

    def isInitialized(self):
        return self.initialized

    def extractConnectionInfo(self, conn, comp_from, comp_to):
        if (not comp_to) or (not comp_from):
            print 'WARNING: wrong edge(1): ', conn, comp_from, comp_to
            return None

        if conn.find(comp_from.name) != 0:
            print 'WARNING: wrong edge(2): ', conn, comp_from.name, comp_to.name
            return None

        idx = len(comp_from.name)
        port_from = None
        for p in comp_from.ports:
            if conn.find(p.name, idx) == idx:
                port_from = p.name
                idx += len(p.name)
                break

        if not port_from:
            print 'WARNING: wrong edge(3): ', conn, comp_from.name, comp_to.name
            return None

        if conn.find(comp_to.name, idx) != idx:
            print 'WARNING: wrong edge(4): ', conn, comp_from.name, comp_to.name

        idx += len(comp_to.name)

        port_to = None
        for p in comp_to.ports:
            if conn.find(p.name, idx) == idx:
                port_to = p.name
                idx += len(p.name)
                break

        if not port_to:
            print 'WARNING: wrong edge(5): ', conn, comp_from.name, comp_to.name
            return None

        return (comp_from.name, port_from, comp_to.name, port_to)

    def parseMasterComponentDiag(self, state):
        ss_history = []
        ret_period = "?"

        dom = minidom.parseString(state)
        mcd = dom.getElementsByTagName("mcd")
        if len(mcd) != 1:
            return (ss_history, ret_period)

        hist = mcd[0].getElementsByTagName("h")
        if len(hist) == 1:
            ss_list = hist[0].getElementsByTagName("ss")
            for ss in ss_list:
#                print "ss", ss
#                print dir(ss.getAttribute("n")) # string
                ss_history.append( (ss.getAttribute("n"), ss.getAttribute("r"), ss.getAttribute("t"), ss.getAttribute("e")) )

        current_predicates = mcd[0].getElementsByTagName("pr")
        curr_pred = None
        if len(current_predicates) == 1:
            curr_pred = current_predicates[0].getAttribute("v")

        period = mcd[0].getElementsByTagName("p")
        ret_period = None
        if len(period) == 1:
            ret_period = period[0].childNodes[0].data

        return (ss_history, curr_pred, ret_period)

    def getConnectionsSet(self, name):
            behavior = None
            for b in self.subsystem_info.behaviors:
                if b.name == name:
                    behavior = b
                    break
            
            sw_comp = set()
            for b in self.subsystem_info.behaviors:
                for r in b.running_components:
                    sw_comp.add(r)

            all_comp = set()
            for comp in self.subsystem_info.components:
                all_comp.add(comp.name)

            always_running = all_comp - sw_comp

            conn_set = {}
            current_running = set()
            if behavior:
                for r in behavior.running_components:
                    current_running.add(r)
            other_behaviors_comp = sw_comp - current_running
            running = always_running.union(current_running)

            for c in self.subsystem_info.connections:
                if ((not c.component_from.strip()) or (not c.component_to.strip())) and not c.unconnected:
                    continue
                if behavior:
                    if (not c.component_from in current_running) and (not c.component_to in current_running):
                        continue
                    if c.component_from in other_behaviors_comp or c.component_to in other_behaviors_comp:
                        continue
                elif name == "<always running>":
                    if (not c.component_from in always_running) or (not c.component_to in always_running):
                        continue
                elif name == "<all>":
                    pass
                else:
                    raise Exception('getConnectionsSet', 'wrong behavior name: ' + name)

#                if not c.component_from.strip():
#                    conn_tuple = ("abcdefghijkl", c.component_to)
#                elif not c.component_to.strip():
#                    conn_tuple = (c.component_from, "abcdefghijkl")
#                else:

                if not c.unconnected:
                    conn_tuple = (c.component_from, c.component_to)
                else:
                    if not c.component_from.strip():
                        conn_tuple = (None, c.component_to)
                    else:
                        conn_tuple = (c.component_from, None)

                if c.name:
                    cname = c.name
                elif c.port_from.strip():
                    cname = c.port_from
                    if not c.unconnected and cname.endswith("_OUTPORT"):
                        cname = cname[:-8]
                else:
                    cname = c.port_to
                    if not c.unconnected and cname.endswith("_INPORT"):
                        cname = cname[:-7]

#                if not cname:
#                    continue
                if conn_tuple in conn_set:
                    conn_set[conn_tuple] = conn_set[conn_tuple] + "\\n" + cname
                else:
                    conn_set[conn_tuple] = cname
            return conn_set

    def update_subsystem(self, msg):
        for value in msg.status[1].values:
            if value.key == 'master_component':
                self.state = value.value
            elif value.key[-2:] == 'Rx' or value.key[-2:] == 'Tx':
                if value.key[0:-2] in self.all_buffers:
                    if value.value == '<data ok>':
                        self.all_buffers[value.key[:-2]].setStyleSheet("background-color: green")
                    else:
                        self.all_buffers[value.key[:-2]].setStyleSheet("background-color: red")
                    self.all_buffers[value.key[:-2]].setToolTip(value.value)

        if self.graph == None and self.initialized:

            draw_unconnected = False

            self.all_connections = []
            for conn in self.subsystem_info.connections:
                self.all_connections.append( (conn.component_from, conn.port_from, conn.component_to, conn.port_to) )

            graphs_list = ["<all>", "<always running>"]
            for behavior in self.subsystem_info.behaviors:
                graphs_list.append(behavior.name)

            for graph_name in graphs_list:
                conn_set = self.getConnectionsSet(graph_name)
                dot = "digraph " + self.subsystem_name + " {\n"
                for c in conn_set:
                    conn = conn_set[c]
                    if c[0] == None:
                        if draw_unconnected:
                            dot += "\"" + c[1] + "_unconnected_in\" [shape=point label=\"\"];\n"
                            dot += c[1] + "_unconnected_in -> " + c[1] + " [label=\"" + conn + "\"];\n"
                    elif c[1] == None:
                        if draw_unconnected:
                            dot += "\"" + c[0] + "_unconnected_out\" [shape=point label=\"\"];\n"
                            dot += c[0] + " -> " + c[0] + "_unconnected_out [label=\"" + conn + "\"];\n"
                    else:
                        # ignore loops (port conversions)
                        if c[0] != c[1]:
                            dot += c[0] + " -> " + c[1] + " [label=\"" + conn + "\"];\n"
                dot += "}\n"

#                print graph_name, dot
                in_read, in_write = os.pipe()
                os.write(in_write, dot)
                os.close(in_write)

                out_read, out_write = os.pipe()
                subprocess.call(['dot', '-Tplain'], stdin=in_read, stdout=out_write)
                graph_str = os.read(out_read, 1000000)
                os.close(out_read)
                graph_str = graph_str.replace("\\\n", "")
#                print graph_name, graph_str

                # generate pdf
                in_read, in_write = os.pipe()
                os.write(in_write, dot)
                os.close(in_write)
                subprocess.call(['dot', '-Tpdf', '-o'+graph_name+'.pdf'], stdin=in_read)

                self.dialogGraph.addGraph(graph_name, graph_str)
            self.dialogGraph.showGraph("<all>")
            self.graph = True

        # iterate through components and add them to QListWidget
        for value in msg.status[0].values:
            if not value.key in self.components:
                item = QListWidgetItem()
                item.setText(value.key)
                self.components[value.key] = item
                self.listWidget.addItem(item)

        components_state = {}
        for value in msg.status[0].values:
            self.components[value.key].setBackground( getComponentBrush(value.value) )
            self.components[value.key].setToolTip( getComponentTooltip(value.value) )
            components_state[value.key] = value.value

        #
        # update graph
        #
        self.dialogGraph.updateState(components_state)

        for value in msg.status[1].values:
            if value.key in self.components:
                self.components[value.key].setText(value.key + ' (' + value.value + ')')

        #rospy.wait_for_service('/' + name = '/getSubsystemInfo')
        if self.subsystem_info == None:
            try:
                self._getSubsystemInfo = rospy.ServiceProxy('/' + self.subsystem_name + '/getSubsystemInfo', GetSubsystemInfo)
                self.subsystem_info = self._getSubsystemInfo()
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

            if self.subsystem_info != None:
                print self.subsystem_info

            self.initialized = True

        mcd = self.parseMasterComponentDiag(self.state)
        if len(mcd[0]) > 0:
            self.SubsystemState.setText(mcd[0][0][0])
            self.dialogHistory.updateState(mcd)
            self.PeriodWall.setText(mcd[2])
        else:
            self.SubsystemState.setText("unknown")

#            self.SubsystemBehavior.setText(behavior_name.strip())

    def getCommonBuffers(self, subsystem):
        if not self.isInitialized() or not subsystem.isInitialized():
            return None
        if (subsystem.subsystem_info == None) or (self.subsystem_info == None):
            return None
        common_buffers = None
        for this_index in range(len(self.subsystem_info.upper_inputs)):
            up_in = self.subsystem_info.upper_inputs[this_index]
            if not self.subsystem_info.upper_inputs_ipc[this_index]:
                continue
            for index in range(len(subsystem.subsystem_info.lower_outputs)):
                lo_out = subsystem.subsystem_info.lower_outputs[index]
                if not subsystem.subsystem_info.lower_outputs_ipc[index]:
                    continue
                if up_in == lo_out:
                    if common_buffers == None:
                        common_buffers = []
                    common_buffers.append(up_in)

        for this_index in range(len(self.subsystem_info.upper_outputs)):
            up_out = self.subsystem_info.upper_outputs[this_index]
            if not self.subsystem_info.upper_outputs_ipc[this_index]:
                continue
            for index in range(len(subsystem.subsystem_info.lower_inputs)):
                lo_in = subsystem.subsystem_info.lower_inputs[index]
                if not subsystem.subsystem_info.lower_inputs_ipc[index]:
                    continue
                if up_out == lo_in:
                    if common_buffers == None:
                        common_buffers = []
                    common_buffers.append(up_out)
        return common_buffers

    def getCommonString(self, str_list):
        idx = 0
        while True:
            character = None
            for s in str_list:
                if idx >= len(s):
                    return s
                if character == None:
                    character = s[idx]
                elif character != s[idx]:
                    return s[:idx]
            idx = idx + 1
        return None     # this is never reached

    def groupBuffers(self, buffer_list, subsystem_name):
        if buffer_list == None or len(buffer_list) == 0:
            print "Error in %s.groupBuffers(%s, %s): buffers list is None or empty"%(self.subsystem_name, buffer_list, subsystem_name)
            return False
        if subsystem_name in self.buffer_groups:
            # TODO: remove old buffer widgets
            return False
        self.buffer_groups[subsystem_name] = buffer_list

        lo_in = []
        up_in = []
        lo_out = []
        up_out = []

        for buf_name in buffer_list:
            if buf_name in self.subsystem_info.lower_inputs:
                lo_in.append(buf_name)
            elif buf_name in self.subsystem_info.upper_inputs:
                up_in.append(buf_name)
            elif buf_name in self.subsystem_info.lower_outputs:
                lo_out.append(buf_name)
            elif buf_name in self.subsystem_info.upper_outputs:
                up_out.append(buf_name)

        # buffer group should be either in lower part or upper part
        if (len(lo_in) > 0 or len(lo_out) > 0) and (len(up_in) > 0 or len(up_out) > 0):
            print "Error in %s.groupBuffers(%s, %s): mixed upper and lower buffers"%(self.subsystem_name, buffer_list, subsystem_name)
            return False

        # get most common part of buffers' names
        name_list = []
        common_name = self.getCommonString(buffer_list)
        for idx in range(len(buffer_list)):
            name_list.append( buffer_list[idx][len(common_name):] )

        print common_name, name_list

        vbox = QVBoxLayout()

        hbox1 = QHBoxLayout()
        hbox1.addStretch()
        if not (common_name) in self.all_buffers:
            self.all_buffers[common_name] = QLabel(common_name)
        hbox1.addWidget(self.all_buffers[common_name])
        self.all_buffers[common_name].show()
        hbox1.addStretch()

        hbox2 = QHBoxLayout()
        hbox2.addSpacing(20)
        for buf_name in name_list:
#            hbox2.addStretch()
            if common_name+buf_name in lo_in or common_name+buf_name in up_out:
                suffix = ' /\\'
            else:
                suffix = ' \\/'

            if not (common_name+buf_name) in self.all_buffers:
                self.all_buffers[common_name+buf_name] = QPushButton(buf_name + suffix)
            hbox2.addWidget( self.all_buffers[common_name+buf_name] )
            self.all_buffers[common_name+buf_name].show()
#            hbox2.addWidget( QPushButton(buf_name + suffix) )
#        hbox2.addStretch()
        hbox2.addSpacing(20)


        if len(lo_in) > 0 or len(lo_out) > 0:
            vbox.addLayout(hbox1)
            vbox.addLayout(hbox2)
            self.lower_buffers_layout.addLayout(vbox)
            self.lower_subsystems.append(subsystem_name)
        else:
            vbox.addLayout(hbox2)
            vbox.addLayout(hbox1)
            self.upper_buffers_layout.addLayout(vbox)

    def getLowerSubsystemPosition(self, subsystem_name):
        for i in range(len(self.lower_subsystems)):
            if self.lower_subsystems[i] == subsystem_name:
                return i
        return -1

    def getLowerSubsystems(self):
        return self.lower_subsystems

    def start(self):
        """
        This method needs to be called to start updating topic pane.
        """

    def shutdown_plugin(self):
        for topic in self._topics.values():
            topic['info'].stop_monitoring()
        self._timer_refresh_topics.stop()


    # TODO(Enhancement) Save/Restore tree expansion state
    def save_settings(self, plugin_settings, instance_settings):
        header_state = self.topics_tree_widget.header().saveState()
        instance_settings.set_value('tree_widget_header_state', header_state)

    def restore_settings(self, pluggin_settings, instance_settings):
        if instance_settings.contains('tree_widget_header_state'):
            header_state = instance_settings.value('tree_widget_header_state')
            if not self.topics_tree_widget.header().restoreState(header_state):
                rospy.logwarn("rqt_topic: Failed to restore header state.")


