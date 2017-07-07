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

import subsystem_graph
import subsystem_components
import subsystem_state_history

class SubsystemWidget(QWidget):
    """
    main class inherits from the ui window class.

    TODO: description.
    """

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

    @Slot()
    def on_click_showComponents(self):
        self.dialogComponents.show()

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

        self.dialogGraph = subsystem_graph.GraphDialog(self.subsystem_name, self)
        self.showGraph.clicked.connect(self.on_click)

        self.dialogHistory = subsystem_state_history.StateHistoryDialog(self.subsystem_name, self)
        self.showHistory.clicked.connect(self.on_click_showHistory)

        self.dialogComponents = subsystem_components.ComponentsDialog(self.subsystem_name, self)
        self.showComponents.clicked.connect(self.on_click_showComponents)

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

        time_tf = mcd[0].getElementsByTagName("t_tf")
        ret_time_tf = None
        if len(time_tf) == 1:
            ret_time_tf = time_tf[0].childNodes[0].data

        time_int1 = mcd[0].getElementsByTagName("int1")
        ret_time_int1 = None
        if len(time_int1) == 1:
            ret_time_int1 = time_int1[0].childNodes[0].data

        time_int2 = mcd[0].getElementsByTagName("int2")
        ret_time_int2 = None
        if len(time_int2) == 1:
            ret_time_int2 = time_int2[0].childNodes[0].data

        time_int3 = mcd[0].getElementsByTagName("int3")
        ret_time_int3 = None
        if len(time_int3) == 1:
            ret_time_int3 = time_int3[0].childNodes[0].data

        time_int4 = mcd[0].getElementsByTagName("int4")
        ret_time_int4 = None
        if len(time_int4) == 1:
            ret_time_int4 = time_int4[0].childNodes[0].data

        time_int5 = mcd[0].getElementsByTagName("int5")
        ret_time_int5 = None
        if len(time_int5) == 1:
            ret_time_int5 = time_int5[0].childNodes[0].data

        return (ss_history, curr_pred, ret_period, float(ret_time_int1), float(ret_time_int2), float(ret_time_int3), float(ret_time_int4), float(ret_time_int5))

    def getConnectionsSet(self, name, hide_converters=True):
            behavior = None
            for b in self.subsystem_info.behaviors:
                if b.name == name:
                    behavior = b
                    break

            if hide_converters:
                conv_comp = set()
                for comp in self.subsystem_info.components:
                    if comp.is_converter:
                        conv_comp.add(comp.name)

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

            if hide_converters:
                conv_connections = {}

            for c in self.subsystem_info.connections:
                if ((not c.component_from.strip()) or (not c.component_to.strip())) and not c.unconnected:
                    continue
                if behavior:
                    if (not c.component_from in current_running) and (not c.component_to in current_running):
                        continue
                    if c.component_from in other_behaviors_comp or c.component_to in other_behaviors_comp:
                        continue
#                elif name == "<always running>":
#                    if (not c.component_from in always_running) or (not c.component_to in always_running):
#                        continue
                elif name == "<all>":
                    pass
                else:
                    raise Exception('getConnectionsSet', 'wrong behavior name: ' + name)

                c_from = c.component_from
                c_to = c.component_to
                unconnected = c.unconnected
                c_name = c.name
                if hide_converters:
                    if c_from in conv_comp:
                        if not c_from in conv_connections:
                            conv_connections[c_from] = (None, c_to)
                            continue
                        else:
                            conv_connections[c_from] = (conv_connections[c_from][0], c_to)
                            c_to = conv_connections[c_from][1]
                            c_from = conv_connections[c_from][0]
                            unconnected = False
                    elif c_to in conv_comp:
                        if not c_to in conv_connections:
                            conv_connections[c_to] = (c_from, None)
                            continue
                        else:
                            conv_connections[c_to] = (c_from, conv_connections[c_to][1])
                            c_from = conv_connections[c_to][0]
                            c_to = conv_connections[c_to][1]
                            unconnected = False

                if not unconnected:
                    conn_tuple = (c_from, c_to)
                else:
                    if not c_from.strip():
                        conn_tuple = (None, c_to)
                    else:
                        conn_tuple = (c_from, None)

                if c_name:
                    cname = c_name
                elif c.port_from.strip():
                    cname = c.port_from
                    if not unconnected and cname.endswith("_OUTPORT"):
                        cname = cname[:-8]
                else:
                    cname = c.port_to
                    if not unconnected and cname.endswith("_INPORT"):
                        cname = cname[:-7]

                latex_name = c.latex
                if not latex_name:
                    #latex_name = '\\text{' + cname + '}'
                    latex_name = None
#                if not cname:
#                    continue
                if conn_tuple in conn_set:
                    #conn_set[conn_tuple] = conn_set[conn_tuple] + "\\n" + cname
                    conn_set[conn_tuple][0].append(cname)
                    conn_set[conn_tuple][1].append(latex_name)
                else:
                    conn_set[conn_tuple] = [[cname], [latex_name]]

            return conn_set

    def getComponentNameFromPath(self, path):
        idx = path.find('.')
        if idx < 0:
            return None
        return path[:idx]

    def exportGraph(self, graph_name):
        dot, eps_file_list, latex_formulas = self.generateGraph(graph_name, True)

        in_read, in_write = os.pipe()
        os.write(in_write, "\\documentclass{minimal}\n")
        os.write(in_write, "\\usepackage{amsmath}\n")
        os.write(in_write, "\\usepackage{mathtools}\n")
        os.write(in_write, "\\begin{document}\n")

        new_page = False
        for f in latex_formulas:
            if new_page:
                os.write(in_write, "\\clearpage\n")
            new_page = True
            os.write(in_write, "\\begin{gather*}" + f + "\\end{gather*}\n")

        os.write(in_write, "\\end{document}\n")
        os.close(in_write)

        # generate dvi file from latex document
        subprocess.call(['latex', '-output-directory=/tmp'], stdin=in_read)

        page_num = 1
        for (handle, path) in eps_file_list:
            subprocess.call(['dvips', '/tmp/texput.dvi', '-pp', str(page_num), '-o', '/tmp/texput.ps'])
            subprocess.call(['ps2eps', '/tmp/texput.ps', '-f'])
            with open('/tmp/texput.eps', 'r') as infile:
                data = infile.read()
            with open(path, 'w') as outfile:
                outfile.write(data)
            page_num += 1

        # generate eps
        #print dot
        in_read, in_write = os.pipe()
        os.write(in_write, dot)
        os.close(in_write)
        subprocess.call(['dot', '-Teps', '-o'+graph_name+'.eps'], stdin=in_read)

        for (handle, file_name) in eps_file_list:
            os.close(handle)
            os.remove(file_name)
        os.remove('/tmp/texput.dvi')
        os.remove('/tmp/texput.ps')
        os.remove('/tmp/texput.eps')

        subprocess.call(['epspdf', graph_name+'.eps', graph_name+'.pdf'], stdin=in_read)
        os.remove(graph_name+'.eps')


    def exportGraphs(self):
        graphs_list = ["<all>"]#, "<always running>"]
        for behavior in self.subsystem_info.behaviors:
            graphs_list.append(behavior.name)

        for graph_name in graphs_list:
            self.exportGraph(graph_name)

    def generateGraph(self, graph_name, use_latex):
            eps_file_list = []
            latex_formulas = []

            draw_unconnected = False

            conn_set = self.getConnectionsSet(graph_name, hide_converters=False)
            dot = "digraph " + self.subsystem_name + " {\n"

            new_page = False
            shown_components = set()
            for c in conn_set:
                if not draw_unconnected and (not c[0] or not c[1]):
                    continue
                if c[0] == c[1]:
                    continue
                conn = conn_set[c]
                if use_latex:
                    conn_latex = ''
                    sep = ''
                    for i in range(len(conn[1])):
                        latex = conn[1][i]
                        if not latex:
                            latex = '\\text{' + conn[0][i].replace('_', '\_') + '}'
                        conn_latex += sep + latex
                        sep = ' \\\\ '

                    if conn_latex in latex_formulas:
                        handle, path = eps_file_list[latex_formulas.index(conn_latex)]
                    else:
                        handle, path = tempfile.mkstemp(suffix=".eps")
                        eps_file_list.append( (handle, path) )
                        latex_formulas.append( conn_latex )
                else:
                    conn_str = ''
                    sep = ''
                    for cname in conn[0]:
                        conn_str += sep + cname
                        sep = '\\n'

                if c[0] == None:
                    shown_components.add(c[1])
#                    if draw_unconnected:
                    if use_latex:
                        dot += "\"" + c[1] + "_unconnected_in\" [shape=point label=\"\"];\n"
                        dot += c[1] + "_unconnected_in -> " + c[1] + " [label=<<TABLE BORDER=\"0\"><TR><TD><IMG src=\"" + path + "\"/></TD></TR></TABLE>>];\n"
                    else:
                        dot += "\"" + c[1] + "_unconnected_in\" [shape=point label=\"\"];\n"
                        dot += c[1] + "_unconnected_in -> " + c[1] + " [label=\"" + conn_str + "\"];\n"
                elif c[1] == None:
                    shown_components.add(c[0])
#                    if draw_unconnected:
                    if use_latex:
                        dot += "\"" + c[0] + "_unconnected_out\" [shape=point label=\"\"];\n"
                        dot += c[0] + " -> " + c[0] + "_unconnected_out [label=<<TABLE BORDER=\"0\"><TR><TD><IMG src=\"" + path + "\"/></TD></TR></TABLE>>];\n"
                    else:
                        dot += "\"" + c[0] + "_unconnected_out\" [shape=point label=\"\"];\n"
                        dot += c[0] + " -> " + c[0] + "_unconnected_out [label=\"" + conn_str + "\"];\n"
                else:
                    # ignore loops (port conversions)
                    shown_components.add(c[0])
                    shown_components.add(c[1])
                    if c[0] != c[1]:
                        if use_latex:
                            dot += c[0] + " -> " + c[1] + " [label=<<TABLE BORDER=\"0\"><TR><TD><IMG src=\"" + path + "\"/></TD></TR></TABLE>>];\n"
                        else:
                            dot += c[0] + " -> " + c[1] + " [label=\"" + conn_str + "\"];\n"

            if use_latex:
                for c in self.subsystem_info.components:
                    if not c.name in shown_components:
                        continue
                    if not c.latex:
                        continue

                    if c.latex in latex_formulas:
                        handle, path = eps_file_list[latex_formulas.index(c.latex)]
                    else:
                        handle, path = tempfile.mkstemp(suffix=".eps")
                        eps_file_list.append( (handle, path) )
                        latex_formulas.append( c.latex )
                    dot += c.name + " [label=\"\"; image=\"" + path + "\"];\n"

            dot     += "}\n"

            print "latex_formulas", latex_formulas
            return dot, eps_file_list, latex_formulas

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

            graphs_list = ["<all>"]#, "<always running>"]
            for behavior in self.subsystem_info.behaviors:
                graphs_list.append(behavior.name)

            for graph_name in graphs_list:
                gr = self.generateGraph(graph_name, False)
                dot = gr[0]
                in_read, in_write = os.pipe()
                os.write(in_write, dot)
                os.close(in_write)

                out_read, out_write = os.pipe()
                subprocess.call(['dot', '-Tplain'], stdin=in_read, stdout=out_write)
                graph_str = os.read(out_read, 1000000)
                os.close(out_read)
                graph_str = graph_str.replace("\\\n", "")

                self.dialogGraph.addGraph(graph_name, graph_str)

            self.dialogGraph.showGraph("<all>")
            self.graph = True

        components_state = {}
        for value in msg.status[0].values:
            components_state[value.key] = value.value

        components_diag_msgs = {}
        for value in msg.status[1].values:
            components_diag_msgs[value.key] = value.value

        #
        # update dialogs
        #
        self.dialogComponents.updateState(components_state, components_diag_msgs)
        self.dialogGraph.updateState(components_state)

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
            self.PeriodWall.setText(mcd[2] + ', ' + str(mcd[3]*1000.0) + 'ms, ' + str(mcd[4]*1000.0) + 'ms, ' + str(mcd[5]*1000.0) + 'ms, ' + str(mcd[6]*1000.0) + 'ms, ' + str(mcd[7]*1000.0) + 'ms')
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
            for index in range(len(subsystem.subsystem_info.lower_outputs)):
                lo_out = subsystem.subsystem_info.lower_outputs[index]
                if up_in == lo_out:
                    if common_buffers == None:
                        common_buffers = []
                    common_buffers.append(up_in)

        for this_index in range(len(self.subsystem_info.upper_outputs)):
            up_out = self.subsystem_info.upper_outputs[this_index]
            for index in range(len(subsystem.subsystem_info.lower_inputs)):
                lo_in = subsystem.subsystem_info.lower_inputs[index]
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


