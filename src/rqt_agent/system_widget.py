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

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QHBoxLayout, QScrollArea
import roslib
import rospkg
import rospy
from rospy.exceptions import ROSException

from .topic_info import TopicInfo
from .subsystem_widget import SubsystemWidget

class SystemWidget(QWidget):
    """
    main class inherits from the ui window class.

    You can specify the topics that the topic pane.

    SystemWidget.start must be called in order to update topic pane.
    """

    _column_names = ['topic', 'type', 'bandwidth', 'rate', 'value']

    def __init__(self, plugin=None):
        """
        """
        super(SystemWidget, self).__init__()

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_agent'), 'resource', 'SystemWidget.ui')
        loadUi(ui_file, self)

        self._plugin = plugin

        self._subsystems = {}

        self.all_subsystems = {}
        self._widgets = {}
        self.prev_subsystems = []
        self.levels_layouts = []

        self.structure_root = None
        self.structure_graph = None

        self.structure_changed = False

        self.scrollArea = QScrollArea()
        self.scrollArea.setWidgetResizable( True );
        self.horizontalLayout.addWidget(self.scrollArea)
        self.mainWidget = QWidget()
        self.scrollArea.setWidget(self.mainWidget)
        self.verticalLayout = QVBoxLayout()
        self.mainWidget.setLayout(self.verticalLayout)

#        self._tree_items = {}
#        self._column_index = {}
#        for column_name in self._column_names:
#            self._column_index[column_name] = len(self._column_index)

        # self.refresh_topics()

        # init and start update timer
        self._timer_refresh_topics = QTimer(self)
        self._timer_refresh_topics.timeout.connect(self.refresh_topics)

    def checkStructureChange(self):
        result = False

        for subsystem_name in self.prev_subsystems:
            if not subsystem_name in self._widgets:
                result = True
                break;

        if result == False:
            for subsystem_name in self._widgets:
                if not subsystem_name in self.prev_subsystems:
                    result = True
                    break

        self.prev_subsystems = []
        for subsystem_name in self._widgets:
            self.prev_subsystems.append(subsystem_name)

        return result

    def start(self):
        """
        This method needs to be called to start updating topic pane.
        """
        self._timer_refresh_topics.start(100)

    def generateStructure(self):
        names = []
        for w1_name in self._widgets:
            names.append(w1_name)
            self._widgets[w1_name].resetBuffersLayout()

        for w1_name in self._widgets:
            w1 = self._widgets[w1_name]
            if not w1.isInitialized():
                continue
            for w2_name in self._widgets:
                w2 = self._widgets[w2_name]
                if not w2.isInitialized():
                    continue
                common_buffers = w1.getCommonBuffers(w2)
                if common_buffers != None:
                    w1.groupBuffers(common_buffers, w2.subsystem_name)
                    w2.groupBuffers(common_buffers, w1.subsystem_name)

        parents_dict = {}
        for w1_name in self._widgets:
            w1 = self._widgets[w1_name]
            if not w1.isInitialized():
                continue
            for w2_name in self._widgets:
                w2 = self._widgets[w2_name]
                if not w2.isInitialized():
                    continue
                rel_pose = w1.getLowerSubsystemPosition(w2.subsystem_name)
                if rel_pose != -1:
                    parents_dict[w2_name] = w1_name

#        print parents_dict

        # get top-most subsystem (root)
        root = None
        if parents_dict:
            root = list(parents_dict.keys())[0]
            while root in parents_dict:
                root = parents_dict[root]

        levels = []
        if root == None:
            # there are no levels
            one_level = []
            for w_name in self._widgets:
                one_level.append(w_name)
            if len(one_level) > 0:
                levels.append(one_level)
        else:
            levels.append([root])
            while True:
                # expand all subsystems in the lowest level
                current_lowest = levels[-1]
                next_lower_level = []
                for s in current_lowest:
                    lower_list = self._widgets[s].getLowerSubsystems()
                    next_lower_level = next_lower_level + lower_list
                if len(next_lower_level) == 0:
                    break
                else:
                    levels.append(next_lower_level)

            # TODO: manage disjoint trees
            added_subsystems = []
            for l in levels:
                for s in l:
                    added_subsystems.append(s)
            for w_name in self._widgets:
                if not w_name in added_subsystems:
                    print "WARNING: subsystem %s is not in the main tree. This is not implemented."%(w_name)

        print "levels:", levels
        return levels

    def layout_widgets(self, layout):
       return (layout.itemAt(i) for i in range(layout.count()))

    @Slot()
    def refresh_topics(self):
        """
        refresh tree view items
        """
        #
        # update the list of subsystems
        #
        topic_list = rospy.get_published_topics()
        if topic_list is None:
            rospy.logerr('Not even a single published topic found. Check network configuration')
            return

        # start new topic dict
        new_subsystems = {}

        for topic_name, topic_type in topic_list:
            name_split = topic_name.split('/')
#            print name_split

            if (len(name_split) == 3) and (name_split[0] == '') and (name_split[2] == 'diag') and (topic_type == "diagnostic_msgs/DiagnosticArray"):
                subsystem_name = name_split[1]
                # if topic is new
                if subsystem_name not in self._subsystems:
                    # create new TopicInfo
                    topic_info = TopicInfo(topic_name, topic_type)
                    new_subsystems[subsystem_name] = topic_info
                    topic_info.start_monitoring()
                else:
                    # if topic has been seen before, copy it to new dict and
                    # remove it from the old one
                    new_subsystems[subsystem_name] = self._subsystems[subsystem_name]
                    del self._subsystems[subsystem_name]

        # remove unused subsystems
        while True:
            repeat = False
            for s in self._subsystems:
                if not s in new_subsystems:
                    del self._subsystems[s]
                    repeat = True
                    break
            if not repeat:
                break

        # switch to new topic dict
        self._subsystems = new_subsystems

#        print self._subsystems.keys()

        #
        # update each subsystem
        #
        new_widgets = {}
        for subsystem_name in self._subsystems:
            msg = self._subsystems[subsystem_name].last_message

            if (msg != None) and (len(msg.status) == 2) and \
              msg.status[0].name == 'components' and msg.status[1].name == 'diagnostics':
                name_split = subsystem_name.split('/')

                if not subsystem_name in self.all_subsystems:
                    self.all_subsystems[subsystem_name] = SubsystemWidget(self._plugin, subsystem_name)

                if not subsystem_name in self._widgets:
                    new_widgets[subsystem_name] = self.all_subsystems[subsystem_name]
#                    self.verticalLayout.addWidget(new_widgets[subsystem_name])
                else:
                    new_widgets[subsystem_name] = self._widgets[subsystem_name]
#                    del self._widgets[subsystem_name]

#                for value in msg.status[1].values:
#                    if value.key == 'master_component':
#                        new_widgets[subsystem_name].setStateName(value.value, '')
#                        break

        # remove unused subsystems
#        while True:
#            repeat = False
#            for s in self._widgets:
#                if not s in new_widgets:
#                    del self._widgets[s]
#                    repeat = True
#                    break
#            if not repeat:
#                break

        self._widgets = new_widgets

#        print self._widgets.keys()

        structure_changed = self.checkStructureChange()

        if structure_changed:
            self.structure_changed = True

        if self.structure_changed:
            allInitialized = True
            for subsystem_name in self._widgets:
                if not self._widgets[subsystem_name].isInitialized():
                    allInitialized = False
                    break
            if allInitialized:
                # remove all widgets from layouts
                # and remove all layouts
                for i in reversed(range(len(self.levels_layouts))):
                    layout = self.levels_layouts[i]
                    for i in reversed(range(layout.count())):
                        # The new widget is deleted when its parent is deleted.
                        layout.itemAt(i).widget().setParent(None)
                    self.verticalLayout.removeItem(layout)
                    del layout

                self.levels_layouts = []

                levels = self.generateStructure()
                for l in levels:
                    hbox = QHBoxLayout()
                    for w in l:
                        hbox.addWidget(self._widgets[w])
                        self._widgets[w].show()
                    self.levels_layouts.append(hbox)
                    self.verticalLayout.addLayout(hbox)
#                for 
                # TODO
                self.structure_changed = False

        while True:
            repeat = False
            for s in self.all_subsystems:
                if not s in self._widgets:
                    del self.all_subsystems[s]
                    repeat = True
                    break
            if not repeat:
                break

        for subsystem_name in self._widgets:
            self._widgets[subsystem_name].update_subsystem(self._subsystems[subsystem_name].last_message)

    def shutdown_plugin(self):
        for topic in self._topics.values():
            topic['info'].stop_monitoring()
        self._timer_refresh_topics.stop()

    def set_selected_topics(self, selected_topics):
        """
        @param selected_topics: list of tuple. [(topic_name, topic_type)]
        @type selected_topics: []
        """
        rospy.logdebug('set_selected_topics topics={}'.format(
                                                         len(selected_topics)))
        self._selected_topics = selected_topics

    # TODO(Enhancement) Save/Restore tree expansion state
    def save_settings(self, plugin_settings, instance_settings):
        header_state = self.topics_tree_widget.header().saveState()
        instance_settings.set_value('tree_widget_header_state', header_state)

    def restore_settings(self, pluggin_settings, instance_settings):
        if instance_settings.contains('tree_widget_header_state'):
            header_state = instance_settings.value('tree_widget_header_state')
            if not self.topics_tree_widget.header().restoreState(header_state):
                rospy.logwarn("rqt_topic: Failed to restore header state.")

