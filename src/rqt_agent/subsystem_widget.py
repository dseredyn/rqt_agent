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
from python_qt_binding.QtWidgets import QHeaderView, QMenu, QTreeWidgetItem, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel
import roslib
import rospkg
import rospy
from rospy.exceptions import ROSException

from rqt_topic.topic_info import TopicInfo

from subsystem_msgs.srv import *

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

    def resetBuffersLayout(self):
        self.buffer_groups = {}
        self.lower_subsystems = []

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

        self.buffer_info = None
        self.state = ''
        self.behavior = ''

        self.resetBuffersLayout()

#        self.topics_tree_widget.sortByColumn(0, Qt.AscendingOrder)
#        header = self.topics_tree_widget.header()
#        header.setResizeMode(QHeaderView.ResizeToContents)
#        header.customContextMenuRequested.connect(self.handle_header_view_customContextMenuRequested)
#        header.setContextMenuPolicy(Qt.CustomContextMenu)

        # Whether to get all topics or only the topics that are set in advance.
        # Can be also set by the setter method "set_selected_topics".
#        self._selected_topics = selected_topics

#        self._current_topic_list = []
#        self._topics = {}
#        self._tree_items = {}
#        self._column_index = {}
#        for column_name in self._column_names:
#            self._column_index[column_name] = len(self._column_index)

        # self.refresh_topics()

        # init and start update timer
#        self._timer_refresh_topics = QTimer(self)
#        self._timer_refresh_topics.timeout.connect(self.refresh_topics)

    def setStateName(self, state, behavior):
        self.state = state
        self.behavior = behavior

    def isInitialized(self):
        return self.initialized

    def update_subsystem(self):
        #rospy.wait_for_service('/' + name = '/getSubsystemInfo')
        if self.buffer_info == None:
            try:
                self._getSubsystemInfo = rospy.ServiceProxy('/' + self.subsystem_name + '/getSubsystemInfo', GetSubsystemInfo)
                self.buffer_info = self._getSubsystemInfo()
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

            if self.buffer_info != None:
                print self.buffer_info

            self.initialized = True

        beg_idx = self.state.find('state:')
        if beg_idx >= 0:
            end_idx = self.state.find(',', beg_idx)
            if end_idx >= 0:
                state_name = self.state[beg_idx+6:end_idx]
            else:
                state_name = self.state[beg_idx+6:]
            self.SubsystemState.setText(state_name.strip())

        beg_idx = self.state.find('behavior:')
        if beg_idx >= 0:
            end_idx = self.state.find(',', beg_idx)
            if end_idx >= 0:
                behavior_name = self.state[beg_idx+9:end_idx]
            else:
                behavior_name = self.state[beg_idx+9:]
            self.SubsystemBehavior.setText(behavior_name.strip())

    def getCommonBuffers(self, subsystem):
        if not self.isInitialized() or not subsystem.isInitialized():
            return None
        if (subsystem.buffer_info == None) or (self.buffer_info == None):
            return None
        common_buffers = None
        for this_index in range(len(self.buffer_info.upper_inputs)):
            up_in = self.buffer_info.upper_inputs[this_index]
            if not self.buffer_info.upper_inputs_ipc[this_index]:
                continue
            for index in range(len(subsystem.buffer_info.lower_outputs)):
                lo_out = subsystem.buffer_info.lower_outputs[index]
                if not subsystem.buffer_info.lower_outputs_ipc[index]:
                    continue
                if up_in == lo_out:
                    if common_buffers == None:
                        common_buffers = []
                    common_buffers.append(up_in)

        for this_index in range(len(self.buffer_info.upper_outputs)):
            up_out = self.buffer_info.upper_outputs[this_index]
            if not self.buffer_info.upper_outputs_ipc[this_index]:
                continue
            for index in range(len(subsystem.buffer_info.lower_inputs)):
                lo_in = subsystem.buffer_info.lower_inputs[index]
                if not subsystem.buffer_info.lower_inputs_ipc[index]:
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
            if buf_name in self.buffer_info.lower_inputs:
                lo_in.append(buf_name)
            elif buf_name in self.buffer_info.upper_inputs:
                up_in.append(buf_name)
            elif buf_name in self.buffer_info.lower_outputs:
                lo_out.append(buf_name)
            elif buf_name in self.buffer_info.upper_outputs:
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
        hbox1.addWidget(QLabel(common_name))
        hbox1.addStretch()

        hbox2 = QHBoxLayout()
        for buf_name in name_list:
            hbox2.addStretch()
            if common_name+buf_name in lo_in or common_name+buf_name in up_out:
                suffix = ' /\\'
            else:
                suffix = ' \\/'
            hbox2.addWidget(QPushButton(buf_name + suffix))
        hbox2.addStretch()

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

#    def getOtherSubsystemRelativePose(self, subsystem):
#        if (subsystem.buffer_info == None) or (self.buffer_info == None):
#            return None
#        for this_index in range(len(self.buffer_info.upper_inputs)):
#            up_in = self.buffer_info.upper_inputs[this_index]
#            if not self.buffer_info.upper_inputs_ipc[this_index]:
#                continue
#            for index in range(len(subsystem.buffer_info.lower_outputs)):
#                lo_out = subsystem.buffer_info.lower_outputs[index]
#                if not subsystem.buffer_info.lower_outputs_ipc[index]:
#                    continue
#                if up_in == lo_out:
#                    return ("above", None)
#        for this_index in range(len(self.buffer_info.lower_inputs)):
#            lo_in = self.buffer_info.lower_inputs[this_index]
#            if not self.buffer_info.lower_inputs_ipc[this_index]:
#                continue
#            for index in range(len(subsystem.buffer_info.upper_outputs)):
#                up_out = subsystem.buffer_info.upper_outputs[index]
#                if not subsystem.buffer_info.upper_outputs_ipc[index]:
#                    continue
#                if lo_in == up_out:
#                    return ("below", this_index)
#        return None

    def start(self):
        """
        This method needs to be called to start updating topic pane.
        """
#        self._timer_refresh_topics.start(1000)


#    def _update_topics_data(self):
#        for topic in self._topics.values():
#            topic_info = topic['info']
#            if topic_info.monitoring:
#                # update rate
#                rate, _, _, _ = topic_info.get_hz()
#                rate_text = '%1.2f' % rate if rate != None else 'unknown'

#                # update bandwidth
#                bytes_per_s, _, _, _ = topic_info.get_bw()
#                if bytes_per_s is None:
#                    bandwidth_text = 'unknown'
#                elif bytes_per_s < 1000:
#                    bandwidth_text = '%.2fB/s' % bytes_per_s
#                elif bytes_per_s < 1000000:
#                    bandwidth_text = '%.2fKB/s' % (bytes_per_s / 1000.)
#                else:
#                    bandwidth_text = '%.2fMB/s' % (bytes_per_s / 1000000.)

#                # update values
#                value_text = ''
#                self.update_value(topic_info._topic_name, topic_info.last_message)

#            else:
#                rate_text = ''
#                bandwidth_text = ''
#                value_text = 'not monitored' if topic_info.error is None else topic_info.error

#            self._tree_items[topic_info._topic_name].setText(self._column_index['rate'], rate_text)
#            self._tree_items[topic_info._topic_name].setText(self._column_index['bandwidth'], bandwidth_text)
#            self._tree_items[topic_info._topic_name].setText(self._column_index['value'], value_text)

    def update_value(self, topic_name, message):
        if hasattr(message, '__slots__') and hasattr(message, '_slot_types'):
            for slot_name in message.__slots__:
                self.update_value(topic_name + '/' + slot_name, getattr(message, slot_name))

        elif type(message) in (list, tuple) and (len(message) > 0) and hasattr(message[0], '__slots__'):

            for index, slot in enumerate(message):
                if topic_name + '[%d]' % index in self._tree_items:
                    self.update_value(topic_name + '[%d]' % index, slot)
                else:
                    base_type_str, _ = self._extract_array_info(self._tree_items[topic_name].text(self._column_index['type']))
                    self._recursive_create_widget_items(self._tree_items[topic_name], topic_name + '[%d]' % index, base_type_str, slot)
            # remove obsolete children
            if len(message) < self._tree_items[topic_name].childCount():
                for i in range(len(message), self._tree_items[topic_name].childCount()):
                    item_topic_name = topic_name + '[%d]' % i
                    self._recursive_delete_widget_items(self._tree_items[item_topic_name])
        else:
            if topic_name in self._tree_items:
                self._tree_items[topic_name].setText(self._column_index['value'], repr(message))

    def _extract_array_info(self, type_str):
        array_size = None
        if '[' in type_str and type_str[-1] == ']':
            type_str, array_size_str = type_str.split('[', 1)
            array_size_str = array_size_str[:-1]
            if len(array_size_str) > 0:
                array_size = int(array_size_str)
            else:
                array_size = 0

        return type_str, array_size

    def _recursive_create_widget_items(self, parent, topic_name, type_name, message):
        if parent is self.topics_tree_widget:
            # show full topic name with preceding namespace on toplevel item
            topic_text = topic_name
            item = TreeWidgetItem(self._toggle_monitoring, topic_name, parent)
        else:
            topic_text = topic_name.split('/')[-1]
            if '[' in topic_text:
                topic_text = topic_text[topic_text.index('['):]
            item = QTreeWidgetItem(parent)
        item.setText(self._column_index['topic'], topic_text)
        item.setText(self._column_index['type'], type_name)
        item.setData(0, Qt.UserRole, topic_name)
        self._tree_items[topic_name] = item
        if hasattr(message, '__slots__') and hasattr(message, '_slot_types'):
            for slot_name, type_name in zip(message.__slots__, message._slot_types):
                self._recursive_create_widget_items(item, topic_name + '/' + slot_name, type_name, getattr(message, slot_name))

        else:
            base_type_str, array_size = self._extract_array_info(type_name)
            try:
                base_instance = roslib.message.get_message_class(base_type_str)()
            except (ValueError, TypeError):
                base_instance = None
            if array_size is not None and hasattr(base_instance, '__slots__'):
                for index in range(array_size):
                    self._recursive_create_widget_items(item, topic_name + '[%d]' % index, base_type_str, base_instance)
        return item

    def _toggle_monitoring(self, topic_name):
        item = self._tree_items[topic_name]
        if item.checkState(0):
            self._topics[topic_name]['info'].start_monitoring()
        else:
            self._topics[topic_name]['info'].stop_monitoring()

    def _recursive_delete_widget_items(self, item):
        def _recursive_remove_items_from_tree(item):
            for index in reversed(range(item.childCount())):
                _recursive_remove_items_from_tree(item.child(index))
            topic_name = item.data(0, Qt.UserRole)
            del self._tree_items[topic_name]
        _recursive_remove_items_from_tree(item)
        item.parent().removeChild(item)

    @Slot('QPoint')
    def handle_header_view_customContextMenuRequested(self, pos):
        header = self.topics_tree_widget.header()

        # show context menu
        menu = QMenu(self)
        action_toggle_auto_resize = menu.addAction('Toggle Auto-Resize')
        action = menu.exec_(header.mapToGlobal(pos))

        # evaluate user action
        if action is action_toggle_auto_resize:
            if header.resizeMode(0) == QHeaderView.ResizeToContents:
                header.setResizeMode(QHeaderView.Interactive)
            else:
                header.setResizeMode(QHeaderView.ResizeToContents)

    @Slot('QPoint')
    def on_topics_tree_widget_customContextMenuRequested(self, pos):
        item = self.topics_tree_widget.itemAt(pos)
        if item is None:
            return

        # show context menu
        menu = QMenu(self)
        action_item_expand = menu.addAction(QIcon.fromTheme('zoom-in'), 'Expand All Children')
        action_item_collapse = menu.addAction(QIcon.fromTheme('zoom-out'), 'Collapse All Children')
        action = menu.exec_(self.topics_tree_widget.mapToGlobal(pos))

        # evaluate user action
        if action in (action_item_expand, action_item_collapse):
            expanded = (action is action_item_expand)

            def recursive_set_expanded(item):
                item.setExpanded(expanded)
                for index in range(item.childCount()):
                    recursive_set_expanded(item.child(index))
            recursive_set_expanded(item)

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


