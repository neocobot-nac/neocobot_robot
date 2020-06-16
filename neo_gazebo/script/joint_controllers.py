#!/usr/bin/env python

import rospy
import random

from python_qt_binding.QtCore import pyqtSlot
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import Signal
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import QApplication
from python_qt_binding.QtWidgets import QHBoxLayout
from python_qt_binding.QtWidgets import QLabel
from python_qt_binding.QtWidgets import QLineEdit
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QSlider
from python_qt_binding.QtWidgets import QVBoxLayout
from python_qt_binding.QtWidgets import QGridLayout
from python_qt_binding.QtWidgets import QScrollArea
from python_qt_binding.QtWidgets import QSpinBox
from python_qt_binding.QtWidgets import QWidget

import xml.dom.minidom
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from math import pi
from threading import Thread
import sys
import signal
import math

RANGE = 10000

def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value

class JointControllers():
    """ Summary of class here.

    Joint Controllers plugin for gazebo test.

    """

    def __init__(self):
        self.joint_num = 0
        self.joint_name_list = []
        self.joint_list = {}

        description = get_param('robot_description')
        robot = xml.dom.minidom.parseString(description)
        self.init_urdf(robot)

        use_gui = get_param("use_gui", False)
        if use_gui:
            self.app = QApplication(sys.argv)
            self.gui = JointControllersGui("Joint Controllers", self)
            self.gui.show()
        else:
            self.gui = None

        source_list = get_param("source_list", [])
        self.sources = []
        for source in source_list:
            self.sources.append(rospy.Subscriber(source, JointState, self.source_cb))

        node_name = "/joint_group_position_controller/command"
        self.joint_pub = rospy.Publisher(node_name, Float64MultiArray, queue_size=5)

    def init_urdf(self, robot):
        """

        urdf parser.

        """
        # Find robot
        robot = robot.getElementsByTagName('robot')[0]
        # Find all non-fixed joints
        for child in robot.childNodes:
            min_limit = 0.0
            max_limit = 0.0
            velocity_limit = 0.0
            effort_limit = 0.0

            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                joint_type = child.getAttribute('type')
                if joint_type in ['fixed', 'floating', 'planar']:
                    continue
                name = child.getAttribute('name')
                self.joint_name_list.append(name)
                if joint_type == 'continuous':
                    min_limit = -pi
                    max_limit = pi
                else:
                    try:
                        limit = child.getElementsByTagName('limit')[0]
                        min_limit = float(limit.getAttribute('lower'))
                        max_limit = float(limit.getAttribute('upper'))
                        velocity_limit = float(limit.getAttribute('velocity'))
                        effort_limit = float(limit.getAttribute('effort'))
                    except:
                        rospy.logwarn("%s is not fixed, nor continuous, but limits are not specified!" % name)
                        continue

                safety_param = child.getElementsByTagName('safety_controller')
                if len(safety_param) == 1:
                    param = safety_param[0]
                    if param.hasAttribute('soft_lower_limit'):
                        min_limit = max(min_limit, float(param.getAttribute('soft_lower_limit')))
                    if param.hasAttribute('soft_upper_limit'):
                        max_limit = min(max_limit, float(param.getAttribute('soft_upper_limit')))

                if min_limit >= 0 and max_limit >= 0:
                    home_position = min_limit
                elif  min_limit <= 0 and max_limit <= 0:
                    home_position = max_limit
                else:
                    home_position = 0.0

                joint = {'min': min_limit, 'max': max_limit, 'home': home_position}
                joint['position'] = home_position
                joint['velocity'] = velocity_limit
                joint['effort'] = effort_limit
                joint['type'] = joint_type
                self.joint_list[name] = joint
                self.joint_num = self.joint_num + 1

    def source_cb(self, msg):
        for i in range(len(msg.name)):
            name = msg.name[i]
            if name not in self.joint_name_list:
                continue

            if msg.position:
                position = msg.position[i]
            else:
                position = None
            if msg.velocity:
                velocity = msg.velocity[i]
            else:
                velocity = None
            if msg.effort:
                effort = msg.effort[i]
            else:
                effort = None

            joint = self.joint_list[name]
            if position is not None:
                joint['position'] = position
            if velocity is not None:
                joint['velocity'] = velocity
            if effort is not None:
                joint['effort'] = effort

        if self.gui is not None:
            # Signal instead of directly calling the update_sliders method, to
            # switch to the QThread.  Note that we do this only once at the end
            # to avoid lots of overhead switching threads.
            self.gui.sliderUpdateTrigger.emit()

    def loop(self):
        hz = get_param("rate", 10)  # 10hz
        r = rospy.Rate(hz)

        # Publish
        while not rospy.is_shutdown():
            joints = []
            joints_msg = Float64MultiArray()
            for name in self.joint_name_list:
                joint = self.joint_list[name]
                joints.append(joint['position'])

            # Only publish non-empty messages
            if joints:
                joints_msg.data = joints
                self.joint_pub.publish(joints_msg)

            try:
                r.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                pass

class JointControllersGui(QWidget):
    sliderUpdateTrigger = Signal()

    def __init__(self, title, joint_controllers):
        super(JointControllersGui, self).__init__()
        self.setWindowTitle(title)
        self.joint_controllers = joint_controllers
        self.joint_map = {}
        self.num_rows = self.joint_controllers.joint_num

        self.vlayout = QVBoxLayout(self)
        self.scrollable = QWidget()
        self.gridlayout = QGridLayout()
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)

        font = QFont("Helvetica", 9, QFont.Bold)

        ### Generate sliders ###
        sliders = []
        for name in self.joint_controllers.joint_name_list:
            joint = self.joint_controllers.joint_list[name]

            if joint['min'] == joint['max']:
                continue

            joint_layout = QVBoxLayout()
            row_layout = QHBoxLayout()

            label = QLabel(name)
            label.setFont(font)
            row_layout.addWidget(label)
            display = QLineEdit("0.0")
            display.setAlignment(Qt.AlignRight)
            display.setFont(font)
            display.setReadOnly(True)
            row_layout.addWidget(display)
            joint_layout.addLayout(row_layout)

            slider = QSlider(Qt.Horizontal)
            slider.setFont(font)
            slider.setRange(0, RANGE)
            slider.setValue(0)
            joint_layout.addWidget(slider)

            self.joint_map[name] = {'slidervalue': 0, 'display': display,
                                    'slider': slider, 'joint': joint}
            # Connect to the signal provided by QSignal
            slider.valueChanged.connect(lambda event,name=name: self.onValueChangedOne(name))
            sliders.append(joint_layout)

        self.positions = self.generate_grid_positions(len(sliders), self.num_rows)
        for item, pos in zip(sliders, self.positions):
            self.gridlayout.addLayout(item, *pos)

        # Set zero positions read from parameters
        self.center()

        # Synchronize slider and displayed value
        self.sliderUpdate(None)

        # Set up a signal for updating the sliders based on external joint info
        self.sliderUpdateTrigger.connect(self.updateSliders)

        self.scrollable.setLayout(self.gridlayout)
        self.scroll.setWidget(self.scrollable)
        self.vlayout.addWidget(self.scroll)

        # Buttons for randomizing and centering sliders and
        # Spinbox for on-the-fly selecting number of rows
        self.randbutton = QPushButton('Randomize', self)
        self.randbutton.clicked.connect(self.randomize_event)
        self.vlayout.addWidget(self.randbutton)
        self.ctrbutton = QPushButton('Center', self)
        self.ctrbutton.clicked.connect(self.center_event)
        self.vlayout.addWidget(self.ctrbutton)
        self.maxrowsupdown = QSpinBox()
        self.maxrowsupdown.setMinimum(1)
        self.maxrowsupdown.setMaximum(len(sliders))
        self.maxrowsupdown.setValue(self.joint_controllers.joint_num)
        self.maxrowsupdown.valueChanged.connect(self.reorggrid_event)
        self.vlayout.addWidget(self.maxrowsupdown)
        self.setLayout(self.vlayout)

    def onValueChangedOne(self, name):
        # A slider value was changed, but we need to change the joint_info metadata.
        joint_info = self.joint_map[name]
        joint_info['slidervalue'] = joint_info['slider'].value()
        joint = joint_info['joint']
        joint['position'] = self.sliderToValue(joint_info['slidervalue'], joint)
        joint_info['display'].setText("%.2f" % joint['position'])

    @pyqtSlot()
    def updateSliders(self):
        self.update_sliders()

    def update_sliders(self):
        for name, joint_info in self.joint_map.items():
            joint = joint_info['joint']
            joint_info['slidervalue'] = self.valueToSlider(joint['position'],
                                                           joint)
            joint_info['slider'].setValue(joint_info['slidervalue'])

    def center_event(self, event):
        self.center()

    def center(self):
        rospy.loginfo("Centering")
        for name, joint_info in self.joint_map.items():
            joint = joint_info['joint']
            joint_info['slider'].setValue(self.valueToSlider(joint['home'], joint))

    def reorggrid_event(self, event):
        self.reorganize_grid(event)

    def reorganize_grid(self, number_of_rows):
        self.num_rows = number_of_rows

        # Remove items from layout (won't destroy them!)
        items = []
        for pos in self.positions:
            item = self.gridlayout.itemAtPosition(*pos)
            items.append(item)
            self.gridlayout.removeItem(item)

        # Generate new positions for sliders and place them in their new spots
        self.positions = self.generate_grid_positions(len(items), self.num_rows)
        for item, pos in zip(items, self.positions):
            self.gridlayout.addLayout(item, *pos)

    def generate_grid_positions(self, num_items, num_rows):
        if num_rows==0:
          return []
        positions = [(y, x) for x in range(int((math.ceil(float(num_items) / num_rows)))) for y in range(num_rows)]
        positions = positions[:num_items]
        return positions

    def randomize_event(self, event):
        self.randomize()

    def randomize(self):
        rospy.loginfo("Randomizing")
        for name, joint_info in self.joint_map.items():
            joint = joint_info['joint']
            joint_info['slider'].setValue(
                    self.valueToSlider(random.uniform(joint['min'], joint['max']), joint))

    def sliderUpdate(self, event):
        for name, joint_info in self.joint_map.items():
            joint_info['slidervalue'] = joint_info['slider'].value()
        self.update_sliders()

    def valueToSlider(self, value, joint):
        return (value - joint['min']) * float(RANGE) / (joint['max'] - joint['min'])

    def sliderToValue(self, slider, joint):
        pctvalue = slider / float(RANGE)
        return joint['min'] + (joint['max']-joint['min']) * pctvalue


if __name__ == '__main__':
    try:
        rospy.init_node('joint_controllers')
        joint_controllers = JointControllers()

        if joint_controllers.gui is None:
            joint_controllers.loop()
        else:
            Thread(target=joint_controllers.loop).start()
            signal.signal(signal.SIGINT, signal.SIG_DFL)
            sys.exit(joint_controllers.app.exec_())

    except rospy.ROSInterruptException:
        pass
