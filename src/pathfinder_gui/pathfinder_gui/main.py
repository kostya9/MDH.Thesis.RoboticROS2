import os
os.environ["KIVY_NO_ARGS"] = "1"

from kivy.app import App
from kivy.uix.label import Label
from kivy.uix.textinput import TextInput
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.anchorlayout import AnchorLayout
from kivy.uix.button import Button
from kivy.uix.dropdown import DropDown
from kivy.graphics import Color


import rclpy
import rclpy.client
import rclpy.node

from pathfinder.msg import Move, Rotate

class PathfinderGuiNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('pathfinder_gui')
        self.publisher = self.create_publisher(Move, '/pathfinder/move', 10)
        self.rot_publisher = self.create_publisher(Rotate, '/pathfinder/rotate', 10)

    def set_namespace(self, namespace):
        self.namespace = namespace
        self.publisher = self.create_publisher(Move, '/' + namespace + '/move', 10)
        self.rot_publisher = self.create_publisher(Rotate, '/' + namespace + '/rotate', 10)

    def sendMove(self, x, y):
        req = Move()
        req.x = x
        req.y = y

        print("Sending {0} {1}".format(x, y))
        self.publisher.publish(req)
        # rclpy.spin_once(self)

    def sendRotate(self, speed):
        req = Rotate()
        req.speed = speed

        print("Sending rotate {0}".format(speed))
        self.rot_publisher.publish(req)

class PathfinderGui(App):
    def send_mv_msg(self, btn):
        self.node.sendMove(float(self.x._get_text()), float(self.y._get_text()))

    def send_rot_msg(self, btn):
        self.node.sendRotate(float(self.speed._get_text()))

    def buildMoveBtn(self):
        self.box = BoxLayout(orientation='horizontal', spacing=20, size_hint_y= None)
        self.x = TextInput(hint_text="x", size_hint=(None,None))
        self.x.width = 100
        self.x.height = 30
        self.y = TextInput(hint_text="y", size_hint=(None,None))
        self.y.width = 100
        self.y.height = 30
        self.y.center = (0.5, 0.5)
        self.btn = Button(text='Move', on_release=self.send_mv_msg, size_hint_y = None)
        self.btn.height = 30

        self.box.add_widget(self.x)
        self.box.add_widget(self.y)
        self.box.add_widget(self.btn)

    def buildRotBtn(self):
        self.box_rot = BoxLayout(orientation='horizontal', spacing=20, size_hint_y= None)
        self.speed = TextInput(hint_text="speed", size_hint=(None,None))
        self.speed.width = 100
        self.speed.height = 30
        self.btn_rot = Button(text='Rotate', on_release=self.send_rot_msg, size_hint_y = None)
        self.btn_rot.height = 30

        self.box_rot.add_widget(self.speed)
        self.box_rot.add_widget(self.btn_rot)

    def set_namespace(self, namespace):
        self.node.set_namespace(namespace)
        self.dropdown_label.text = 'Selected robot: ' + namespace
        self.dropdown.dismiss()

    def populate_dropdown(self):
        self.dropdown.clear_widgets()
        topics = self.node.get_topic_names_and_types(no_demangle=True)

        anyRobot = False
        for (topic, _) in topics:
            separated = str.split(topic, "/")
            if separated[-1] == "path":
                namespace = separated[1]
                btn = Button(text=namespace, on_release=lambda btn: self.set_namespace(btn.text), size_hint_y = None)
                self.dropdown.add_widget(btn)
                anyRobot = True
        
        if not anyRobot:
            btn = Button(text="No available robots", size_hint_y = None, disabled=True)
            self.dropdown.add_widget(btn)

    def buildChooseRobotDropdown(self):
        self.box_dropdown = BoxLayout(orientation='horizontal', spacing=20, size_hint_y= None)
        self.dropdown_label = Label(text='No robot selected', size_hint = (1, .4))
        self.dropdown = DropDown(size_hint_y = None)
        self.populate_dropdown()
        self.dropdown_refresh = Button(text='Refresh list', on_release=lambda _: self.populate_dropdown(), size_hint = (1, .4))
        self.dropdown_refresh.width = 50
        self.dropdown_btn = Button(text='Choose robot', size_hint = (1, .4))
        self.dropdown_btn.width = 100

        self.dropdown_btn.bind(on_release=self.dropdown.open)
        self.box_dropdown.add_widget(self.dropdown_refresh)
        self.box_dropdown.add_widget(self.dropdown_btn)
        self.box_dropdown.add_widget(self.dropdown_label)

    def build(self):
        rclpy.init()
        self.node = PathfinderGuiNode()

        self.buildMoveBtn()
        # self.buildRotBtn()
        self.buildChooseRobotDropdown()

        midBox = BoxLayout(orientation='vertical', spacing=20, size_hint_y= None)
        midBox.add_widget(self.box_dropdown)
        midBox.add_widget(self.box)
        # midBox.add_widget(self.box_rot)

        topBox = AnchorLayout(anchor_x= 'center', anchor_y= 'center')
        topBox.add_widget(midBox)

        return topBox

def main():
    PathfinderGui().run()

if __name__ == '__main__':
    main()