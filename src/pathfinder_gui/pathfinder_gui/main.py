from kivy.app import App
from kivy.uix.label import Label
from kivy.uix.textinput import TextInput
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.anchorlayout import AnchorLayout
from kivy.uix.button import Button

import rclpy
import rclpy.client
import rclpy.node

from pathfinder.msg import Move

class PathfinderGuiNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('pathfinder_gui')
        self.publisher = self.create_publisher(Move, '/pathfinder/move', 10)

    def sendMove(self, x, y):
        req = Move()
        req.x = x
        req.y = y

        print("Sending {0} {1}".format(x, y))
        self.publisher.publish(req)
        # rclpy.spin_once(self)

class PathfinderGui(App):
    def send_msg(self, btn):
        self.node.sendMove(float(self.x._get_text()), float(self.y._get_text()))

    def build(self):
        rclpy.init()
        self.node = PathfinderGuiNode()

        self.box = BoxLayout(orientation='horizontal', spacing=20, size_hint_y= None)
        self.x = TextInput(hint_text="x", size_hint=(None,None))
        self.x.width = 100
        self.x.height = 30
        self.y = TextInput(hint_text="y", size_hint=(None,None))
        self.y.width = 100
        self.y.height = 30
        self.y.center = (0.5, 0.5)
        self.btn = Button(text='Move', on_release=self.send_msg, size_hint_y = None)
        self.btn.height = 30

        self.box.add_widget(self.x)
        self.box.add_widget(self.y)
        self.box.add_widget(self.btn)

        topBox = AnchorLayout(anchor_x= 'center', anchor_y= 'center')
        topBox.add_widget(self.box)

        return topBox

def main():
    PathfinderGui().run()

if __name__ == '__main__':
    main()