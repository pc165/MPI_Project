import serial
import kivy
from kivy.app import App
from kivy.uix.gridlayout import GridLayout


class MainWidget(GridLayout):
    pass


class myApp(App):

    msg = "hj"
    ser = 0

    def build(self):
        # return a MainWidget() as a root widget
        return MainWidget()

    def configure(self, prt, baud, tout, par, sb):
        ser = serial.Serial(port=prt, baudrate=baud, timeout=tout, parity=par, stopbits=sb)
        return ser

    def write(self, ser, msg):
        ser.write(msg)

    def say_hello(self, msg):
        print(msg)


if __name__ == '__main__':
    myApp.ser = myApp().configure('COM8', 9600, 0, serial.PARITY_EVEN, 1)
    myApp().run()

