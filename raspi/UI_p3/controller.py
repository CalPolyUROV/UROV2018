__author__ = 'johna, tina and luca'

from sfml.window import Joystick
from sys import platform

import time


# These are there codes for the getButton command, you can just use the variables for readability
A = 0
B = 1
X = 2
Y = 3
L_TRIGGER = 4
R_TRIGGER = 5
BACK = 6
START = 7
L_JOYSTICK_CLICK = 8
R_JOYSTICK_CLICK = 9

if platform == "linux" or platform == "linux2":
    L_JOYSTICK_CLICK = 9
    R_JOYSTICK_CLICK = 10
    WIN_BUTTON = 8

# These are their values in the signal sent to the arduino
A_HEX = 0x1
B_HEX = 0x2
X_HEX = 0x4
Y_HEX = 0x8
L_TRIGGER_HEX = 0x10
R_TRIGGER_HEX = 0x20
BACK_HEX = 0x40
BROKEN_HEX = 0x80   # this hex signal over serial seems to lag the arduino by a second, so we are not using it, ever
START_HEX = 0x100
L_JOYSTICK_CLICK_HEX = 0x200
R_JOYSTICK_CLICK_HEX = 0x400

if platform == "linux" or platform == "linux2":
    L_JOYSTICK_CLICK = 9
    R_JOYSTICK_CLICK = 10
    WIN_BUTTON = 8

class Controller:

    #initializes the Controller Object

    #UI:        The UI it is going to use.

    #@retval: None
    def __init__(self, UI):
        self.Joystick = 0;
        self.deadZone = 20
        self.minValue = 0
        self.maxValue = 400
        foundController = False
        UI.textwrite(50, 250, "Looking for controller, press A to choose the controller", 10, 10, 10, 50)
        UI.update()

        while not foundController:
            for i in range(0,5):
                Joystick.update()
                if Joystick.is_connected(i):
                    Joystick.update()
                    if Joystick.is_button_pressed(i, A):
                        foundController = True
                        self.joystick = i
            UI.shouldQuit()

        UI.textdelete(50, 250, "Looking for controller, press A to choose the controller", 50)
        UI.textwrite(450, 250, "Found " + str(self.joystick), 10, 10, 10, 50)
        UI.update()
        time.sleep(2)

        UI.textdelete(450, 250, "Found " + str(self.joystick), 50)

    # used inside the class, not necessary to call from outside this class, use the other calls
    def getAxis(self, joyStickNumber, axis):
        size = self.maxValue - self.minValue
        return ((self.applyDeadZone(Joystick.get_axis_position(joyStickNumber, axis))/(100.0-self.deadZone)) * size) - self.minValue

    def getPrimaryX(self):
        return self.getAxis(self.joystick, Joystick.X)

    def getPrimaryY(self):
        return -self.getAxis(self.joystick, Joystick.Y)

    def getSecondaryX(self):
        return self.getAxis(self.joystick, Joystick.U)

    def getSecondaryY(self):
        if platform == "linux" or platform == "linux2":
            return -self.getAxis(self.joystick, Joystick.V)
        return -self.getAxis(self.joystick, Joystick.R)

    def getTriggers(self):
        if(platform == "linux" or platform == "linux2"):
            value = self.getAxis(self.joystick, Joystick.Z)/2 - self.getAxis(self.joystick, Joystick.R)/2
        else:
            value = self.getAxis(self.joystick, Joystick.Z)
        return value

    def isConnected(self):
        return Joystick.is_connected(self.joystick)

    def getButton(self, button):
        return Joystick.is_button_pressed(self.joystick, button)

    def getNumButtons(self):
        return Joystick.get_button_count(self.joystick)

    def update(self):
        Joystick.update()

    def setDeadZone(self, value):
        self.deadZone = value

    def getValueForButton(self, button):
        return {
            A:A_HEX,
            B:B_HEX,
            X: X_HEX,
            Y: Y_HEX,
            L_TRIGGER: L_TRIGGER_HEX,
            R_TRIGGER: R_TRIGGER_HEX,
            BACK: BACK_HEX,
            START: START_HEX,
            L_JOYSTICK_CLICK: L_JOYSTICK_CLICK_HEX,
            R_JOYSTICK_CLICK: R_JOYSTICK_CLICK_HEX
        }.get(button, 0)

    # This creates a dead zone to prevent situations where you are unable to stop the motors because of touchy input
    def applyDeadZone(self, value):
        negative = False
        if value <= 0:
            negative = True
        temp = abs(value)
        temp -= self.deadZone
        if temp <= 0:
            temp = 0
        return -temp if negative else temp

