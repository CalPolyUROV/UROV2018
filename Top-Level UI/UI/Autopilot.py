import pygame.camera
from pygame.locals import *

from time import sleep

__author__ = "Luca"

LIST_SIZE = 20

MOT_LIM = 400

P_CONST = 10
I_CONST = 10
D_CONST = 10

import threading
from DataHandling import *

DEVICE = "/dev/video0"
#DEVICE = "/dev/bus/usb/001/008"

class AutoPilot:

    def __init__(self, dataObjs, UI):
        self.UI = UI
        self.errors = [0.0] * LIST_SIZE
        self.times = [0.0] * LIST_SIZE
        self.times[0] = 1.1
        self.P = 0
        self.I = 0
        self.D = 0
        self.goal = 0
        self.on = False
        self.pressure = getDataObj(dataObjs, "Pressure")
        self.init = True

        if(self.pressure == None):
            print "FATAL: Could not initialize the Autopilot!"
            self.init = False

        UI.textwrite(250, 310, "AutoPilot:")
        self.UI.textwrite(350, 310, str(self.on))

    def CalcAltitude(self, user, pfactor):

        if not self.init:
            return user * pfactor

        if(user != 0 or not self.on):
            error = 0
        else:
            error = self.goal - self.pressure.filtered

        for i in range(LIST_SIZE - 1):
            self.errors[LIST_SIZE - i - 1] = self.errors[LIST_SIZE - i - 2]
            self.times[LIST_SIZE - i - 1] = self.times[LIST_SIZE - i - 2]

        self.errors[0] = error
        self.times[0] = self.pressure.time

        if(not self.on):
            return user * pfactor

        elif(user != 0):
            self.goal = self.pressure.filtered
            PID = self.I + self.D + user * pfactor

            return clamp(PID)

        self.I = 0
        for i in range(LIST_SIZE - 1):
            self.I += self.errors[i] * (self.times[i] - self.times[i + 1])
        self.I *= I_CONST

        self.D = ((self.errors[0] - self.errors[1]) / (self.times[0] - self.times[1]))*  D_CONST

        self.P = error * P_CONST

        PID = self.P + self.I +self.D

        return clamp(PID)

    def toggle(self):
        self.UI.textdelete(350, 310, str(self.on))
        if(self.on):
            self.on = False
        elif(self.init):
            self.on = True

        self.UI.textwrite(350, 310, str(self.on))

def clamp(num):
    ret = num
    if (ret > MOT_LIM):
        ret = MOT_LIM
    elif (ret < MOT_LIM):
        ret = -MOT_LIM

    return ret

class camera(threading.Thread):
    def __init__(self, UI):
        try:
            pygame.camera.init()
            self.UI = UI
            self.camera = pygame.camera.Camera(DEVICE, (500, 370))
            self.camera.start()
            self.snapshot = pygame.surface.Surface((500, 370), 0, self.UI.screen).convert()
            self.runing = True
            self.shouldUpdate = True
            threading.Thread.__init__(self)
            self._stop = threading.Event()
        except:
            self.runing = False
            print "FATAL: Could not initialize Camera"

    def run(self):
        while self.runing:
            try:
                if self.camera.query_image():
                    self.snapshot = self.camera.get_image(self.snapshot)
                    self.UI.blit(self.snapshot, (0, 330))
                    print self.shouldUpdate
                    if self.shouldUpdate:
                        self.UI.update()
                    self.UI.shouldQuit()
                    sleep(0.03)
            except:
                pass

    def update(self, shouldUpdate):
         self.shouldUpdate = shouldUpdate

    def end_proc(self):
        print "Called!"
        self.run = False

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()