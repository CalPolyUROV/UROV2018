import serial
import serial.tools.list_ports

from pygame.locals import *

from Autopilot import *

import sys

from UIUtils import *

from time import sleep

from controller import *
import serial_finder

from ArtificialHorizon import *

__author__ = 'johna, tina and luca'

#version 3.1

POS_PORTS_MULTI = 65
if sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
    POS_PORTS_MULTI = 120

POS_PORTS_X = 135
POS_PORTS_Y = 0
CON_TO_X = 130
CON_TO_Y = 15

start = 0

procs = []

st = True

p_factor = 1

no_serial = False

ports = serial_finder.serial_ports()
port = None

UI = UI(procs)

cont = Controller(UI)

UI.textwrite(175, 250, "Please connect the serial device", 10, 10, 10, 50)
UI.textwrite(250, 300, "Press X for test mode", 10, 10, 10, 50)

UI.update()                         #Updates display


while (len(ports) == 0 or port == None):
    ports = serial_finder.serial_ports()
    port = serial_finder.find_port(ports)
    UI.shouldQuit()
    cont.update()
    if(cont.getButton(2)):
        no_serial = True
        break

#port ='/dev/ttyS0'

UI.textdelete(175, 250, "Please connect the serial device", 50)
UI.textdelete(250, 300, "Press X for test mode", 50)
UI.textwrite(0, POS_PORTS_Y, "Possible ports: ")

for i in range(len(ports)):
    UI.textwrite(POS_PORTS_X + POS_PORTS_MULTI * i, POS_PORTS_Y, str(ports[i]))

UI.textwrite(0, CON_TO_Y, "Connected To: ")
UI.textwrite(CON_TO_X, CON_TO_Y, str(port))

outbound = serial.Serial(
    port=port,
    baudrate=9600,
    parity=serial.PARITY_NONE,   # parity is error checking, odd means the message should have an odd number of 1 bits
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,   # eight bits of information per pulse/packet
    timeout=0.1
)

UI.textwrite(0, 310, "Power factor:")

dataObjs = [DataHandling(UI, "PSR", "Pressure", "mbars", 70),
            #DataHandling(UI, "VLT", "Current", "amps", 90),
            DataHandling(UI, "ACL", "x", "", 130, 0),
            DataHandling(UI, "ACL", "y", "", 130, 150),
            DataHandling(UI, "ACL", "z", "", 130, 300),
            DataHandling(UI, "TMP", "Temperature", "degrees C",90),
            DataHandling(UI, "DPT", "Depth", "feet", 110),
            MotHandling(UI, "MOT", "M1", "", 190),
            MotHandling(UI, "MOT", "M2", "", 210),
            MotHandling(UI, "MOT", "M3", "", 230),
            MotHandling(UI, "MOT", "M4", "", 250),
            MotHandling(UI, "MOT", "M5", "", 270),
            MotHandling(UI, "MOT", "M6", "", 290),
            YPRHandling(UI, "GYR", "Y", "", 150, 0, 10),
            YPRHandling(UI, "GYR", "P", "", 150, 150, 10),
            YPRHandling(UI, "GYR", "R", "", 150, 300, 10)]

AH = AH(dataObjs, UI)

AutoPilot = AutoPilot(dataObjs, UI)

camera = camera(UI)

try:
    camera.start()
    procs.append(camera)
except:
    pass

UI.update()

while True:

    if no_serial:
        UI.textdelete(0, 35, "Serial device Connected")
        UI.textwrite(0, 35, "Connect the serial device", 255, 10, 10)
        UI.textdelete(CON_TO_X, CON_TO_Y, str(port))
        for i in range(len(ports)):
            UI.textdelete(POS_PORTS_X + POS_PORTS_MULTI * i, POS_PORTS_Y, str(ports[i]))
        ports = serial_finder.serial_ports()
        for i in range(len(ports)):
            UI.textwrite(POS_PORTS_X + POS_PORTS_MULTI * i, POS_PORTS_Y, str(ports[i]))
        if(len(ports) > 0):
            port = serial_finder.find_port(ports)
            if(port != None):
                outbound = serial.Serial(
                    port=port,
                    baudrate=115200,
                    parity=serial.PARITY_NONE,   # parity is error checking, odd means the message should have an odd number of 1 bits
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,   # eight bits of information per pulse/packet
                    timeout=0.1
                )
                UI.textwrite(CON_TO_X, CON_TO_Y, str(port))
                no_serial = False

    if not no_serial:
        UI.textdelete(0, 35, "Connect the serial device")
        UI.textwrite(0, 35, "Serial device Connected", 10, 125, 10)


    if not cont.isConnected():                                      # Updates controller and shows whether it is connected.
        UI.textdelete(0, 50, "Controller connected")
        UI.textwrite(0, 50, "Connect the controller", 255, 10, 10)

    else:
        UI.textdelete(0, 50, "Connect the controller")
        UI.textwrite(0, 50, "Controller connected", 10, 125, 10)

    for Object in dataObjs:
            Object.wasUpdated = False

    cont.update()
    buttons1 = 0x0
    buttons2 = 0x0
    # going to eight would include the start button; however, it seems that when 0x80 (only the start button) is sent
    # the arduino lags for a second or two and reports false values for buttons
    for i in range(0, cont.getNumButtons()):
        if(cont.getButton(i)):
            if(cont.getValueForButton(i) <= 0xFF):
                buttons1 += cont.getValueForButton(i)
                if buttons1 == 32:
                    AutoPilot.toggle()

            else:
                buttons2 += cont.getValueForButton(i) >> 8
                if buttons2 == 1:
                    UI.textdelete(140, 310, str(p_factor))
                    p_factor = p_factor / 2.0
                    if(p_factor < 0.25):
                        p_factor = 1

    try:
        if(not no_serial):
            outbound.write("STR") #  sends a signal to tell that this is the start of data
            #print("STR")
            outbound.write(chr(buttons1))# writes the buttons first
            outbound.write(chr(buttons2))

            outbound.write(str(int(cont.getPrimaryX() * p_factor)))# casts the floats to ints, then to strings for simple parsing
            outbound.write(" ")
            outbound.write(str(int(cont.getPrimaryY() * p_factor)))
            outbound.write(" ")
            outbound.write(str(int(cont.getSecondaryX() * p_factor)))
            outbound.write(" ")
            outbound.write(str(int(cont.getSecondaryY() * p_factor)))
            outbound.write(" ")
            outbound.write(str(int(AutoPilot.CalcAltitude(cont.getTriggers(), p_factor))))
            outbound.write(" ")

    except serial.serialutil.SerialException:
        no_serial = True

    except:
        print("WARN: Crashed while sending controller input")

    try:
        if (not no_serial):
            counter = 10
            proceed = False

            while counter > 0:
                counter -= 1
                if outbound.readable():
                    if "STR" == outbound.read(3):
                        proceed = True
                        if st:
                            start = time.time()
                            st = False
                        break

            if(proceed):                                            # Reads the serial line.
                linesToRead = int(outbound.read(3)) # allows for up to 999 lines to be read...
                if linesToRead > 24:
                    linesToRead = 24
                for i in range(0, linesToRead // 2):
                    label = outbound.readline().rstrip().lstrip()
                    found = False

                    rev = outbound.readline().rstrip().split(",")
                    j = 0

                    if label == DEBUG_LABEL:
                        found = True
                        print(label)
                        print(rev)

                    else:
                        for Object in dataObjs:
                            if Object.label == label:
                                found = True
                                Object.update(rev[j])
                                j += 1

                    if not found:                                                       #In case it receives weird data, it prints it out on the terminal
                        print("INFO: unknown datatype: " + label)
                        print("      data: " + str(rev))

            else:
                print("WARN: No Response From Arduino")
                if not st:
                    end = time.time()
                    st = True
                    print("INFO: Lost data after" + ("%.2f" % (end - start)) + "seconds")

    except serial.serialutil.SerialException:
        no_serial = True

    except:
        print("WARN: Crashed while reading from arduino")

    for Object in dataObjs:
            Object.writeOldData()

    AH.update()

    UI.textwrite(140, 310, str(p_factor))

    #print(pygame.mouse.get_pos())
    #print(pygame.mouse.get_pressed())
    UI.update()                         #Updates display
    #sleep(0.03)                         #Waits for 30ms

    UI.shouldQuit()