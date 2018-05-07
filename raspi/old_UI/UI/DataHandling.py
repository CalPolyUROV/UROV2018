__author__ = "Luca"

CHAR_SIZE = 10
MOT_MEDIUM = 30
MOT_HIGH = 60
MOT_CONST = 2

DEBUG_LABEL = "STS"

import time

class DataHandling(object):

    #Initializes the object DataHandling

    #UI:                The UI Object which it is going to write on.
    #label(String):     The label used on the srail transmissions.
    #tytle(String):     The tytle of object that is going to be writen on screen.
    #unit(String):      The unit of the data been received.
    #posY(num):         The Y position of the text on screen.
    #posX(num):         The X position of the text on screen. Default = 0.
    #size(num):         The size of the filter to be used when calculating the data. Default = 3.

    #@retval: None
    def __init__(self, UI, label, tytle, unit, posY, posX = 0, size = 3):
        if not isinstance(label, str):
            raise TypeError("Param 2 is not a string")

        if not isinstance(tytle, str):
            raise TypeError("Param 3 is not a string")

        if not isinstance(unit, str):
            raise TypeError("Param 4 is not a string")

        if not (isinstance(posY, int) or isinstance(posY, float) or isinstance(posY, long)):
            raise TypeError("Param 5 is not a number")

        if not (isinstance(posX, int) or isinstance(posX, float) or isinstance(posX, long)):
            raise TypeError("Param 6 is not a number")

        if not (isinstance(size, int) or isinstance(size, float) or isinstance(size, long)):
            raise TypeError("Param 7 is not a number")

        self.ui = UI
        self.label = label
        self.tytle = tytle
        self.unit = unit
        self.posY = posY
        self.data = [0.0] * size
        self.filtered = 0
        self.tytlePosX = posX
        self.time = 1
        self.wasUpdated = True
        self.wasUpdatedBefore = True

        self.ui.textwrite(self.tytlePosX, self.posY, self.tytle + ": ")

    #filter the data on the filter and stores it in self.filtered

    #@retval: None
    def filterData(self):
        self.filtered = round(sum(self.data)/len(self.data), 2)

    #updates the DataHandling object

    #rev:       The string received from the serial port

    #@retval: Error code
    #success: True
    #error: False
    def update(self, rev):
        try:
            index = len(self.data) - 1
            while index > 0:
                self.data[index] = self.data[index - 1]
                index -= 1
            self.data[0] = float(rev)

            textDelete = ("%.2f" % self.filtered) + " " + self.unit

            self.ui.textdelete(self.tytlePosX + (len(self.tytle)) * CHAR_SIZE + 10, self.posY, textDelete)
            self.filterData()

            textWrite = ("%.2f" % self.filtered) + " " + self.unit
            self.ui.textwrite(self.tytlePosX + (len(self.tytle)) * CHAR_SIZE + 10, self.posY, textWrite, 10, 125, 10)

            self.wasUpdated = True
            self.wasUpdatedBefore = True

            self.time = time.time()

            return True

        except:
            print "WARN: Could not read " + self.tytle + " data: " + rev
            return False

    #Writes the old data in red if it wasn't updated

    #@retval: None
    def writeOldData(self):
        if self.wasUpdated or not self.wasUpdatedBefore:
            return
        self.wasUpdatedBefore = False
        textWrite = ("%.2f" % self.filtered) + " " + self.unit
        self.ui.textwrite(self.tytlePosX + (len(self.tytle)) * CHAR_SIZE + 10, self.posY, textWrite, 255, 10, 10)

class MotHandling(DataHandling):

    #updates the DataHandling object

    #rev:       The string received from the serial port

    #@retval: Error code
    #success: True
    #error: False
    def update(self, rev):
        try:
            index = len(self.data) - 1
            while index > 0:
                self.data[index] = self.data[index - 1]
                index -= 1
            self.data[0] = float(rev)

            textDelete = ("%.2f" % self.filtered) + " " + self.unit
            self.ui.textdelete(self.tytlePosX + (len(self.tytle)) * CHAR_SIZE + 10, self.posY, textDelete)

            self.filterData()

            if(self.filtered < MOT_MEDIUM):
                R = 10
                G = 125
                B = 10
            elif(self.filtered > MOT_MEDIUM and self.filtered < MOT_HIGH):
                R = 255
                G = 255
                B = 10
            else:
                R = 255
                G = 140
                B = 10

            textWrite = ("%.2f" % self.filtered) + " " + self.unit

            xpos = self.tytlePosX + (len(self.tytle)) * CHAR_SIZE + 10
            self.ui.textwrite(xpos, self.posY, textWrite, R, G, B)
            self.ui.drawRect((xpos + len(textWrite) * CHAR_SIZE + 10, self.posY + 3, round(self.filtered * MOT_CONST), 10), (R,G,B))

            self.wasUpdated = True
            self.wasUpdatedBefore = True

            return True

        except:
            print "Could not read " + self.tytle + " data: " + rev
            return False

    #Writes the old data in red if it wasn't updated

    #@retval: None
    def writeOldData(self):
        if self.wasUpdated or not self.wasUpdatedBefore:
            return
        self.wasUpdatedBefore = False

        textWrite = str(self.filtered) + " " + self.unit
        xpos = self.tytlePosX + (len(self.tytle)) * CHAR_SIZE + 10
        self.ui.textwrite(xpos, self.posY, textWrite, 255, 10, 10)
        self.ui.drawRect((xpos + len(textWrite) * CHAR_SIZE + 10, self.posY + 3, round(self.filtered * MOT_CONST), 10), (255,10,10))

class YPRHandling(DataHandling):

    #filter the data on the filter and stores it in self.filtered

    #@retval: None
    def filterData(self):
        if abs(self.data[0] - self.data[len(self.data) - 1]) <= 5:      #Filters the noise in the data.
            max = len(self.data)
        elif abs(self.data[0] - self.data[len(self.data) - 1]) <= 10:
            max = len(self.data) / 2
        else:
            max = len(self.data) / 3
        i = 0
        total = 0
        while i < max:
            total += self.data[i]
            i += 1
        self.filtered = total / float(i)

#finds a DataHandling object in a list and returns it.

#@retval: DataHandling Object

#LookupError returns: None
def getDataObj(L, tytle):

    if not isinstance(L, list):
        raise TypeError("Param 1 is not a list")

    if not isinstance(tytle, str):
        raise TypeError("Param 2 is not a string")

    for Object in L:
        if type(Object) is DataHandling or type(Object) is YPRHandling or type(Object) is MotHandling:
            if Object.tytle == tytle:
                return Object

    return None