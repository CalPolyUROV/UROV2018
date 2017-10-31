import pygame

from DataHandling import *
from pygame.locals import *

__author__ = "Luca"

BACKGROUND = 'ArtificialHorizon.png'
OVERLAY = "ArtificialHorizonOverlay.png"
MARKER = "ArtificialHorizonMarker.png"

X_CONST = 75
Y_CONST = 20


class AH:

    #initializes the artificial horizon

    #dataObjs(list):      A list containing the YPRHandling objects for yaw pitch and roll.
    #UI:                  The UI which it is going to use.

    #@retval: None
    def __init__(self, dataObjs, UI):

        if not isinstance(dataObjs, list):
            raise TypeError("Param 1 is not a list")

        self.UI = UI

        self.background = pygame.image.load(BACKGROUND)                       #Loads the Artificial Horizon images.
        self.overlay = pygame.image.load(OVERLAY)
        self.marker = pygame.image.load(MARKER).convert_alpha()

        self.yaw = getDataObj(dataObjs, "Y")
        self.pch = getDataObj(dataObjs, "P")
        self.rol = getDataObj(dataObjs, "R")

        if(self.yaw == None or self.pch == None or self.pch == None):
            raise LookupError("Could not find objects for yaw, pitch or roll")

        self.yawOffset = 0
        self.pchOffset = 0
        self.rolOffset = 0

        self.UI.textwrite(self.yaw.tytlePosX, self.yaw.posY + Y_CONST, "YCom:")
        self.UI.textwrite(self.pch.tytlePosX, self.pch.posY + Y_CONST, "PCom:")
        self.UI.textwrite(self.rol.tytlePosX, self.rol.posY + Y_CONST, "RCom:")

        self.writeOffset()
        
        self.cover = pygame.Surface((500,250))          # Set background to white.
        self.cover = self.cover.convert()
        self.cover.fill((255, 255, 255))
        
        self.update()

    #writes the offset data on screen just under the YPR data

    #@retval: None
    def writeOffset(self):

        self.UI.textwrite(self.yaw.tytlePosX + X_CONST, self.yaw.posY + Y_CONST, str(self.yawOffset))
        self.UI.textwrite(self.pch.tytlePosX + X_CONST, self.pch.posY + Y_CONST, str(self.pchOffset))
        self.UI.textwrite(self.rol.tytlePosX + X_CONST, self.rol.posY + Y_CONST, str(self.rolOffset))

    #deletes the offset data on screen

    #@retval: None
    def deleteOffset(self):

        self.UI.textdelete(self.yaw.tytlePosX + X_CONST, self.yaw.posY + Y_CONST, str(self.yawOffset))
        self.UI.textdelete(self.pch.tytlePosX + X_CONST, self.pch.posY + Y_CONST, str(self.pchOffset))
        self.UI.textdelete(self.rol.tytlePosX + X_CONST, self.rol.posY + Y_CONST, str(self.rolOffset))

    #Updates the artificial horizon.

    #@retval: None
    def update(self):
        try:
            for event in pygame.event.get(pygame.KEYDOWN):
                if event.type == pygame.KEYDOWN:
                    if event.key == K_n:                 #Nomalizes the Artificial Horizon
                        self.deleteOffset()
                        self.yawOffset = self.yaw.filtered
                        self.pchOffset = self.pch.filtered
                        self.rolOffset = self.rol.filtered
                        self.writeOffset()
                    elif event.key == K_r:               #Sets the Artificial Horizon to the Raw data from the accelerometer.
                        self.deleteOffset()
                        self.yawOffset = 0
                        self.pchOffset = 0
                        self.rolOffset = 0
                        self.writeOffset()

        except:
            print "WARN: Crashed while reading keyboard inputs"

        try:
            if self.pch.wasUpdated or self.rol.wasUpdated:
                backgroundPos = self.background.get_rect()                  #Places down the artificial horizon background
                backgroundPos.centerx = 750
                backgroundPos.centery = 250 + (round(self.pch.filtered * 5 - self.pchOffset * 5))
                self.UI.blit(self.background, backgroundPos)

                overlayPos = self.overlay.get_rect()                  #Places down the artificial horizon overlay
                overlayPos.centerx = 750
                overlayPos.centery = 250
                self.UI.blit(self.overlay, overlayPos)

                markerTrans = pygame.transform.rotate(self.marker, round(self.rol.filtered - self.rolOffset)) #Places down the artificial horizon marker
                markerTransPos = markerTrans.get_rect()
                markerTransPos.centerx = 750
                markerTransPos.centery = 264
                self.UI.blit(markerTrans, markerTransPos)
            
                self.UI.blit(self.cover, (500, 500))

        except:
            print "WARN: Crashed while loading AH images"