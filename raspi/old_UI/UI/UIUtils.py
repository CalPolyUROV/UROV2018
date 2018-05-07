import pygame
from pygame.locals import *

__author__ = "Luca"

class UI:
    #Initialization function of the clas UI

    #@retval: None
    def __init__(self, procs):
        pygame.init()                                           #initializes the UI
        self.procs = procs
        self.screen = pygame.display.set_mode((1000, 750), DOUBLEBUF)
        pygame.display.set_caption("Cal Poly Control Center")

        self.background = pygame.Surface(self.screen.get_size())          # Set background to white.
        self.background = self.background.convert()
        self.background.fill((255, 255, 255))
        self.update()

    #Updates the UI elements

    #@retval: None
    def update(self):
        pygame.display.update()

    #Adds a string to the screen. The function "update" needs to be called so it appears.

    #Positionx:     The X coordinate of the left most letter of the string
    #Positiony:     The Y coordinate of the left most letter of the string
    #Text:          The string to be written on the screen
    #r:             The color of the string in red. Range 0-255. Default = 10.
    #g:             The color of the string in green. Range 0-255. Default = 10.
    #b:             The color of the string in blue. Range 0-255. Default = 10.
    #size           The size of the string. Default = 25.

    #@retval: None
    def textwrite(self, Positionx, Positiony, Text, r = 10, g = 10 , b = 10, size = 25): #Function that write on screen Strings.

        font = pygame.font.Font(None, size)
        text = font.render(Text, 0, (r, g, b))
        textpos = text.get_rect()
        textpos.x = Positionx
        textpos.y = Positiony
        self.background.blit(text, textpos)
        if len(self.procs) > 0:
            self.procs[0].update(False)
            self.screen.blit(self.background, (0, 0))
            self.procs[0].update(True)
        else:
            self.screen.blit(self.background, (0, 0))


    #Deletes a String on the screen

    #Positionx:     The X coordinate of the left most letter of the string
    #Positiony:     The Y coordinate of the left most letter of the string
    #Text:          The string to be written on the screen
    #size           The size of the string. Default = 25.

    #@retval: None
    def textdelete(self, Positionx, Positiony, Text, size = 25):                 #Function that deletes strings

        font = pygame.font.Font(None, size)
        text = font.render(Text, 0, (255, 255, 255))
        textpos = text.get_rect()
        textpos.x = Positionx
        textpos.y = Positiony
        self.background.blit(text, textpos)
        if len(self.procs) > 0:
            self.procs[0].update(False)
            self.screen.blit(self.background, (0, 0))
            self.procs[0].update(True)
        else:
            self.screen.blit(self.background, (0, 0))


    #Adds an object to the background but it does not update it.

    #obj:       The object to be added
    #objpos:    The object position in the format (X,Y).

    #@retval: None
    def blit(self, obj, objPos):
        try:
            self.background.blit(obj, objPos)
        except:
            print "WARN: Couldn't blit object!"

    #Draws a rectangle on the screen

    #rect:          The rectangle object in the format (left, top, width, height).
    #color:         The color of the rectangle in RGB standard. Default = (10,10,10).
    #width:         The width of the rectangle. If 0, it gets completly filled. Default = 0.

    #@retval: None
    def drawRect(self, rect, color = (10, 10, 10), width = 0):

        pygame.draw.rect(self.background, color, rect, width)

    #Checks if the X button was clicked and if it was it quits the program.

    #@retval: None
    def shouldQuit(self):
        for event in pygame.event.get():
            if event.type == QUIT:
                for proc in self.procs:
                    print "Calling procs"
                    proc.end_proc()
                    proc.stop()
                quit()
