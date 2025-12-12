#!/usr/bin/python
#
# pygame example of analog bar with triangle pointing at the analog value 0-100 picsels in size 
#
import os
import pygame
from pygame import Rect
from pygame.math import Vector2

os.environ['SDL_VIDEO_CENTERED'] = '1'

class GaugeState(x=70,y=30):
    def __init__(self):
        self.tankPos = Vector2(x, y)

    def update(self, moveGaugeCommand):
        self.tankPos += moveGaugeCommand
        if self.tankPos.x < 0:
            self.tankPos.x = 0
        if self.tankPos.y < 0:
            self.tankPos.y = 0

class UserInterface():
    def __init__(self, title="Example of an analog gauge",x=70,y=30):
        pygame.init()
        pygame.font.init()
        self.font = pygame.font.Font(None, 50)

        self.GaugeState = GaugeState(x,y)

        # Rendering properties
        self.cellSize = Vector2(1280, 720.0)                                # define screen size
        self.arrow = pygame.image.load("arrow.png")

        # Window
        self.window = pygame.display.set_mode(self.cellSize.x,self.cellSize.y)
        pygame.display.set_caption(title)
        pygame.display.set_icon(pygame.image.load("icon.png"))
        self.moveGaugeCommand = Vector2(0,0)
        self.tag_name = title
        # Loop properties
        self.clock = pygame.time.Clock()
        self.running = True

    def processInput(self, val):
        self.moveGaugeCommand = Vector2(0,0)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
                break
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                    break
        self.moveGaugeCommand.y = 100 - val

    def update(self):
        self.GaugeState.update(self.moveGaugeCommand)

    def render(self):
        self.window.fill((0,0,0))

        # Gauge object base
        spritePoint = self.GaugeState.tankPos
		GREEN = (0, 255, 70)
        rectangle1 = Rect(self.GaugeState.tankPos.x-60, self.GaugeState.tankPos.y, self.GaugeState.tankPos.x-20, 100)
		pygame.draw.rect(self.window, GREEN, rectangle1)
        self.window.blit(self.arrow, spritePoint)
        textimg1 = self.font.render(self.tag_name, True, pygame.Color("BLUE"))
        self.window.blit(textimg1, (self.GaugeState.tankPos.x-60, 140))		
        pygame.display.update()    

    def run(self):
        val = 0
        while self.running:
            self.processInput(val)
            self.update()
            self.render()
            self.clock.tick(60)
            val +=1                                           # count up a value this would be a real value you read from i,o or comms for example (here im just simulating it)
			val %=101
if __name__ == '__main__':
    userInterface = UserInterface("Temperature TT0001")
    userInterface.run()

    pygame.quit()
