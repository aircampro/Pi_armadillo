# coding: utf-8
#
# Use of menu bar in pysimple gui to control the pygame display
# and use openCV to write text of a real time value on the pygame screen
#
import PySimpleGUI as sg	
import pygame
import cv2

# define the screens
screens = [ "1.png", "2.png", "3.png", "num6.png" ]
back = [ "a.png", "b.png", "sasa.png", "all.png" ]
active_scn=0
	
def main():
    global active_scn
    w=300
    h=200
    L=[[sg.MenuBar([
	    ['Area1','New','Area2','Area3','Area4','Quit'],
	    ['Branch',['1','2','All']]], key='mb1')],
		[sg.Image(key='IMAGE', size=(w, h))]]
    window = sg.Window('menu window example', L, size=(w,h))
    png=screens[0]
    pygame.init()
    screen = pygame.display.set_mode((w, g))
    pygame.display.set_caption(" Menu Example ")
    clock  = pygame.time.Clock()
    v = 0
    while True:
	    event, values = window.read()	
	    print('event ˆ:',event,', value :',values)		
	    if event==None or values['mb1']=='Quit':	
		    break
	    elif values['mb1']=='Area1':	
		    png=screens[0]
	    elif values['mb1']=='Area2':	
		    png=screens[1]
	    elif values['mb1']=='Area3':	
		    png=screens[2]
	    elif values['mb1']=='Area4':	
		    png=screens[3]	
	    elif values['mb1']=='Branch':	
		    png=screens[0]
	    elif values['mb1']=='1':	
		    png=screens[1]
	    elif values['mb1']=='2':	
		    png=screens[2]
	    elif values['mb1']=='All':	
		    png=screens[3]	
        fm = cv2.imread(png)
        fm = cv2.putText(fm, f"pressure : {v}", (20, 20), font, .3, (0, 0, 255), 1, cv2.LINE_AA)	        # write the text to the image
        cv2.imwrite(fm, "./text.png")		
        show_img = pygame.image.load("./text.png")	                                                        # display in pygame
        graphic = screen.blit(show_img, (50, 50))
        img = cv2.imencode('.png', fm)[1].tobytes()
        window['IMAGE'].update(img)                                               # display in sg
        pygame.display.update()
        clock.tick(60)  
        v += 1
        v %= 100		
    window.close()
    pygame.quit()

if __name__ == '__main__':
    main()
