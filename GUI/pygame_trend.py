#!/usr/bin/python
#
# example using pychart to show trend informastion
#
import pygame, sys
import pygame_chart as pyc
import pandas as pd
import matplotlib.pyplot as plt

# read_csv which has the data and show bar graph using matplot lib
df = pd.read_csv('data.csv', encoding='utf-8')
df.plot(x='X', y='Y', kind='bar')  
plt.show()

# pip3 install xlrd==1.2.0
# df= pd.read_excel('data.xlsx')
# pygame app for figure to run
pygame.init()
screen = pygame.display.set_mode((800,600))

# Figure instance on screen with position and size
figure = pyc.Figure(screen, 50, 50, 700, 500)

while True:
    events = pygame.event.get()
    for event in events:
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                pygame.quit()
                sys.exit()
    # Add chart title with a title
    figure.add_title('pygameChart Example')
    # Add legend and gridlimes
    figure.add_legend()
    figure.add_gridlines()
    # add a line chart. First argument "name" should be unique for every chart
    figure.line('LineChart', df['Y'], df['X'])
    figure.bar('BarChart', df['Y'], df['X'])
    figure.scatter('ScatterChart', df['Y'].astype(float), df['X'].astype(float))
    # draw figure with specified properties
    figure.draw()    
    pygame.display.update()