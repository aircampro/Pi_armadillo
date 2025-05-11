#
# interface to move the robot to the co-ordinates specified as X and Y on the HMI
# $ sudo apt-get install python3-tk
# $ sudo apt-get install tk-dev
#
import tkinter
import time

# uses unitree robots github sdk
import sys
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import SportClient
import math

# start connection to robot as a global object to be accessed via the GUI
def startRobotComms(arg1):
    ChannelFactoryInitialize(0, arg1)

    test_option = TestOption(name=None, id=None) 
    user_interface = UserInterface()
    user_interface.test_option_ = test_option

    sport_client = SportClient() 
    return sport_client

if len(sys.argv) >= 1:
    a1 = sys.argv[1]
    sport_client = startRobotComms(a1)
    sport_client.SetTimeout(10.0)
    sport_client.Init()    
else:
    print("arg1 is port to communicate with robot on ") 
    sys.exit(-1)

def click_btn():
    text_message.set("move Y....") 
    x = start_value.get()
    x1 = "move Y: "+str(x)+" "
    text_y.insert(tkinter.END, x1)
    sport_client.Move(0,int(x),0)
def click_btn2():
    text_message.set("move X....") 
    x = start_value2.get()
    x1 = "move X: "+str(x)+" "
    text_y.insert(tkinter.END, x1)
    sport_client.Move(int(x),0,0)
def click_btn3():
    text_message.set("move XY....") 
    y = start_value.get()
    x = start_value2.get()
    x1 = "move XY: "+str(x)+" "+str(y)+" "
    text_y.insert(tkinter.END, x1)
    sport_client.Move(int(x),int(y),0)
def click_btn4():
    text_message.set("starting sequence of operations... connecting to the robot")
    sport_client.SpeedLevel(1)
    sport_client.Move(0,0.5,0.3) 
    sport_client.SwitchGait(1) 
    time.sleep(2)
    sport_client.RecoveryStand() 
    time.sleep(2)
    sport_client.SpeedLevel(2)
    sport_client.Move(1.0,0.5,0) 
    sport_client.SwitchGait(1)        
    time.sleep(2)
    sport_client.BalanceStand() 
    text_message.set("completed operation...")
def click_btn5():
    text_message.set("stopping....")
    sport_client.StopMove()  
    text_y.delete('1.0', tkinter.END)  
    text_message.set("stopped....")    
def delete_btn():
    text_y.delete('1.0', tkinter.END)

# ================ HMI ========================
#
root = tkinter.Tk()
root.title('move the robot')
root.geometry('600x600')

#text_x = tkinter.Text(width = 50, height = 6, font = ('Times New Roman', 16))
start_value = tkinter.Entry(width=30)
start_value.insert(tkinter.END, "0.1")
lbl_massage = tkinter.Label(text='Y-coordinate')
lbl_massage.place(x=10, y=320)
start_value.place(x=20, y=350)
start_value2 = tkinter.Entry(width=30)
start_value2.insert(tkinter.END, "0.1")
lbl_massage2 = tkinter.Label(text='X-coordinate')
lbl_massage2.place(x=10, y=220)
start_value2.place(x=20, y=250)
#text_x.place(x = 20, y = 350)

text_y = tkinter.Text(width = 50, height = 6, font = ('Times New Roman', 16), bg = 'lightyellow')
text_y.place(x = 20, y = 50)
btn = tkinter.Button(text = 'Move Y', font = ('Times New Roman', 16), command = click_btn, bg = 'cyan')
btn.place(x = 90, y = 550)
btn = tkinter.Button(text = 'Move X', font = ('Times New Roman', 16), command = click_btn2, bg = 'lightblue')
btn.place(x = 190, y = 550)
btn = tkinter.Button(text = 'Move XY', font = ('Times New Roman', 16), command = click_btn3, bg = 'magenta')
btn.place(x = 290, y = 550)
btn = tkinter.Button(text = 'Clear message field', font = ('Times New Roman', 16), command = delete_btn, bg = 'lightgreen')
btn.place(x = 390, y = 550)
btn = tkinter.Button(text = 'Stop', font = ('Times New Roman', 16), command = click_btn5, bg = 'pink')
btn.place(x = 190, y = 500)

text_message = tkinter.StringVar()
text_message.set("this is a message field")
label_info = tkinter.Label(textvariable = text_message, font = ("", 10,"bold"), justify = "left")
label_info.grid(row = 1, column = 0)

btn = tkinter.Button(text='Seq', font=("MSゴシック", "10", "bold"), fg="green", relief= "ridge",command=click_btn4)
btn.place(x=10, y=550)

root.mainloop()