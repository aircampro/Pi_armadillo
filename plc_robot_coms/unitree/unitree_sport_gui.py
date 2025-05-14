#
# simple gui window to operate unitree robot using the sdk
#
import tkinter
import threading
import time
from tkinter import messagebox

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

# define each button for the HMI and their respective action    
class Button(tkinter.Button):
    def __init__(self):
        super().__init__(
            master=None,
            text="Click to Stand Up",
            width=100,
            fg="green",
            command=self.Button_click
            )
    def Button_click(self):
        text_message.set("started stand up operation please wait")
        thread = threading.Thread(target = action_but)
        thread.start()
        #thread.join()

class Button2(tkinter.Button):
    def __init__(self):
        super().__init__(
            master=None,
            text="Click to Stand Down",
            width=100,
            fg="red",
            command=self.Button_click
            )
    def Button_click(self):
        text_message.set("started stand down operation please wait")
        thread = threading.Thread(target = action_but2)
        thread.start()
        #thread.join()

class Button3(tkinter.Button):
    def __init__(self):
        super().__init__(
            master=None,
            text="Click to X Move",
            width=100,
            fg="cyan",
            bg="black",
            command=self.Button_click
            )
    def Button_click(self):
        text_message.set("started X move operation please wait")
        thread = threading.Thread(target = action_but3)
        thread.start()
        #thread.join()

class Button4(tkinter.Button):
    def __init__(self):
        super().__init__(
            master=None,
            text="Click to Stop Move",
            width=100,
            fg="orange",
            bg="black",
            command=self.Button_click
            )
    def Button_click(self):
        text_message.set("stopping moving operation please wait")
        thread = threading.Thread(target = action_but4)
        thread.start()
        #thread.join()

class Button5(tkinter.Button):
    def __init__(self):
        super().__init__(
            master=None,
            text="Click to Start Y Move",
            width=100,
            fg="magenta",
            bg="black",
            command=self.Button_click
            )
    def Button_click(self):
        text_message.set("starting Y move operation please wait")
        thread = threading.Thread(target = action_but5)
        thread.start()
        #thread.join()

class Button6(tkinter.Button):
    def __init__(self):
        super().__init__(
            master=None,
            text="Click to Z Move",
            width=100,
            fg="yellow",
            bg="black",
            command=self.Button_click
            )
    def Button_click(self):
        text_message.set("starting Z move operation please wait")
        thread = threading.Thread(target = action_but6)
        thread.start()
        #thread.join()

class Button7(tkinter.Button):
    def __init__(self):
        super().__init__(
            master=None,
            text="Click to Damp",
            width=100,
            fg="blue",
            command=self.Button_click
            )
    def Button_click(self):
        text_message.set("starting damp operation please wait")
        thread = threading.Thread(target = action_but7)
        thread.start()
        #thread.join()

class Button8(tkinter.Button):
    def __init__(self):
        super().__init__(
            master=None,
            text="Click to perform sequence",
            width=100,
            fg="white",
            bg="black",
            command=self.Button_click
            )
    def Button_click(self):
        text_message.set("starting sequence operations please wait")
        thread = threading.Thread(target = action_but8)
        thread.start()
        #thread.join()
      
def action_but():
    Button_act["state"] = tkinter.DISABLED
    Button_act2["state"] = tkinter.DISABLED
    Button_act3["state"] = tkinter.DISABLED
    Button_act4["state"] = tkinter.DISABLED
    Button_act5["state"] = tkinter.DISABLED
    Button_act6["state"] = tkinter.DISABLED
    Button_act7["state"] = tkinter.DISABLED
    Button_act8["state"] = tkinter.DISABLED
    try:
        sport_client.StandUp()
    except Exception:
        messagebox.showwarning("warning", "warning")
    else:
        messagebox.showinfo("message", "standup complete")
    finally:
        text_message.set("completed action")
        Button_act["state"] = tkinter.NORMAL
        Button_act2["state"] = tkinter.NORMAL
        Button_act3["state"] = tkinter.NORMAL
        Button_act4["state"] = tkinter.NORMAL   
        Button_act5["state"] = tkinter.NORMAL
        Button_act6["state"] = tkinter.NORMAL
        Button_act7["state"] = tkinter.NORMAL
        Button_act8["state"] = tkinter.NORMAL

def action_but2():
    Button_act["state"] = tkinter.DISABLED
    Button_act2["state"] = tkinter.DISABLED
    Button_act3["state"] = tkinter.DISABLED
    Button_act4["state"] = tkinter.DISABLED
    Button_act5["state"] = tkinter.DISABLED
    Button_act6["state"] = tkinter.DISABLED
    Button_act7["state"] = tkinter.DISABLED
    Button_act8["state"] = tkinter.DISABLED
    try:
        sport_client.StandDown()
    except Exception:
        messagebox.showwarning("warning", "warning")
    else:
        messagebox.showinfo("message", "standdown complete")
    finally:
        text_message.set("completed action")
        Button_act["state"] = tkinter.NORMAL
        Button_act2["state"] = tkinter.NORMAL
        Button_act3["state"] = tkinter.NORMAL
        Button_act4["state"] = tkinter.NORMAL
        Button_act5["state"] = tkinter.NORMAL
        Button_act6["state"] = tkinter.NORMAL
        Button_act7["state"] = tkinter.NORMAL
        Button_act8["state"] = tkinter.NORMAL

def action_but3():
    Button_act["state"] = tkinter.DISABLED
    Button_act2["state"] = tkinter.DISABLED
    Button_act3["state"] = tkinter.DISABLED
    Button_act4["state"] = tkinter.DISABLED
    Button_act5["state"] = tkinter.DISABLED
    Button_act6["state"] = tkinter.DISABLED
    Button_act7["state"] = tkinter.DISABLED
    Button_act8["state"] = tkinter.DISABLED
    try:
        sport_client.Move(0.5,0,0)
    except Exception:
        messagebox.showwarning("warning", "warning")
    else:
        messagebox.showinfo("message", "move complete")
    finally:
        text_message.set("completed action")
        Button_act["state"] = tkinter.NORMAL
        Button_act2["state"] = tkinter.NORMAL
        Button_act3["state"] = tkinter.NORMAL
        Button_act4["state"] = tkinter.NORMAL
        Button_act5["state"] = tkinter.NORMAL
        Button_act6["state"] = tkinter.NORMAL
        Button_act7["state"] = tkinter.NORMAL
        Button_act8["state"] = tkinter.NORMAL

def action_but4():
    Button_act["state"] = tkinter.DISABLED
    Button_act2["state"] = tkinter.DISABLED
    Button_act3["state"] = tkinter.DISABLED
    Button_act4["state"] = tkinter.DISABLED
    Button_act5["state"] = tkinter.DISABLED
    Button_act6["state"] = tkinter.DISABLED
    Button_act7["state"] = tkinter.DISABLED
    Button_act8["state"] = tkinter.DISABLED
    try:
        sport_client.StopMove()
    except Exception:
        messagebox.showwarning("warning", "warning")
    else:
        messagebox.showinfo("message", "stop move complete")
    finally:
        text_message.set("completed action")
        Button_act["state"] = tkinter.NORMAL
        Button_act2["state"] = tkinter.NORMAL
        Button_act3["state"] = tkinter.NORMAL
        Button_act4["state"] = tkinter.NORMAL
        Button_act5["state"] = tkinter.NORMAL
        Button_act6["state"] = tkinter.NORMAL
        Button_act7["state"] = tkinter.NORMAL
        Button_act8["state"] = tkinter.NORMAL

def action_but5():
    Button_act["state"] = tkinter.DISABLED
    Button_act2["state"] = tkinter.DISABLED
    Button_act3["state"] = tkinter.DISABLED
    Button_act4["state"] = tkinter.DISABLED
    Button_act5["state"] = tkinter.DISABLED
    Button_act6["state"] = tkinter.DISABLED
    Button_act7["state"] = tkinter.DISABLED
    Button_act8["state"] = tkinter.DISABLED
    try:
        sport_client.Move(0,0.5,0)
    except Exception:
        messagebox.showwarning("warning", "warning")
    else:
        messagebox.showinfo("message", "move y complete")
    finally:
        text_message.set("completed action")
        Button_act["state"] = tkinter.NORMAL
        Button_act2["state"] = tkinter.NORMAL
        Button_act3["state"] = tkinter.NORMAL
        Button_act4["state"] = tkinter.NORMAL
        Button_act5["state"] = tkinter.NORMAL
        Button_act6["state"] = tkinter.NORMAL
        Button_act7["state"] = tkinter.NORMAL
        Button_act8["state"] = tkinter.NORMAL

def action_but6():
    Button_act["state"] = tkinter.DISABLED
    Button_act2["state"] = tkinter.DISABLED
    Button_act3["state"] = tkinter.DISABLED
    Button_act4["state"] = tkinter.DISABLED
    Button_act5["state"] = tkinter.DISABLED
    Button_act6["state"] = tkinter.DISABLED
    Button_act7["state"] = tkinter.DISABLED
    Button_act8["state"] = tkinter.DISABLED
    try:
        sport_client.Move(0,0,0.5)
    except Exception:
        messagebox.showwarning("warning", "warning")
    else:
        messagebox.showinfo("message", "move z complete")
    finally:
        text_message.set("completed action")
        Button_act["state"] = tkinter.NORMAL
        Button_act2["state"] = tkinter.NORMAL
        Button_act3["state"] = tkinter.NORMAL
        Button_act4["state"] = tkinter.NORMAL
        Button_act5["state"] = tkinter.NORMAL
        Button_act6["state"] = tkinter.NORMAL
        Button_act7["state"] = tkinter.NORMAL
        Button_act8["state"] = tkinter.NORMAL

def action_but7():
    Button_act["state"] = tkinter.DISABLED
    Button_act2["state"] = tkinter.DISABLED
    Button_act3["state"] = tkinter.DISABLED
    Button_act4["state"] = tkinter.DISABLED
    Button_act5["state"] = tkinter.DISABLED
    Button_act6["state"] = tkinter.DISABLED
    Button_act7["state"] = tkinter.DISABLED
    Button_act8["state"] = tkinter.DISABLED
    try:
        sport_client.Damp()
    except Exception:
        messagebox.showwarning("warning", "warning")
    else:
        messagebox.showinfo("message", "damp complete")
    finally:
        text_message.set("completed action")
        Button_act["state"] = tkinter.NORMAL
        Button_act2["state"] = tkinter.NORMAL
        Button_act3["state"] = tkinter.NORMAL
        Button_act4["state"] = tkinter.NORMAL
        Button_act5["state"] = tkinter.NORMAL
        Button_act6["state"] = tkinter.NORMAL
        Button_act7["state"] = tkinter.NORMAL
        Button_act8["state"] = tkinter.NORMAL

def action_but8():
    Button_act["state"] = tkinter.DISABLED
    Button_act2["state"] = tkinter.DISABLED
    Button_act3["state"] = tkinter.DISABLED
    Button_act4["state"] = tkinter.DISABLED
    Button_act5["state"] = tkinter.DISABLED
    Button_act6["state"] = tkinter.DISABLED
    Button_act7["state"] = tkinter.DISABLED
    Button_act8["state"] = tkinter.DISABLED
    try:
        sport_client.SpeedLevel(1)
        sport_client.Move(0.5,0.5,0.1)
        time.sleep(2)
        sport_client.SpeedLevel(2)
        sport_client.Move(1.0,0.5,0) 
        sport_client.SwitchGait(1)        
        time.sleep(2)
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
    except Exception:
        messagebox.showwarning("warning", "warning")
    else:
        messagebox.showinfo("message", "sequence complete")
    finally:
        text_message.set("completed sequence actions")
        Button_act["state"] = tkinter.NORMAL
        Button_act2["state"] = tkinter.NORMAL
        Button_act3["state"] = tkinter.NORMAL
        Button_act4["state"] = tkinter.NORMAL
        Button_act5["state"] = tkinter.NORMAL
        Button_act6["state"] = tkinter.NORMAL
        Button_act7["state"] = tkinter.NORMAL
        Button_act8["state"] = tkinter.NORMAL
        
# create a GUI for wrapping the robot sdk commands       
def runGUI():   
    root = tkinter.Tk()
    root.title(u"=== UniTree Robot control GUI ===")
    Button_act = Button()
    Button_act.grid(row = 1, column = 0)
    Button_act2 = Button2()
    Button_act2.grid(row = 2, column = 0)
    Button_act3 = Button3()
    Button_act3.grid(row = 4, column = 0)  
    Button_act4 = Button4()
    Button_act4.grid(row = 7, column = 0)
    Button_act5 = Button5()
    Button_act5.grid(row = 5, column = 0)  
    Button_act6 = Button6()
    Button_act6.grid(row = 6, column = 0)
    Button_act7 = Button7()
    Button_act7.grid(row = 3, column = 0)  
    Button_act8 = Button8()
    Button_act8.grid(row = 8, column = 0)    
    text_message = tkinter.StringVar()
    text_message.set("Click Button to Start Action")
    label_info = tkinter.Label(textvariable = text_message, font = ("", 10,"bold"), justify = "left")
    label_info.grid(row = 0, column = 0)
    tkinter.mainloop()

if __name__ == "__main__":
    runGUI()
