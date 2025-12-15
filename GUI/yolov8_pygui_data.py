#!/usr/bin/python
#
# Example using pysimple gui and camera control with yolo v8
# we read sensors and put their values on the video output from the yolov8 object detection using oipenCV
# we demonstrate the functionality of a pysimple gui. read/write data and checkbox bars sliders and buttons
#
import cv2 as cv
import PySimpleGUI as sg
from ultralytics import YOLO
import openpyxl as op
# https://github.com/FiloCara/pyS7 for siemens s7 connection
#
from pyS7 import S7Client, DataType, S7Tag, MemoryArea
import sys
import shlex
from subprocess import Popen, PIPE, STDOUT

# this class wraps the subprocess for shell and pipe
# 
class Shell(object):
    def __init__(self):
        self.p  = None
    def cmd(self, cmd):
        p_args = {'stdin'     : None,
                  'stdout'    : PIPE,
                  'stderr'    : STDOUT,
                  'shell'     : False,}
        return self._execute(cmd, p_args)
    def pipe(self, cmd):
        p_args = {'stdin'     : self.p.stdout,
                  'stdout'    : PIPE,
                  'stderr'    : STDOUT,
                  'shell'     : False,}
        return self._execute(cmd, p_args)
    def _execute(self, cmd, p_args):
        try :
            self.p = Popen(shlex.split(cmd), **p_args)
            return self
        except :
            print('Command Not Found: %s' % (cmd))
    def commit(self):
        result = self.p.communicate()[0]
        status = self.p.returncode
        return (status, result)

# Lucid IO module voltage read on ACM2
read_lucid_volts=`LucidIoCtrl -drs485:/dev/ttyACM2:11 -tV -c0,1,2,3 -r` 
def get_lucid_volts():
    shl = Shell()
    (return_code, stdout) = shl.cmd(read_lucid_volts).commit()
    if return_code == 0:
        l=stdout.decode('utf-8')                                       # CH00:5.000 CH01:5.000 CH02:5.000 CH03:5.000
        ll=l.split(" ")
        lucid_v_vals=[]
        for z in ll:
            valu=z.split(":")[1]
            lucid_v_vals.append(valu)
    return lucid_v_vals
# Create a new 'S7Client' object to connect to S7-300/400/1200/1500 PLC.
# Provide the PLC's IP address and slot/rack information
client = S7Client(address="10.168.6.170", rack=0, slot=1)

# Establish connection with the PLC
client.connect()

# Define area tags to read from the above siemens PLC
itags = [
    "DB1,X0.0",     # Read BIT 0 (first bit) of DB1
    "DB1,X0.6",     # Read BIT 7 (7th bit) of DB1
    "DB2,I31",      # Read INT at address 31 of DB2
    "M54.7",        # Read BIT 8 (eigth bit) in the memory area
    "IW22",         # Read WORD at address 22 in input area
    "QR24",         # Read REAL at address 24 in output area
    "DB1,S10.5"     # Read sequence of CHAR of length 5 starting at address 10 of DB1
]

# Define s7 PLC area tags to write
otags = [
    "DB2,I30",      # => S7Tag(MemoryArea.DB, 1, DataType.INT, 30, 0, 1) - INT at address 30 of DB2
    "DB1,S11.5",    # => S7Tag(MemoryArea.DB, 1, DataType.CHAR, 10, 0, 5) - Sequence of CHAR (string) of length 5 starting at address 10 of DB1
    S7Tag(memory_area=MemoryArea.DB, db_number=3, data_type=DataType.REAL, start=20, bit_offset=0, length=1) # => Sequence of REAL of length 1 starting at address 20 of DB3 
]

path = 0
switch = False
font = cv.FONT_HERSHEY_SIMPLEX
model = YOLO('yolov8n.pt')
w=640
h=480
layout = [[sg.Text('video feed', key='TEXT')],
          [sg.Text('Enter Text for Pop-up ', size=(15, 1)), sg.InputText('default')],
          [sg.Checkbox('CheckBox ', default=True)],
          [sg.Slider(range=(0.0,100.0), default_value=0.0, resolution=1.0, orientation='h', size=(35,None))],
          [sg.Text('Enter Text to write output ', size=(15, 1)), sg.Input(key='-INPUT-')],
          [sg.Text('float a '),sg.InputText(key='num1')],
          [sg.Text('float b '),sg.InputText(key='num2')],      
          [sg.Button('Start Video', key='BUTTON1')],
          [sg.Button('Stop Video', key='BUTTON2', disabled=True)],
		  [sg.Text(size=(40,2), key='-OUTPUT-')],
	      [sg.Slider(range=(0,100),orientation='h',size=(25,None),resolution=1,enable_events=True,key='sld1'), sg.ProgressBar(100,orientation='h',size=(25,15),key='pbar1',bar_color=('#aaa','#fff'))],
          [sg.Text('s7 integer DB1,I31', key='TEXT')],
          [sg.ProgressBar(4095,orientation='h',size=(25,15),key='s7int1',bar_color=('#aaa','#fff'))],
          [sg.Text("File : "), sg.InputText(), sg.FileBrowse('browse', key="inputFilePath")],
          [sg.Button('Write')],
          [sg.Multiline(default_text='', size=(60,10), border_width=2, key='tb1')],
          [sg.Submit(button_text='Submit')],
          [sg.Image(key='IMAGE', size=(w, h))]]
sg.theme('DarkBlue')
# sg.theme('DarkAmber')
window = sg.Window('v_cap_test', layout)
p1 = window['pbar1']
s7i = window['s7int1']
add_v=0
last_send=0

# draws results for detector and shows some external data values read from s7 PLC and lucid io module
def draw_res(img, results, s7d, lucid):
    for i in results:
        for j in i.boxes:
            xy = j.xyxy.numpy().astype(int)
            cv.rectangle(img, (xy[0][0], xy[0][3]), (xy[0][2], xy[0][1]), (0,0,255), 4)
            cls = int(j.cls)
            cv.putText(img, model.model.names[cls], (xy[0][0], xy[0][3]), font, 1, (0, 0, 0), 2, cv.LINE_AA)   
    if s7d[3] == True:
        t = f"v1:{lucid[0]} v1:{lucid[1]} v1:{lucid[2]} v1:{lucid[3]} pressure:{s7d[5]} batch:{s7d[6]} in operation "
        cv.putText(img, t, (20, 20), font, 1, (0, 0, 0), 2, cv.LINE_AA)
	else:
        t = f"v1:{lucid[0]} v1:{lucid[1]} v1:{lucid[2]} v1:{lucid[3]} pressure:{s7d[5]} batch:{s7d[6]} stopped "
        cv.putText(img, t, (20, 20), font, 1, (0, 0, 0), 2, cv.LINE_AA)		
    return 0

while True:
    event, values = window.read(timeout=0)
    p1.UpdateBar(values['sld1'])                                                      # update progress bar with slider1
    print("the input value is ",values['-INPUT-'])
    window['-OUTPUT-'].update('value entered、' + values['-INPUT-'])
    # Read the data from the PLC using the specified tag list
    s7data = client.read(tags=itags)
    # [True, False, 13, True, 10, -2.54943805634653e-12, 'PR010']
    s7i.UpdateBar(s7data[2])                                                           # update bar chart
    if s7data[1] == True:                                                              # write text from bit
        window['-s7bit-'].update('robot moving')    
    else:
        window['-s7bit-'].update('robot stopped')     
    lv = get_lucid_volts()                                                             # read 4 voltages for the lucid io unit 

    if event == 'Submit':
        add_v = float(values['num1'])+float(values['num2'])                            # add values 
        if add_v >= 10 and not (last_send == add_v):                                   # ask to write if total > 10
            sg.PopupYesNo(f'Write {add_v} to DB2.I30 of s7 PLC?')                      # yes no popup
            last_send = add_v
		    if 'Yes':
                print("yes pressed")
                write_dat = [ int(add_v) , values['-INPUT-'][:5], add_v ]
                client.write(tags=otags, values=write_dat)
            elif 'No':
                print("no pressed")            
        update_text = f"values read、{values[0]} cb: {str(values[1])} add : {add_v}"
        window['-TEXT-'].update(update_text)                                           # update main window text
        msg = f"pop-up example、{values[0]}"
        msg += f"checkbox {str(values[1])}"
        msg += f"slider {str(values[2])}"
        sg.popup(msg)                                                                  # call pop-up with text

    if event in (sg.WIN_CLOSED, 'Exit'):
        if switch:
            vc.release()
        break

    if event == 'BUTTON1':                                                            # start video feed from camera 
        vc = cv.VideoCapture(path)                                                    # open usb 0 camera stream
        switch = vc.isOpened()
        window['BUTTON2'].update(disabled=False)
        window['BUTTON1'].update(disabled=True)

    if event == 'BUTTON2':                                                            # stop cam feed 
        vc.release()
        switch = False
        window['BUTTON2'].update(disabled=True)
        window['BUTTON1'].update(disabled=False)

    elif event == 'Write':                                                            # write to xl file
        wb = op.Workbook()
        sheet = wb.active
        sheet['A1'].value = str(add_v) 
        window['tb1'].print(sheet['A1'].value)

    if switch:
        ret, frame = vc.read()
        if ret:
            results = model(frame, device='cpu')                                      # do detection 
            draw_res(frame, results, s7data, lv)
            img = cv.imencode('.png', frame)[1].tobytes()
            window['IMAGE'].update(img)                                               # display video
        else:
            print('cant capture camera')


window.close()




