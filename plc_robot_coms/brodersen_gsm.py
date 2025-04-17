#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Brodersen RTUCOM GSM Modem AT commands
# baud rates can be  300, 600, 1200, 2400, 4800, 9600
# https://brodersen.com/wp-content/uploads/RTUCOM-AT_Command_Manual-mucm9xa_153.pdf
#
# the phonebook can be used as a memory for strings and numbers
# the gpio and adc provide additional 3x analogue in and 8x digital in/out functionality
# the current speaker or on the buzzer can be used to play a series of tones
# the modem can also provide gsm functionality
#
import serial
import time

def read_gpio_brodersen(gpiopin=0, spt='/dev/ttyUSB0', baud_rt=9600):
    io_value = -1
    try:
        read_gpio_pin=f"AT+WIOR={gpiopin}"
        brodersen_init_snd=bytearray(read_gpio_pin.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()
        io_value = int(str(line.decode('utf-8')).split(":")[1])
    except Exception as e:
        print(f'read_gpio_brodersen: {e}')
    finally:    
        ser.close()
    return io_value

def set_gpio_brodersen(gpiopin=0, spt='/dev/ttyUSB0', baud_rt=9600, val=1):
    ret = False
    try:
        set_gpio_pin=f"AT+WIOW={gpiopin},{val}"
        brodersen_init_snd=bytearray(set_gpio_pin.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()
        ret = not str(line).find("OK") == -1    
    except Exception as e:
        print(f'set_gpio_brodersen: {e}')
    finally:    
        ser.close()
    return ret

# mode = 0 == 2 converters, mode = 1 == 3 converters
def set_no_of_adc_brodersen(spt='/dev/ttyUSB0', baud_rt=9600, val=1, mode=1):
    ret = False
    try:
        set_no_of_adc=f"AT+ADC={mode}"
        brodersen_init_snd=bytearray(set_no_of_adc.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()
        ret = not str(line).find("OK") == -1
    except Exception as e:
        print(f'set_no_of_adc_brodersen: {e}')
    finally:    
        ser.close()
    return ret

# gets 10bit ADC returns list of 2 for mode=0 and 3 for mode=1
def get_adc_brodersen(spt='/dev/ttyUSB0', baud_rt=9600):
    val_list = []
    try:
        get_no_of_adc="AT+ADC?"
        brodersen_init_snd=bytearray(get_no_of_adc.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()
        val_list = str(line.decode('utf-8')).split(":")[1].split(",")   
    except Exception as e:
        print(f'get_adc_brodersen: {e}')
    finally:    
        ser.close()
    return val_list

def get_mode_adc_brodersen(spt='/dev/ttyUSB0', baud_rt=9600):
    try:
        get_no_of_adc="AT+ADC=?"
        brodersen_init_snd=bytearray(get_no_of_adc.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()   
    except Exception as e:
        print(f'get_mode_adc_brodersen: {e}')
    finally:    
        ser.close()
    return line.decode('utf-8')

# The supported services are GSM originated (SMS-MO) and terminated short message (SMS-MT), Cell Broadcast Message
# (SMS-CB) services.
def set_sms_serv_brodersen(spt='/dev/ttyUSB0', baud_rt=9600, sv=0):
    try:
        sms_msg=f"AT+CSMS={sv}"
        brodersen_init_snd=bytearray(sms_msg.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline() 
        line = ser.readline()                                           # 2nd line is OK  
    except Exception as e:
        print(f'set_sms_serv_brodersen: {e}')
    finally:    
        ser.close()
    return line.decode('utf-8')

# 1 = text 0 = pdu (hex)
def set_sms_msg_mode_brodersen(spt='/dev/ttyUSB0', baud_rt=9600, md=1):
    try:
        sms_msg=f"AT+CMGF={md}"
        brodersen_init_snd=bytearray(sms_msg.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()  
    except Exception as e:
        print(f'set_sms_msg_mode_brodersen: {e}')
    finally:    
        ser.close()
    return line.decode('utf-8')

def save_to_eeprom_brodersen(spt='/dev/ttyUSB0', baud_rt=9600):
    try:
        sms_msg="AT+CSAS"
        brodersen_init_snd=bytearray(sms_msg.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()  
    except Exception as e:
        print(f'save_to_eeprom_brodersen: {e}')
    finally:    
        ser.close()
    return line.decode('utf-8')

def restore_factory_brodersen(spt='/dev/ttyUSB0', baud_rt=9600):
    ret = False
    try:
        sms_msg="AT&F"
        brodersen_init_snd=bytearray(sms_msg.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()  
        ret = not str(line).find("OK") == -1
    except Exception as e:
        print(f'restore_factory_brodersen: {e}')
    finally:    
        ser.close()
    return ret

def restore_from_eeprom_brodersen(spt='/dev/ttyUSB0', baud_rt=9600):
    try:
        sms_msg="AT+CRES"
        brodersen_init_snd=bytearray(sms_msg.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()  
    except Exception as e:
        print(f'restore_from_eeprom_brodersen: {e}')
    finally:    
        ser.close()
    return line.decode('utf-8')

def rcv_from_network_brodersen(spt='/dev/ttyUSB0', baud_rt=9600):
    ret = False
    try:
        sms_msg="AT+CNMI=2,1,0,0,0"
        brodersen_init_snd=bytearray(sms_msg.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()  
        ret = not str(line).find("OK") == -1
    except Exception as e:
        print(f'rcv_from_network_brodersen: {e}')
    finally:    
        ser.close()
    return ret

#read the message
def read_sms_brodersen(spt='/dev/ttyUSB0', baud_rt=9600, msg_idx=1):
    try:
        smsm=f"AT+CMGR={msg_idx}"
        brodersen_init_snd=bytearray(smsm.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()
    except Exception as e:
        print(f'read_sms_brodersen: {e}')
    finally:    
        ser.close()
    return line.decode('utf-8')

#delete the message
def del_sms_brodersen(spt='/dev/ttyUSB0', baud_rt=9600, msg_idx=1):
    ret = False
    try:
        smsm=f"AT+CMGD={msg_idx}"
        brodersen_init_snd=bytearray(smsm.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()
        ret = not str(line).find("OK") == -1
    except Exception as e:
        print(f'del_sms_brodersen: {e}')
    finally:    
        ser.close()
    return ret 

# list first unread msg
def list_unread_sms_brodersen(spt='/dev/ttyUSB0', baud_rt=9600):
    msg_no = -1
    try:
        smsm="AT+CMGL=“REC UNREAD”"
        brodersen_init_snd=bytearray(smsm.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()
        msg_no = str(line).decode('utf-8).split(":")[1].split(",")[0]
    except Exception as e:
        print(f'list_unread_sms_brodersen: {e}')
    finally:    
        ser.close()
    return msg_no

# This command shall be used to indicate to which service center the message has to be sent.
# The GSM module has no default value for this address. If the application tries to send a message without having indicated
# the service center address, an error will be generated.
# (must be done first with correct center address)
#
def set_sms_serv_center_brodersen(spt='/dev/ttyUSB0', baud_rt=9600, cent=”0696741234”):
    ret = False
    try:
        smsm=f"AT+CSCA={cent}"
        brodersen_init_snd=bytearray(smsm.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()
        ret = not str(line).find("OK") == -1
    except Exception as e:
        print(f'set_sms_serv_center_brodersen: {e}')
    finally:    
        ser.close()
    return ret

def send_sms_brodersen(spt='/dev/ttyUSB0', baud_rt=9600, pno=”+33146290800”, msg="the level is low"):
    ret = False
    try:
        smsm=f"AT+CMGS={pno}"
        brodersen_init_snd=bytearray(smsm.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()
        if not str(line).decode('utf-8).find("CMGS") == -1:
            msg=f"{msg}"
            brodersen_init_snd=bytearray(msg.encode('utf-8'))
            ser = serial.Serial(spt, baud_rt, timeout=0.1)
            ser.write(brodersen_init_snd)
            esc_end = [ 0x1B ]
            ctl_z = [ 0x1A, 'Z' ]
            eof = [ 0x05 ]
            ser.write(bytearray(esc_end))                        # ESC completes the message
            #ser.write(bytearray(ctl_z))                         CTL-Z to end message
            line = ser.readline()
            ret = not str(line).find("OK") == -1
    except Exception as e:
        print(f'send_sms_brodersen: {e}')
    finally:    
        ser.close()
    return ret

def enable_modem_brodersen(spt='/dev/ttyUSB0', baud_rt=9600):
    ret = False
    try:
        smsm="AT+CFUN=1"
        brodersen_init_snd=bytearray(smsm.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()
        ret = not str(line).find("OK") == -1
    except Exception as e:
        print(f'enable_modem_brodersen: {e}')
    finally:    
        ser.close()
    return ret

def disable_modem_brodersen(spt='/dev/ttyUSB0', baud_rt=9600):
    ret = False
    try:
        smsm="AT+CFUN=0"
        brodersen_init_snd=bytearray(smsm.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()
        ret = not str(line).find("OK") == -1
    except Exception as e:
        print(f'disable_modem_brodersen: {e}')
    finally:    
        ser.close()
    return ret

def get_clock_brodersen(spt='/dev/ttyUSB0', baud_rt=9600):
    try:
        smsm="AT+CCLK?"
        brodersen_init_snd=bytearray(smsm.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()
    except Exception as e:
        print(f'get_clock_broderse: {e}')
    finally:    
        ser.close()
    return line.decode('utf-8')

def read_info_brodersen(spt='/dev/ttyUSB0', baud_rt=9600, mo=0):
    mo = mo % 5
    try:
        if mo == 0:
            smsm="AT+CGSN"                  # get the IMEI 
        elif mo == 1:
            smsm="AT+CIMI"                  # read and identify the IMSI
        elif mo == 2:
            smsm="AT+CCID"                  # read the EF-CCID file on the SIM card.
        elif mo == 3:
            smsm="AT+CGMM"                  # band info
        elif mo == 4:
            smsm="AT+CNUM"                  # returns the MSISDN(s) related to the subscriber
        brodersen_init_snd=bytearray(smsm.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()
    except Exception as e:
        print(f'read_info_brodersen: {e}')
    finally:    
        ser.close()
    return line.decode('utf-8')

def call_control_brodersen(spt='/dev/ttyUSB0', baud_rt=9600, mo="answer", no="+33146290800"):
    try:
        if mo == "answer":
            smsm="ATA"                   
        elif mo == "hangup":
            smsm="ATH"                 
        elif mo == "dial":
            smsm="ATD**61*"+no+"**25#"
        elif mo == "last":
            smsm="ATDL"
        elif mo == "auto":
            smsm="ATS0"
        else:
            smsm="ATS0"
        brodersen_init_snd=bytearray(smsm.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()
    except Exception as e:
        print(f'call_control_brodersen: {e}')
    finally:    
        ser.close()
    return line.decode('utf-8')

def set_mic_speaker_brodersen(spt='/dev/ttyUSB0', baud_rt=9600, mod=1, sp=0):
    try:
        mod = mod % 2
        smsm=f"AT+CMUT={mod}"
        brodersen_init_snd=bytearray(smsm.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()
        sp = sp % 2
        smsm=f"AT+SPEAKER={sp}"
        brodersen_init_snd=bytearray(smsm.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()
    except Exception as e:
        print(f'set_mic_speaker_brodersen: {e}')
    finally:    
        ser.close()
    return line.decode('utf-8')

# 0 = none, 1 = network, 2 = network and location
def reg_network_brodersen(spt='/dev/ttyUSB0', baud_rt=9600, mod=1):
    try:
        mod = mod % 3
        smsm=f"AT+CREG={mod}"
        brodersen_init_snd=bytearray(smsm.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()
    except Exception as e:
        print(f'reg_network_brodersen: {e}')
    finally:    
        ser.close()
    return line.decode('utf-8')

def enter_pin_puk_brodersen(spt='/dev/ttyUSB0', baud_rt=9600, mod=1, puk=12345678, pin=1234, npin=0000):
    ret = False
    try:
        mod = mod % 5
        if mod == 0:
            smsm=f"AT+CPIN={pin}"
        elif mod == 1:
            smsm=f"AT+CPIN={puk},{pin}"
        elif mod == 2:
            smsm=f"AT+CPIN2={pin}"
        elif mod == 3:
            smsm=f"AT+CPIN2={puk},{pin}"
        elif mod == 4:
            smsm=f"AT+CPWD=”SC”,{pin},{npin}"

        brodersen_init_snd=bytearray(smsm.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()
        ret = not str(line).find("OK") == -1
    except Exception as e:
        print(f'enter_pin_puk_brodersen: {e}')
    finally:    
        ser.close()
    return ret

# save entry to the phonebook memory
def save_to_phonebook_brodersen(spt='/dev/ttyUSB0', baud_rt=9600, idx=1, pho="012823", typ=129, txt="MY_NO"):
    ret = False
    try:
        smsm=f"AT+CPBW={idx},”{pho}”,{typ},”{txt}”"
        brodersen_init_snd=bytearray(smsm.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()
        ret = not str(line).find("OK") == -1:
    except Exception as e:
        print(f'save_to_phonebook_brodersen: {e}')
    finally:    
        ser.close()
    return ret

# search on description (name) tag
def search_from_phonebook_brodersen(spt='/dev/ttyUSB0', baud_rt=9600, txt="MY_NO", timeout=5.0):
    lines = []
    try:
        if txt == 0:
            smsm="AT+CPBN=0"                                     # read first entry
        elif txt == 1:
            smsm="AT+CPBN=2"                                     # next entry
        elif txt == 2:
            smsm="AT+CPBN=3"                                     # previous entry
        else:
            smsm=f"AT+CPBF=”{txt}”"
        brodersen_init_snd=bytearray(smsm.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        ts = time.time()
        while time.time - ts < timeout:                          # receive all lines in timeout (check long phone book time)
            if ser.in_waiting >0: 
                receive = ser.read_all()
                lines.append(receive.decode('utf-8'))
    except Exception as e:
        print(f'search_from_phonebook_brodersen: {e}')
    finally:    
        ser.close()
    return lines

# delete a particular id
def erase_phonebookid_brodersen(spt='/dev/ttyUSB0', baud_rt=9600, id1=1):
    try:
        smsm=f"AT+CPBW={id1}"
        brodersen_init_snd=bytearray(smsm.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        if ser.in_waiting >0: 
            receive = ser.read_all()           
    except Exception as e:
        print(f'erase_phonebookid_brodersen: {e}')
    finally:    
        ser.close()
    return receive.decode('utf-8')

# load entries from a range of ids
def load_from_phonebookid_brodersen(spt='/dev/ttyUSB0', baud_rt=9600, id1=1, id2=4, timeout=5.0):
    lines = []
    try:
        smsm=f"AT+CPBR={id1},{id2}"
        brodersen_init_snd=bytearray(smsm.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        ts = time.time()
        while time.time - ts < timeout:
            if ser.in_waiting >0: 
                receive = ser.read_all()
                lines.append(receive.decode('utf-8'))
    except Exception as e:
        print(f'load_from_phonebookid_brodersen: {e}')
    finally:    
        ser.close()
    return lines

# frequncy table for tones
# note number and frequency
#
C_m1_m2 = [0,8.2]
Cs_m1_m2 = [1,8.7]
D_m1_m2	= [2,9.2]
Ds_m1_m2 = [3,9.7]
E_m1_m2 = [4,10.3]
F_m1_m2 = [5,10.9]
Fs_m1_m2 = [6,11.6]
G_m1_m2 = [7,12.2]
Gs_m1_m2 = [8,13.0]
A_m1_m2 = [9,13.8]
As_m1_m2 = [10,14.6]
B_m1_m2 = [11,15.4]
# 0 -1 	
C_0_m1 = [12,16.4]
Cs_0_m1 = [13,17.3]
D_0_m1 = [14,18.4]
Ds_0_m1 = [15,19.4]
E_0_m1 = [16,20.6]
F_0_m1 = [17,21.8]
Fs_0_m1 = [18,23.1]
G_0_m1 = [19,24.5]
Gs_0_m1 = [20,26.0]
A_0_m1 = [21,27.5]
As_0_m1 = [22,29.1]
B_0_m1 = [23,30.9]
# 1	0	
C_1_0 =	[24,32.7]
Cs_1_0 = [25,34.6]
D_1_0 =	[26,36.7]
Ds_1_0 = [27,38.9]
E_1_0 =	[28,41.2]
F_1_0 =	[29,43.7]
Fs_1_0 = [30,46.2]
G_1_0 =	[31,49.0]
Gs_1_0 = [32,51.9]
A_1_0 =	[33,55.0]
As_1_0 = [34,58.3]
B_1_0 =	[35,61.7]
# 2	1	
C_2_1 =	[36,65.4]
Cs_2_1 = [37,69.3]
D_2_1 =	[38,73.4]
Ds_2_1 = [39,77.8]
E_2_1 =	[40,82.4]
F_2_1 =	[41,87.3]
Fs_2_1 = [42,92.5]
G_2_1 =	[43,98.0]
Gs_2_1 = [44,103.8]
A_2_1 =	[45,110.0]
As_2_1 = [46,116.5]
B_2_1 = [47,123.5]
#3	2	
C_3_2 =	[48,130.8]
Cs_3_2=	[49,138.6]
D_3_2=	[50,146.8]
Ds_3_2=	[51,155.6]
E_3_2=	[52,164.8]
F_3_2=	[53,174.6]
Fs_3_2=	[54,185.0]
G_3_2=	[55,196.0]
Gs_3_2=	[56,207.7]
A_3_2=	[57,220.0]
As_3_2=	[58,233.1]
B_3_2=	[59,246.9]
#4	3	
C_4_3=	[60,261.6]
Cs_4_3=	[61,277.2]
D_4_3=	[62,293.7]
Ds_4_3=	[63,311.1]
E_4_3=	[64,329.6]
F_4_3=	[65,349.2]
Fs_4_3=	[66,370.0]
G_4_3=	[67,392.0]
Gs_4_3=	[68,415.3]
A_4_3=	[69,440.0]
As_4_3=	[70,466.2]
B_4_3=	[71,493.9]
#5	4	
C_5_4=	[72,523.3]
Cs_5_4=	[73,554.4]
D_5_4=	[74,587.3]
Ds_5_4=	[75,622.3]
E_5_4=	[76,659.3]
F_5_4=	[77,698.5]
Fs_5_4=	[78,740.0]
G_5_4=	[79,784.0]
Gs_5_4=	[80,830.6]
A_5_4=	[81,880.0]
As_5_4=	[82,932.3]
B_5_4=	[83,987.8]
#6	5	
C_6_5=	[84,1046.5]
Cs_6_5=	[85,1108.7]
D_6_5=	[86,1174.7]
Ds_6_5=	[87,1244.5]
E_6_5=	[88,1318.5]
F_6_5=	[89,1396.9]
Fs_6_5=	[90,1480.0]
G_6_5=	[91,1568.0]
Gs_6_5=	[92,1661.2]
A_6_5=	[93,1760.0]
As_6_5=	[94,1864.7]
B_6_5=	[95,1975.5]
#7	6	
C_7_6=	[96,2093.0]
Cs_7_6=	[97,2217.5]
D_7_6=	[98,2349.3]
Ds_7_6=	[99,2489.0]
E_7_6=	[100,2637.0]
F_7_6=	[101,2793.8]
Fs_7_6=	[102,2960.0]
G_7_6=	[103,3136.0]
Gs_7_6=	[104,3322.4]
A_7_6=	[105,3520.0]
As_7_6=	[106,3729.3]
B_7_6=	[107,3951.1]
# 8	7	
C_8_7=	[108,4186.0]
C_8_7=	[109,4434.9]
D_8_7=	[110,4698.6]
Ds_8_7=	[111,4978.0]
E_8_7=	[112,5274.0]
F_8_7=	[113,5587.7]
Fs_8_7=	[114,5919.9]
G_8_7=	[115,6271.9]
Gs_8_7=	[116,6644.9]
A_8_7=	[117,7040.0]
As_8_7=	[118,7458.6]
B_8_7=	[119,7902.1]
#9	8	
C_9_8=	[120,8372.0]
Cs_9_8=	[121,8869.8]
D_9_8=	[122,9397.3]
Ds_9_8=	[123,9956.1]
E_9_8=	[124,10548.1]
F_9_8=	[125,11175.3]
Fs_9_8=	[126,11839.8]
G_9_8=	[127,12543.9]

# note duration table @ 120 bpm
# worked in units of 100ms as per the modem function
#
W = round(1000 / 100.0)
T = round(750 / 100.0)
H = round(500 / 100.0)
Q = round(250 / 100.0)
E = round(125 / 100.0)
S = round(62 / 100.0)

# <mode>[,<dest>,<freq>,<gain>,<duration> This parameter settles the duration of the tone (unit of 100 ms)
def play_tone_sms_brodersen(spt='/dev/ttyUSB0', baud_rt=9600, m=1, d=1, f=D_4_3[1], g=9, du=50):
    ret = False
    g = g % 16
    try:
        smsm=f"AT+WTONE={m},{d},{f},{g},{du}"
        brodersen_init_snd=bytearray(smsm.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()
        ret = not str(line).find("OK") == -1
    except Exception as e:
        print(f'play_tone_sms_brodersen: {e}')
    finally:    
        ser.close()
    return ret

def stop_tone_sms_brodersen(spt='/dev/ttyUSB0', baud_rt=9600):
    ret = False
    try:
        smsm="AT+WTONE=0"
        brodersen_init_snd=bytearray(smsm.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(brodersen_init_snd)
        line = ser.readline()
        ret = not str(line).find("OK") == -1
    except Exception as e:
        print(f'stop_tone_sms_brodersen: {e}')
    finally:    
        ser.close()
    return ret

# _dc is dont close the port as it might effect the music timing takes the port handle as first arg
# <mode>[,<dest>,<freq>,<gain>,<duration> This parameter settles the duration of the tone (unit of 100 ms)
def play_tone_sms_brodersen_dc(ser, spt='/dev/ttyUSB0', baud_rt=9600, m=1, d=1, f=D_4_3[1], g=9, du=50):
    ret = False
    g = g % 16
    try:
        smsm=f"AT+WTONE={m},{d},{f},{g},{du}"
        brodersen_init_snd=bytearray(smsm.encode('utf-8'))
        ser.write(brodersen_init_snd)
        line = ser.readline()
        ret = not str(line).find("OK") == -1
    except Exception as e:
        print(f'play_tone_sms_brodersen: {e}')
    return ret

def stop_tone_sms_brodersen_dc(ser, spt='/dev/ttyUSB0', baud_rt=9600):
    ret = False
    try:
        smsm="AT+WTONE=0"
        brodersen_init_snd=bytearray(smsm.encode('utf-8'))
        ser.write(brodersen_init_snd)
        line = ser.readline()
        ret = not str(line).find("OK") == -1
    except Exception as e:
        print(f'stop_tone_sms_brodersen: {e}')
    return ret

# example of how to play music using the above functions (see if it keeps time okay)
def play_tune1_brodersen(spt='/dev/ttyUSB0', baud_rt=9600, bpm=120):
    ser = serial.Serial(spt, baud_rt, timeout=0.1)
    for j in range(0,2):
        play_tone_sms_brodersen_dc(ser, spt, baud_rt, m=1, d=1, f=C_5_4[1], g=9, du=round(H*(120.0/bpm)))
        play_tone_sms_brodersen_dc(ser, spt, baud_rt, m=1, d=1, f=E_5_4[1], g=9, du=round(Q*(120.0/bpm)))
        play_tone_sms_brodersen_dc(ser, spt, baud_rt, m=1, d=1, f=D_5_4[1], g=9, du=round(Q*(120.0/bpm)))
        stop_tone_sms_brodersen_dc(ser, spt, baud_rt)
        time.sleep((Q*(120.0/bpm)*100)/1000.0)                                                   # off for a quarter beat
        play_tone_sms_brodersen_dc(ser, spt, baud_rt, m=1, d=1, f=G_4_3[1], g=9, du=round(H*(120.0/bpm)))
        play_tone_sms_brodersen_dc(ser, spt, baud_rt, m=1, d=1, f=E_4_3[1], g=9, du=round(Q*(120.0/bpm)))
        play_tone_sms_brodersen_dc(ser, spt, baud_rt, m=1, d=1, f=D_4_3[1], g=9, du=round(Q*(120.0/bpm)))
        stop_tone_sms_brodersen_dc(ser, spt, baud_rt)
        time.sleep((Q*(120.0/bpm)*100)/1000.0)                                                   # off for a quarter beat
    ser.close()