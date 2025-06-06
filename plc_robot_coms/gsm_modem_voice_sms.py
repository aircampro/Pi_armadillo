# general gsm modem commands
#
import serial
import time

# we are auto answering the voice call to speak back to the control command center
#
def auto_answer_voice_calls(spt='/dev/ttyUSB0', baud_rt=9600):
    io_value = False
    try:
        auto_ans="ATS0=1"
        aa_snd=bytearray(auto_ans.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(aa_snd)
        line = ser.readline()
        io_value = line == "OK"
    except Exception as e:
        print(f'auto answer set: {e}')
    finally:    
        ser.close()
    return io_value

# Get the registration status of the device. If the status is ‘1’, the device is registered and in home network
#
def get_registration(spt='/dev/ttyUSB0', baud_rt=9600):
    io_value = False
    try:
        auto_ans="AT+CREG?"
        aa_snd=bytearray(auto_ans.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(aa_snd)
        line = ser.readline()
        io_value = line == "1"
    except Exception as e:
        print(f'get registration: {e}')
    finally:    
        ser.close()
    return io_value

# Get the network
#
def get_net(spt='/dev/ttyUSB0', baud_rt=9600):
    try:
        c="AT+COPS?"
        aa_snd=bytearray(c.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(aa_snd)
        line = ser.readline()
    except Exception as e:
        print(f'get network: {e}')
    finally:    
        ser.close()
    return line

# check for pin code True if no pin needed other wise it enters it abd looks for ok abd returns true
#
def check_pin(pin=1234, spt='/dev/ttyUSB0', baud_rt=9600):
    io_value = False
    try:
        c="AT+CPIN?"
        aa_snd=bytearray(c.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(aa_snd)
        line = ser.readline()
        io_value = line == "Ready"                         # no pin needed
        if io_value == False:                              # pin needed so enter it
            c="AT+CPIN=\"{pin}\""
            aa_snd=bytearray(c.encode('utf-8'))
            ser.write(aa_snd)
            line = ser.readline()
            io_value = line == "OK" 
    except Exception as e:
        print(f'get registration: {e}')
    finally:    
        ser.close()
    return io_value

# Get signal
#
def get_sig_strength(spt='/dev/ttyUSB0', baud_rt=9600):
    try:
        c="AT+CSQ"
        aa_snd=bytearray(c.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(aa_snd)
        line = ser.readline()
    except Exception as e:
        print(f'get signal: {e}')
    finally:    
        ser.close()
    return line

# Get id
#
def get_id(spt='/dev/ttyUSB0', baud_rt=9600):
    try:
        auto_ans="AT+CCID"
        aa_snd=bytearray(auto_ans.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(aa_snd)
        line = ser.readline()
    except Exception as e:
        print(f'get id: {e}')
    finally:    
        ser.close()
    return line

# Get IMEI Retrieves the International Mobile Equipment Identity of the module
#
def get_imei(spt='/dev/ttyUSB0', baud_rt=9600):
    try:
        auto_ans="AT+GSN"
        aa_snd=bytearray(auto_ans.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(aa_snd)
        line = ser.readline()
    except Exception as e:
        print(f'get imei: {e}')
    finally:    
        ser.close()
    return line

# Get IMSI
#
def get_imsi(spt='/dev/ttyUSB0', baud_rt=9600):
    try:
        auto_ans="AT+CIMI"
        aa_snd=bytearray(auto_ans.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(aa_snd)
        line = ser.readline()
    except Exception as e:
        print(f'get imsi: {e}')
    finally:    
        ser.close()
    return line

# Get iccid Returns the ICCID of the SIM
#
def get_iccid(spt='/dev/ttyUSB0', baud_rt=9600):
    try:
        auto_ans="AT+CCID"
        aa_snd=bytearray(auto_ans.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(aa_snd)
        line = ser.readline()
    except Exception as e:
        print(f'get iccid: {e}')
    finally:    
        ser.close()
    return line

# Get phone no
#
def get_phone_no(spt='/dev/ttyUSB0', baud_rt=9600):
    try:
        auto_ans="AT+CNUM"
        aa_snd=bytearray(auto_ans.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(aa_snd)
        line = ser.readline()
    except Exception as e:
        print(f'get phone number: {e}')
    finally:    
        ser.close()
    return line

# dial a number
#
def dial_number(tel_no="12345354", spt='/dev/ttyUSB0', baud_rt=9600):
    io_value = -1
    try:
        cmd=f"ATD{tel_no}"
        aa_snd=bytearray(cmd.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(aa_snd)
        line = ser.readline()
        io_value = line == "OK"
    except Exception as e:
        print(f'dial: {e}')
    finally:    
        ser.close()
    return io_value

# hang up
#
def hang_up_call(spt='/dev/ttyUSB0', baud_rt=9600):
    io_value = -1
    try:
        cmd="ATH"
        aa_snd=bytearray(cmd.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(aa_snd)
        line = ser.readline()
        io_value = line == "OK"
    except Exception as e:
        print(f'hang-up: {e}')
    finally:    
        ser.close()
    return io_value

# tcp/udp connection
#
# Set transparent (1) or non-transparent mode (0)
#
def simcomm_send_data(your_data, n, mode=0, host=10.0.0.1, port=80, spt='/dev/ttyUSB0', baud_rt=9600):
    io_value = -1
    try:
        cmd=f"AT+CIPMODE={mode}"
        aa_snd=bytearray(cmd.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(aa_snd)
        line = ser.readline()
        io_value = line == "OK"
        if io_value == True:
            cmd=f"AT+CIPSTART=[{n},]{mode},{host},{port}"
            aa_snd=bytearray(cmd.encode('utf-8'))
            ser.write(aa_snd)
            line = ser.readline()
            io_value = line == "OK"	
            if io_value == True:
                cmd="AT+CIPSEND"
                aa_snd=bytearray(cmd.encode('utf-8'))
                ser.write(aa_snd)
                cmd = "WAIT=1"
                aa_snd=bytearray(cmd.encode('utf-8'))
                ser.write(aa_snd)
				snd=bytearray(your_data.encode('utf-8'))
				ser.write(snd)
				ctl_z = [ 0x1A, 'Z' ]
				ser.write(ctl_z)				
                line = ser.readline()
                io_value = line == "OK"			
    except Exception as e:
        print(f'send data: {e}')
    finally:    
        ser.close()
    return io_value

# tcp/udp connection
#
# Set transparent (1) or non-transparent mode (0)
#
def simcomm_serve_data(mode=0, port=80, spt='/dev/ttyUSB0', baud_rt=9600):
    io_value = -1
    try:
        cmd=f"AT+CIPMODE={mode}"
        aa_snd=bytearray(cmd.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(aa_snd)
        line = ser.readline()
        io_value = line == "OK"
        if io_value == True:
            cmd=f"AT+CIPSTART=[{n},]{mode},{host},{port}"
            aa_snd=bytearray(cmd.encode('utf-8'))
            ser.write(aa_snd)
            line = ser.readline()
            io_value = line == "OK"	
            if io_value == True:
                cmd="AT+CIPSERVER={mode}[,{port}]"
                aa_snd=bytearray(cmd.encode('utf-8'))
                ser.write(aa_snd)				
                line = ser.readline()
                io_value = line == "OK"			
    except Exception as e:
        print(f'server: {e}')
    finally:    
        ser.close()
    return io_value

def simcomm_send_text_sms(your_data, cent="+1234567890", t_no="+447860603540", spt='/dev/ttyUSB0', baud_rt=9600):
    io_value = -1
    try:
        cmd="AT+CMGF=1"                                                        # text mode
        aa_snd=bytearray(cmd.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(aa_snd)
        line = ser.readline()
        io_value = line == "OK"
        if io_value == True:
            cmd=f"AT+CSCA={cent}"
            aa_snd=bytearray(cmd.encode('utf-8'))
            ser.write(aa_snd)
            line = ser.readline()
            io_value = line == "OK"	
            if io_value == True:
                cmd="AT+CMGS={t_no}"
                aa_snd=bytearray(cmd.encode('utf-8'))
                ser.write(aa_snd)
                cmd = "WAIT=1"
                aa_snd=bytearray(cmd.encode('utf-8'))
                ser.write(aa_snd)
				snd=bytearray(your_data.encode('utf-8'))
				ser.write(snd)
				ctl_z = [ 0x1A, 'Z' ]
				ser.write(ctl_z)				
                line = ser.readline()
                io_value = line == "OK"			
    except Exception as e:
        print(f'sms txt send: {e}')
    finally:    
        ser.close()
    return io_value
						 
# check sim card
#
def check_sim(spt='/dev/ttyUSB0', baud_rt=9600):
    io_value = -1
    try:
        cmd="AT+CPIN?"
        aa_snd=bytearray(cmd.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(aa_snd)
        line = ser.readline()
        io_value = line.find("OK")
    except Exception as e:
        print(f'check sim: {e}')
    finally:    
        ser.close()
    return io_value
	
