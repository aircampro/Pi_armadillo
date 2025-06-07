# SIM7500/7600 modem AT commands
#
# ref:- https://simcom.ee/documents/SIM7600C/SIM7500_SIM7600%20Series_AT%20Command%20Manual_V1.01.pdf
#
# some commands are the same as the std gsm modem i.e. voice call etc
# not all of the many commands have been implemented
#

# AT+CGPS Start/Stop GPS session
# When AT+CGPS=1, the GPS session automatically selects cold start or hot start.
#
def start_gps(id=1, spt='/dev/ttyUSB0', baud_rt=9600, val=1):
    ret = False
    try:
        set_sess=f"AT+CGPS={id}"
        sim_init_snd=bytearray(set_sess.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(sim_init_snd)
        line = ser.readline()
        ret = not str(line).find("OK") == -1    
    except Exception as e:
        print(f'start gps session: {e}')
    finally:    
        ser.close()
    return ret

# AT+CGPSINFO Get GPS fixed position information
# <lat> 
# Latitude of current position. Output format is ddmm.mmmmmm 
# <N/S> 
# N/S Indicator, N=north or S=south 
# <log> 
# Longitude of current position. Output format is dddmm.mmmmmm 
# <E/W> 
# E/W Indicator, E=east or W=west 
# <date> 
# Date. Output format is ddmmyy 
# <UTC time> 
# UTC Time. Output format is hhmmss.s 
# <alt> 
# MSL Altitude. Unit is meters. 
# <speed> 
# Speed Over Ground. Unit is knots. 
# <course> 
# Course. Degrees. 
# <time> 
# The range is 0-255, unit is second, after set <time> will report the GPS information every the 
# seconds.
#
# +CGPSINFO: [<lat>],[<N/S>],[<log>],[<E/W>],[<date>],[<UTCtime>],[<alt>],[<speed>],[<course>] 
# OK 
class GpsInfo:
    def __init__(self):
        self.lat = 0
        self.ns = "N"
        self.lon = 0
        self.ew = "E"
        self.date = "0"
        self.utc = 0
        self.alt = 0
        self.speed = 0
        self.course = 0
        self.tm = 0

def get_gps_posn(spt='/dev/ttyUSB0', baud_rt=9600, val=1):
    ret = False
    gps_info = GpsInfo()
    try:
        set_sess="AT+CGPSINFO"
        sim_init_snd=bytearray(set_sess.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(sim_init_snd)
        line = ser.readline()
        line2 = ser.readline()
        ret = not str(line2).find("OK") == -1 
        g = line.split(":")  
        if not g[0].find("GPSINFO") == -1:		
            self.lat = g[1].split(",")[0].replace('[','').replace(']','').replace('<','').replace('>','')
            self.ns = g[1].split(",")[1].replace('[','').replace(']','').replace('<','').replace('>','')
            self.lon = g[1].split(",")[2].replace('[','').replace(']','').replace('<','').replace('>','')
            self.ew = g[1].split(",")[3].replace('[','').replace(']','').replace('<','').replace('>','')
            self.date = g[1].split(",")[4].replace('[','').replace(']','').replace('<','').replace('>','')
            self.utc = g[1].split(",")[5].replace('[','').replace(']','').replace('<','').replace('>','')
            self.alt = g[1].split(",")[6].replace('[','').replace(']','').replace('<','').replace('>','')
            self.speed = g[1].split(",")[7].replace('[','').replace(']','').replace('<','').replace('>','')
            self.course = g[1].split(",")[8].replace('[','').replace(']','').replace('<','').replace('>','')
            self.tm = g[1].split(",")[9].replace('[','').replace(']','').replace('<','').replace('>','')			
    except Exception as e:
        print(f'get gps position: {e}')
    finally:    
        ser.close()
    return ret,gps_info

# AT+CGPSCOLD Cold start GPS
#
def cold_start_gps(spt='/dev/ttyUSB0', baud_rt=9600, val=1):
    ret = False
    try:
        set_sess="AT+CGPSCOLD"
        sim_init_snd=bytearray(set_sess.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(sim_init_snd)
        line = ser.readline()
        ret = not str(line).find("OK") == -1    
    except Exception as e:
        print(f'gps cold: {e}')
    finally:    
        ser.close()
    return ret

# AT+CGPSHOT Hot start GPS
#
def hot_start_gps(spt='/dev/ttyUSB0', baud_rt=9600, val=1):
    ret = False
    try:
        set_sess="AT+CGPSHOT"
        sim_init_snd=bytearray(set_sess.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(sim_init_snd)
        line = ser.readline()
        ret = not str(line).find("OK") == -1    
    except Exception as e:
        print(f'gps hot: {e}')
    finally:    
        ser.close()
    return ret

# AT+CGPSAUTO Start/Stop GPS automatic
#
def auto_gps(a=1, spt='/dev/ttyUSB0', baud_rt=9600, val=1):
    ret = False
    try:
        set_sess=f"AT+CGPSAUTO={a}"
        sim_init_snd=bytearray(set_sess.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(sim_init_snd)
        line = ser.readline()
        ret = not str(line).find("OK") == -1    
    except Exception as e:
        print(f'gps auto: {e}')
    finally:    
        ser.close()
    return ret

# AT+CGPSNMEA configure NMEA sentance
#
# variable passed is as follows
# Range – 0 to 511 
# Each bit enables an NMEA sentence output as follows: 
# Bit 0 – GPGGA (global positioning system fix data) -- deafault
# Bit 1 – GPRMC (recommended minimum specific GPS/TRANSIT data) 
# Bit 2 – GPGSV (GPS satellites in view) 
# Bit 3 – GPGSA (GPS DOP and active satellites) 
# Bit 4 – GPVTG (track made good and ground speed) 
# Bit 5 –PQXFI (Global Positioning System Extended Fix Data.) 
# Bit 6 – GNGNS (fix data for GNSS receivers; output for GPS-only, GLONASS-only, hybrid 
# GLONASS+GPS fixes, or even AFLT fixes)  
# Bit 7 – GNGSA (DOP and GLONASS satellites; GPS+GLONASS or GLONASS-only fixes. 
# Contains DOP information for all active satellites, but other information is GLONASS-only)  
# Bit 8 – GLGSV (GLONASS satellites in view GLONASS fixes only)  
# Set the desired NMEA sentence bit(s). If multiple NMEA sentence formats are desired, “OR” the 
# desired bits together. 
#
def config_nmea_sentance(a=1, spt='/dev/ttyUSB0', baud_rt=9600, val=1):
    ret = False
    try:
        set_sess=f"AT+CGPSNMEA={a}"
        sim_init_snd=bytearray(set_sess.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(sim_init_snd)
        line = ser.readline()
        ret = not str(line).find("OK") == -1    
    except Exception as e:
        print(f'config nmea sentance: {e}')
    finally:    
        ser.close()
    return ret

# AT+CGPSINFOCFG Report GPS NMEA-0183 sentence
#
def get_nmea_sentance(spt='/dev/ttyUSB0', baud_rt=9600, val=1):
    try:
        set_sess="AT+CGPSINFOCFG"
        sim_init_snd=bytearray(set_sess.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(sim_init_snd)
        line = ser.readline()  
    except Exception as e:
        print(f'get nmea sentance: {e}')
    finally:    
        ser.close()
    return line

# AT+CGPSPMD Configure positioning mode
#
# Defined values 
# <mode> 
# Default is 0xFF7F 
# Each bit enables a supported positioning mode as follows: 
# Bit 0 – Standalone 
# Bit 1 – UP MS-based 
# Bit 2 – UP MS-assisted 
# Bit 3 – CP MS-based (2G) 
# Bit 4 – CP MS-assisted (2G) 
# Bit 5 – CP UE-based (3G) 
# Bit 6 – CP UE-assisted (3G) 
# Bit 7 – NOT USED 
# Bit 8 – UP MS-based (4G) 
# Bit 9 – UP MS-assisted(4G) 
# Bit 10 – CP MS-based (4G) 
# Bit 11 – CP MS-assisted (4G) 
# Set the desired mode sentence bit(s). If multiple modes are desired, “OR” the desired bits together. 
# Example, support standalone, UP MS-based and UP MS-assisted, set Binary value 0000 0111, is 7. 
#
def config_pos_mode(a=0xFF7F, spt='/dev/ttyUSB0', baud_rt=9600, val=1):
    ret = False
    try:
        set_sess=f"AT+CGPSPMD={a}"
        sim_init_snd=bytearray(set_sess.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(sim_init_snd)
        line = ser.readline()
        ret = not str(line).find("OK") == -1    
    except Exception as e:
        print(f'config position mode: {e}')
    finally:    
        ser.close()
    return ret

# AT+CGPSHOR Configure positioning desired accuracy
# This command is used to set the desired positioning accuracy threshold in meters.
# AT+CGPSHOR=<acc>
# Range – 0 to 1800000 
# Default value is 50
#
def config_pos_acc(a=2000, spt='/dev/ttyUSB0', baud_rt=9600, val=1):
    ret = False
    try:
        set_sess=f"AT+CGPSHOR={a}"
        sim_init_snd=bytearray(set_sess.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(sim_init_snd)
        line = ser.readline()
        ret = not str(line).find("OK") == -1    
    except Exception as e:
        print(f'config nmea sentance: {e}')
    finally:    
        ser.close()
    return ret

# AT+CVAUXV  Set voltage value of the pin named VREG_AUX1
# AT+CVAUXV=? +CVAUXV: (list of supported <voltage>s) 
# OK 
# Read Command Responses 
# AT+CVAUXV? +CVAUXV: <voltage> 
# OK 
# Write Command Responses 
# AT+CVAUXV=<voltage> 
#
# e.g. The unit is in mV. And the value must the 
# multiple of 50mv.
# AT+CVAUXV=2800 
#
def set_aux_mv(a=2800, spt='/dev/ttyUSB0', baud_rt=9600, val=1):
    ret = False
    try:
        set_sess=f"AT+CVAUXV={a}"
        sim_init_snd=bytearray(set_sess.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(sim_init_snd)
        line = ser.readline()
        ret = not str(line).find("OK") == -1    
    except Exception as e:
        print(f'set mv of VREG_AUX1: {e}')
    finally:    
        ser.close()
    return ret

def get_aux_mv(spt='/dev/ttyUSB0', baud_rt=9600, val=1):
    ret = -1
    try:
        set_sess="AT+CVAUXV?"
        sim_init_snd=bytearray(set_sess.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(sim_init_snd)
        line = ser.readline()
        ret = str(line).split(":")[1]    
    except Exception as e:
        print(f'get mv of VREG_AUX1: {e}')
    finally:    
        ser.close()
    return ret

# AT+CVAUXS  Set state of the pin named VREG_AUX1 
# =1 or 0
def set_aux_state(a=1, spt='/dev/ttyUSB0', baud_rt=9600, val=1):
    ret = False
    try:
        set_sess=f"AT+CVAUXS={a}"
        sim_init_snd=bytearray(set_sess.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(sim_init_snd)
        line = ser.readline()
        ret = not str(line).find("OK") == -1    
    except Exception as e:
        print(f'set mv of VREG_AUX1: {e}')
    finally:    
        ser.close()
    return ret

# AT+CADC  Read ADC value 
# AT+CADC=? +CADC: (range of supported <adc>s) 
# OK 
# Write Command Responses 
# AT+CADC=<adc> +CADC: <value> 
# OK 
# ERROR
#
# AT+CADC2=? +CADC2: (range of supported <adc>s) 
# OK 
# Write Command Responses 
# AT+CADC2=<adc> +CADC2: <value> 
# OK
#
def get_cadc(adcno=1, spt='/dev/ttyUSB0', baud_rt=9600, val=1):
    ret = -1
    try:
        if adcno == 1:
            set_sess="AT+CADC?"
        else:
            set_sess="AT+CADC2?"		
        sim_init_snd=bytearray(set_sess.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(sim_init_snd)
        line = ser.readline()
        ret = str(line).split(":")[1]    
    except Exception as e:
        print(f'get mv of VREG_AUX1: {e}')
    finally:    
        ser.close()
    return ret

# set changes the input range as follows 
# adc=0 raw 2=mV
#
def set_cadc(adcno=1, adc=2, spt='/dev/ttyUSB0', baud_rt=9600, val=1):
    ret = False
    try:
        if adcno == 1:
            set_sess=f"AT+CADC={adc}"
        else:
            set_sess=f"AT+CADC2={adc}"		
        sim_init_snd=bytearray(set_sess.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(sim_init_snd)
        line = ser.readline()
        ret = not str(line).find("OK") == -1    
    except Exception as e:
        print(f'set mv of VREG_AUX1: {e}')
    finally:    
        ser.close()
    return ret

# AT+CVALARM  Low and high voltage Alarm  =1 enable
#
def set_alarm(a=1, spt='/dev/ttyUSB0', baud_rt=9600, val=1):
    ret = False
    try:
        set_sess=f"AT+CVALARM={a}"		
        sim_init_snd=bytearray(set_sess.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(sim_init_snd)
        line = ser.readline()
        ret = not str(line).find("OK") == -1    
    except Exception as e:
        print(f'set mv of VREG_AUX1: {e}')
    finally:    
        ser.close()
    return ret

# AT+CMICGAIN =<value> Change mic volume 
# 0-8 8=max volume
#
def set_vol(a=8, spt='/dev/ttyUSB0', baud_rt=9600, val=1):
    ret = False
    try:
        set_sess=f"AT+CMICGAIN={a}"		
        sim_init_snd=bytearray(set_sess.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(sim_init_snd)
        line = ser.readline()
        ret = not str(line).find("OK") == -1    
    except Exception as e:
        print(f'set mv of VREG_AUX1: {e}')
    finally:    
        ser.close()
    return ret

# AT+CGREG GPRS network registration status
#
def get_reg(spt='/dev/ttyUSB0', baud_rt=9600, val=1):
    try:
        set_sess="AT+CGREG"		
        sim_init_snd=bytearray(set_sess.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(sim_init_snd)
        line = ser.readline()    
    except Exception as e:
        print(f'set mv of VREG_AUX1: {e}')
    finally:    
        ser.close()
    return line

# send a text message fist set text mode from general gsm library e.g. text mode (AT+CMGF=1):
# da = Destination-Address
#
def send_sms_simcom(da, spt='/dev/ttyUSB0', baud_rt=9600, pno=”+33146290800”, msg="time to start simcomm tests"):
    ret = False
    try:
        smsm=f"AT+CMGSEX={da}"
        snd_string=bytearray(smsm.encode('utf-8'))
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(snd_string)
        line = ser.readline()
        if not str(line).decode('utf-8).find("CMGS") == -1:
            msg=f"{msg}"
            snd_string=bytearray(msg.encode('utf-8'))
            ser = serial.Serial(spt, baud_rt, timeout=0.1)
            ser.write(snd_string)
            esc_end = [ 0x1B ]
            ctl_z = [ 0x1A, 'Z' ]
            eof = [ 0x05 ]
            ser.write(bytearray(esc_end))                        # ESC completes the message
            #ser.write(bytearray(ctl_z))                         CTL-Z to end message
            line = ser.readline()
            ret = not str(line).find("OK") == -1
    except Exception as e:
        print(f'send_sms_simcomm: {e}')
    finally:    
        ser.close()
    return ret
