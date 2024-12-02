#!/usr/bin/env python
#
# Example of functions to provide prioritised set of 5 queues
#
# e.g. telephone system, equipment booking system, filter backwash queue
#
# This example shows filters (units) being queued for a wash operation with common blower and wash pump
#
import pigpio

# gpio numbers mapping
Q1_INH = 27
Q2_INH = 5
Q3_INH = 6
Q4_INH = 17                       
Q5_INH = 27
BW_PMPS_FAIL = 22                       
WW_PUMPS_FAIL = 23
WW_TANK_LOW = 24
WW_PUMP_ON = 25
AB_BLOW_ON = 26
ESD_PIN = 12
WW_TANK_RF = 20
SP2 = 21
ESD_FLG=0

# ================================================
# modbus tcp and serial server for filter queue 
# sudo pip install pymodbus --ignore-installed six
# =================================================
import asyncio
import pymodbus
from pymodbus.server import StartAsyncSerialServer
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext

from pymodbus.server.sync import StartTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.transaction import ModbusRtuFramer
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.payload import BinaryPayloadDecoder

import logging

logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.DEBUG)

MOD_SER_PORT = "/dev/ttyS1"                        # port for connection on serial
MY_SLAVE_RTU1 = 0x11
MY_SLAVE_RTU2 = 0x12
BR = 19200
MY_TCP_SLAVE="localhost"
TCP_SLAVE_PORT=502

# global data written each way from modbus slave <-> python appl. prog
INH_PLC1 = []                                     # inhib queue from plc1
INH_PLC2 = []                                     # inhib queue from plc2
CHAN_LVL_PLC2 = 0
INH_HMI = []                                      # inhibit queue from hmi
NO_OF_UNITS = 6
DP_TRIG = [ 12, 3, 4, 12, 14, 30 ]                # data read from application to HMI (tcp slave)
RUN_TIMES = [ 82, 33, 4, 12, 1, 23 ]
ALUM_TR = [ 2.1, 3, 4, 1.2, 4, 3 ]
TURBID_TR = [ 29, 53, 64, 82.7, 54, 0.13 ]
MANU_TRIG = [ 1, 0, 0, 1, 1, 0 ]
FLOAT32ARR = []                                   # data from plc2
NO_OF_PLC2_FLOATS = 2                             # number of registers to read as above floats
DO_WRITE_SLAVE=0
QUE_INH=0                                         # queue inhibit option active from HMI

NO_IR_INTS = 30                                   # data area in the modbus tcp slave
NO_IR_FLOATS = 30

# =================================================
#  modbus serial clients (slaves) which are novus
# =================================================
from pymodbus.client.sync import ModbusSerialClient as ModbusClient_Novus

#---------- this is for the NOVUS DigiRail-2A universal Analog Input Module ----------#
NOVUS_SER_PRT="/dev/ttyS0"

# =================================================
#  modbus tcp client (slave) which is procon PLC101
# =================================================
from pymodbus.client.sync import ModbusTcpClient as ModbusClient_Procon

import modbus_novus_procon

# trigger request lists
man_req = []
dp_req = []
service_time_req = []
turb_req = []
alum_req = []

# prioritized queues
p1_q = []
p2_q = []
p3_q = []
p4_q = []
p5_q = []

# shared resource queue
ab_q = []
ww_q = []

# queue priority per item e.g. backwashing a filter on these triggers
queue_nos = [ 1, 2, 3, 4, 5 ]
# list the triggers and their priority in the list, you can easily modify priorities of triggers here
MANUAL = 0
DIFF_PRESS = 1
TIME_EXPIRED = 2
TURBID = 3
ALUM = 4
# so you say
# priority_man = queue_no[MANUAL]

# put an item into a Q
def put_on_Q(list_nm, filter_no):
    list_nm.append(filter_no)

# get an item from the front of the queue FIFO and return the queue without it
def get_item_from_top_of_Q(list_nm):
    if len(list_nm) == 0:
        fn = -1
    else:
        fn = list_nm[0]
        list_nm = list_nm[1:]                          # pop the top number from the stack (list) and return the result
    return fn, list_nm

# get an item from the end of a queue LIFO and modify queue without it
def get_item_from_end_of_Q(list_nm):
    if len(list_nm) == 0:
        fn = -1
    else:
        fn = list_nm.pop()                             # pop the last number from the stack (list) no need to return
    return fn

# get an item from the end of a queue LIFO and dont change queue
def get_item_from_end_of_Q_keepQ(q_time):
    return q_time[len(q_time)-1]
    
# find an item in the list (queue) and delete it from it	
def find_delete_from_Q(qlist, numb):
    new_q = []
    if not str(qlist).find(str(numb)) == -1:         # number is in the queue
        i = 0
        for i in range(0,len(filt_q_time)):
            if int(numb) == int(filt_q_time[i]):
                print(filt_q_time[i])
                break
            new_q.append(filt_q_time[i])
        for z in range(i+1,len(filt_q_time)):
            print(z)
            new_q.append(filt_q_time[z])
    else:
	    new_q = qlist
    return new_q

# find if an item is in the list specified	
def find_in_Q(qlist, numb):
    new_q = True
    if str(qlist).find(str(numb)) == -1:         # number is not in the queue
	    new_q = False
    return new_q

# check analog signal against a setpoint and place id in request list
def ani_signal_chk(s, hl, id. reqlist):
    if float(s) > hl:
        reqlist.append(id)

# check digital signal against a setpoint and place id in request list
def di_signal_chk(s, state, id. reqlist):
    if bool(s) == state:
        reqlist.append(id)
        
# service the priority requests for the queue(s) p1, p2, p3, p4, p5
#
def prio5_req(p5, p4, p3, p2, p1, id):
    ret = False
    if find_in_Q(p1, id) == True:               # already in a higher queue then return an error
        return ret
    if find_in_Q(p2, id) == True:
        return ret
    if find_in_Q(p3, id) == True:
        return ret
    if find_in_Q(p4, id) == True:
        return ret
    if find_in_Q(p5, id) == False:
        put_on_Q(p5, id)
		ret = True
    return ret
	
def prio4_req(p5, p4, p3, p2, p1, id):
    if find_in_Q(p1, id) == True:               # already in a higher queue then return an error
        return [-1]
    if find_in_Q(p2, id) == True:
        return [-1]
    if find_in_Q(p3, id) == True:
        return [-1]
    if find_in_Q(p4, id) == False:
        put_on_Q(p4, id)
    if find_in_Q(p5, id) == True:
        new_p5 = find_delete_from_Q(p5, id)
    return new_p5
		
def prio3_req(p5, p4, p3, p2, p1, id):
    if find_in_Q(p1, id) == True:
        return [-1], [-1]
    if find_in_Q(p2, id) == True:
        return [-1], [-1]
    if find_in_Q(p4, id) == True:
        new_p4 = find_delete_from_Q(p4, id)
    if find_in_Q(p5, id) == True:
        new_p5 = find_delete_from_Q(p5, id)
    if find_in_Q(p3, id) == False:
        put_on_Q(p3, id)
    return new_p5, new_p4
	
def prio2_req(p5, p4, p3, p2, p1, id):
    if find_in_Q(p1, id) == True:
        return [-1], [-1], [-1]
    if find_in_Q(p5, id) == True:
        new_p5 = find_delete_from_Q(p5, id)
    if find_in_Q(p4, id) == True:
        new_p4 = find_delete_from_Q(p4, id)
    if find_in_Q(p3, id) == True:
        new_p3 = find_delete_from_Q(p3, id)
    if find_in_Q(p2, id) == False:
        put_on_Q(p2, id)
    return new_p5, new_p4, new_p3
		
def prio1_req(p5, p4, p3, p2, p1, id):
    if find_in_Q(p5, id) == True:
        new_p5 = find_delete_from_Q(p5, id)
    if find_in_Q(p4, id) == True:
        new_p4 = find_delete_from_Q(p4, id)
    if find_in_Q(p3, id) == True:
        new_p3 = find_delete_from_Q(p3, id)
    if find_in_Q(p2, id) == True:
        new_p2 = find_delete_from_Q(p2, id)
    if find_in_Q(p1, id) == False:
        put_on_Q(p1, id)
    return new_p5, new_p4, new_p3, new_p2

# program globals for testing they actually shall be read over comms from the remote resources
#
Free=1                                  # define the resource states
Booked=2
RESOURCE=Free                           # resource state when Booked it is freed by the resource when it completes its sequence or action
store_tm = time.time()
INHIB=False                             # comes from logic and set to True to ignore any queue triggers
INHIB_DOFF_TIME=1200                    # when queue not inhibited delay it off by this long first
INHIB_ACTIVE=True                       # when True we will inhibit items going to the queue when a unit is washing and for INHIB_DOFF_TIME thereafter 
UNAVAIL_COMMON_RES=False                # set True if a common resource is failed 
UNIT_NOS=[]                             # units that are perfroming the operation
MAX_NO_OPS=2                            # only 2 unit operations at one time
abon = 0                                # command to start airblower
wwon = 0                                # command to start wash pump
abon_unit = 0                           # store unit number (filter) that currently booked the airblower
wwon_unit = 0                           # store unit number (filter) that currently booked the wash pump 
WASH_TANK_REFIL_TM=1800                 # wash tank refill time after use of the wash pumps
AB_TIMEOUT=3000
WW_TIMEOUT=3000

RNG_DP = 5.0
RNG_TUR = 10.0
   
# process the requested information and update the prioritised queues
#   
def process_request_info(req_list, typ):
    for m in req_list:
        if 1 == queue_nos[typ]:
            p5_q, p4_q, p3_q, p2_q = prio1_req(p5_q, p4_q, p3_q, p2_q, p1_q, int(m))  
        elif 2 == queue_nos[typ]:
            new_p5, new_p4, new_p3 = prio2_req(p5_q, p4_q, p3_q, p2_q, p1_q, int(m)) 
            if not new_p5 == -1:
                p5_q, p4_q, p3_q = new_p5, new_p4, new_p3
        elif 3 == queue_nos[typ]:
            new_p5, new_p4 = prio3_req(p5_q, p4_q, p3_q, p2_q, p1_q, int(m)) 
            if not new_p5 == -1:
                p5_q, p4_q = new_p5, new_p4
        elif 4 == queue_nos[typ]:
            new_p5 = prio4_req(p5_q, p4_q, p3_q, p2_q, p1_q, int(m)) 
            if not new_p5 == -1:
                p5_q = new_p5
        elif 5 == queue_nos[typ]:
            prio5_req(p5_q, p4_q, p3_q, p2_q, p1_q, int(m))

# this shall send the book request to the unit over a socket
#
def send_item_book_req(item):
    print("sending the book request to Unit No: ",item)

def read_gpio(self, pi, pin):
    return pi.read(pin) 

def init_io_input(self, pi, gpio_no):
    pi.set_mode(gpio_no, pigpio.INPUT)
    pi.set_pull_up_down(gpio_no, pigpio.PUD_UP) 

def init_io_output(self, pi, gpio_no):
    pi.set_mode(gpio_no, pigpio.OUTPUT)

def range_novus(raw, ur):
    return (float(raw) / 62000.0) * ur

# ------------- modbus parameters ---------------      #
# address   : modbus register to start with
# values    : value to set or list of values
# unit      : the slave number when on serial
# count     : number of registers to perform action upon 

def write_holding_reg(client, reg=1, value=0, method="tcp", unit=1):
    print("Write to a holding register and read back")
    if method == "tcp":
        rq = client.write_register(reg, value)
    else :
        rq = client.write_register(reg, value, unit=unit)	
    print(rq)
    return rq
	
def read_holding_reg(client, reg=1, method="tcp", unit=1):
    print("Read holding register")
    if method == "tcp":
        rr = client.read_holding_registers(address=reg, count=1)
    else :		
        rr = client.read_holding_registers(address=reg, count=1, unit=unit)
    print(rr.registers)
    return rr.registers
	
def write_holding_regs(client, reg=1, values=[0,2,3], method="tcp", unit=1):
    print("Write to multiple holding registers and read back")
    if method == "tcp":
        request = client.write_registers(address=reg, values=values1)
    else :
        request = client.write_registers(address=reg, values=values1, unit= unit)
    print(request)
    return request
	
def read_holding_regs(client, reg=1, method="tcp", cc=3, unit=1):
    print("Read holding registers")
    if method == "tcp":
        rr = client.read_holding_registers(address=reg, count=cc)
    else :		
        rr = client.read_holding_registers(address=reg, count=cc, unit=unit)
    print(rr.registers)	
    return rr.registers
	
def read_input_regs(client, reg=1, method="tcp", cc=3, unit=1):
    print("Read input registers")
    if method == "tcp":
        rr = client.read_input_registers(reg, cc)
    else :		
        rr = client.read_input_registers(address=reg, count=cc, unit=unit)
    print(rr.registers)		
    return rr.registers

def read_disc_inputs(client, reg=0, num=3, method="tcp", unit=1):
    print("Read discrete inputs")
    if method == "tcp":   
        rr = client.read_discrete_inputs(req, num)
    else:
        rr = client.read_discrete_inputs(req, num, unit)    
    print(rr.bits)
    return rr.bits
    
def write_coil(client, reg=0, v=True, method="tcp", unit=1):
    print("Write to a coil and read back")
    if method == "tcp":   
        rq = client.write_coil(reg, v)
    else:
        rq = client.write_coil(reg, v, unit) 
    print(rq)
    return rq

def write_coils(client, reg=0, v=[True]*9, method="tcp", unit=1):
    print("Write to multiple coils and read back")
    if method == "tcp":   
        rq = client.write_coils(reg, v)
    else:
        rq = client.write_coils(reg, v, unit) 
    print(rq)
    return rq

def read_coils(client, reg=0, cc=9, method="tcp", unit=1):
    print("Reading multiple/Single coils and read back")
    if method == "tcp":   
        req = client.read_coils(reg, cc)
    else:
        req = client.read_coils(reg, cc, unit) 
    print(req.bits)
    return req.bits

def four_list_or(a, b, c, d):
    r = [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ]
    if len(r) < len(a):
        print("max list size is length : ",len(r))
        return
    for i in range(0, len(a)):                        # or the elements in the list idividually
        r[i] = a[i] | b[i] | c[i] | d[i]
    for rr in range(0, len(r)-len(a)):                # trim back the list to sizes ored
        b = r.pop() 
        
# variable to decide it the queue is latching (i.e. once seen is enough) = True otherwise if condition trigger lost queue looses it
LATCH_QUE = False
     
async def main_filter_queue_task(): 

    if LATCH_QUE == True:
        # this option creates latched queue once seen it remains
        man_req = [ ]
        dp_req = [ ]
        service_time_req = [ ]
        turb_req = [ ]
        alum_req = [ ]
        remove_list = [ ] 
        
    # the dp and tubidity for each unit are read over modbus rs495 serial line to the Novus Modbus i/o
    # set up the novus on serial
    client_novus = ModbusClient_Novus(method = "rtu", port=NOVUS_SER_PRT, stopbits = 1, bytesize = 8, parity = 'E', baudrate= 115200, timeout= 1)
    client_novus.connect()
    mthd = "ser"

    # example we have 6 units then we have 12 novus 2 per unit giving us 4 AIN
    num_of_units = NO_OF_UNITS
    
    # set up the novus module -- write the channel type
    # NOVS_DR2A_TYPE1 = NDR2A_4to20mA with dp 4-20 / turb_req 0-20
    # NOVS_DR2A_TYPE1 = NDR2A_4to20mA with alum    
    for ser_addr in range(1, (num_of_units*2)+1):
        def_addr_novus_d2a = ser_addr                                                                             # default is modbus_novus_procon.NOVS_DR2A_DEF_ADDR 
        input_types = [modbus_novus_procon.NDR2A_4to20mA, modbus_novus_procon.NDR2A_0to20mA]
   
        # writing as single registers
        w = write_holding_reg(client_novus, modbus_novus_procon.NOVS_DR2A_TYPE1, value=input_types[0], method=mthd, unit=def_addr_novus_d2a)
        w = write_holding_reg(client_novus, modbus_novus_procon.NOVS_DR2A_TYPE2, value=input_types[1], method=mthd, unit=def_addr_novus_d2a)

        ranges = [modbus_novus_procon.NDR2A_lo4to20mA, modbus_novus_procon.NDR2A_ul4to20mA, modbus_novus_procon.NDR2A_lo0to20mA, modbus_novus_procon.NDR2A_ul0to20mA]   
        # writing as single registers
        w = write_holding_reg(client_novus, modbus_novus_procon.NOVS_DR2A_CH1_LOW, value=ranges[0], method=mthd, unit=def_addr_novus_d2a)
        w = write_holding_reg(client_novus, modbus_novus_procon.NOVS_DR2A_CH1_HIGH, value=ranges[1], method=mthd, unit=def_addr_novus_d2a)
        w = write_holding_reg(client_novus, modbus_novus_procon.NOVS_DR2A_CH2_LOW, value=ranges[2], method=mthd, unit=def_addr_novus_d2a)
        w = write_holding_reg(client_novus, modbus_novus_procon.NOVS_DR2A_CH2_HIGH, value=ranges[3], method=mthd, unit=def_addr_novus_d2a)
        # set input filter time
        w = write_holding_reg(client_novus, modbus_novus_procon.NOVS_DR2A_FILTER, value=10.0, method=mthd, unit=def_addr_novus_d2a)

    # set up the procon on tcp/ip 
    # %%%%%%% Procon Electronic Modbus TCP 16DI logic and counter up to 1KHz %%%%%
    #    
    PROCON_TCP_PORT=5020
    client_pro = ModbusClient_Procon(modbus_novus_procon.PROCON_TCP16DI_DEF_IP, port=PROCON_TCP_PORT)
    client_pro.connect()    
    mthd_pro = "tcp"
    # set a filter on the inputs
    filter_ms = 10
    w = write_holding_reg(client_pro, modbus_novus_procon.PROCON_PT16DI_Filter, filter_ms, method=mthd_pro)
           
    # initailise the gpio on board the pi
    pi = pigpio.pi()
    inhibit_pins = [ Q1_INH, Q2_INH, Q3_INH, Q4_INH, Q5_INH ]
    for ihp in inhibit_pins:
        init_io_input(pi, int(ihp))
    other_gpio = [ BW_PMPS_FAIL, WW_PUMPS_FAIL, WW_TANK_LOW ]
    for g in other_gpio:
        init_io_input(pi, int(g))    
    out_gpio = [ WW_PUMP_ON, AB_BLOW_ON ]
    for o in out_gpio:
        init_io_output(pi, init(o))
    init_io_input(pi, WW_TANK_RF)

    # define an ESD when pressed puts all filters out of service
    def ESD_cb(gpio, level, tick):
        print (f'# {gpio=} : {level=} : {tick/(1000*1000)=}s')
        if level == 1:
            for a_offset in range(1,num_of_units+1):
                w = write_holding_reg(client_pro, reg=modbus_novus_procon.PROCON_PL101_BBRAM_START+30+a_offset, value=99, method="tcp")          # out of service 
            ESD_FLG = 1
            pi.write(WW_PUMP_ON, 0)                                  # imediate stop to the pumps on ESD
            pi.write(AB_BLOW_ON, 0)
        elif level == 0:
            ESD_FLG = 0
            
    # initialise the ESD pin and callback function for it
    def init_esd(pi, esd_gpio):
        pi.set_mode(esd_gpio, pigpio.INPUT)                          # set-up esd with a callback function
        pi.set_pull_up_down(esd_gpio,pigpio.PUD_UP)
        cb = pi.callback(esd_gpio, pigpio.EITHER_EDGE, ESD_cb)  # call back function will be called on either edge
    init_esd(pi, ESD_PIN)
    
    # timers for sequwnce timeouts if they loose comms with the unit during the pump operation
    ab_pm_tm = time.time() 
    ww_pm_tm = time.time()

    unit_state_prev = []

    # wash water pump controls 
    db_tick_wwp = 0                                                                  # timer interacting with wash pump run signal 
    latch1 = False 
                
    while True:
        try:    
            # inhibit queue requests while the resource is booked and INHIB_DOFF_TIME (s) thereafter
            INHIB_ACTIVE = (QUE_INH == 1)
            if RESOURCE == Booked and INHIB_ACTIVE == True:
                INHIB = True
                store_tm = time.time()
            elif INHIB == True and INHIB_ACTIVE == True:
                if (time.time() - store_tm) > INHIB_DOFF_TIME:
                    INHIB = False
            elif INHIB_ACTIVE == False:
                INHIB = False     

            # check to see if we have any inhibits that means we have no available common resources
            UNAVAIL_COMMON_RES=False
            for g in other_gpio:
                if not read_gpio(pi, int(g)) :                                       # if any are low state then make the resource unavailable
                    UNAVAIL_COMMON_RES = True
            wash_tank_low = not read_gpio(pi, WW_TANK_LOW)                           # 0 = low so invert it
            wash_tank_refil = read_gpio(pi, WW_TANK_RF)                              # 1 - refil from conductivity probe
            
            # update the queues if we are not inhibited e.g. interference becasue of other washes going on
            if INHIB == False:

                # define the triggers from each weighted output for testing - uncomment for testing   
                #man_req = [ 1, 34, 45, 11 ]
                #dp_req = [ 2, 12, 1 ]
                #service_time_req = [ 2, 44, 65, 76, 67 ]
                #turb_req = [ 77. 92 ]
                #alum_req = [ 24, 1, 54, 23, 75, 47 ]
                #remove_list = [ 1, 3 ]

                if LATCH_QUE == False:        
                    # this option creates a new queue every time so the queue is as per the requests and if they go out of trigger they are removed
                    man_req = [ ]
                    dp_req = [ ]
                    service_time_req = [ ]
                    turb_req = [ ]
                    alum_req = [ ]
                    remove_list = [ ]  
        
                # create the trigger lists (this shall be created by the realtime data read from each remote unit) as shown below but use this for testing the queue
                # again uncomment for testing and comment out the collection section below
                #man_trig = [ 1, 0, 0, 0, 1, 1 ]
                man_spt = True 
                #dp_trig = [ 1.1, 0.1, 0.9, 0, 2.1, 9.81 ]
                dp_spt = 1.0
                #service_time_trig = [ 2, 44, 65, 76, 67, 89 ]
                st_spt = 45
                #turb_trig = [ 2, 4, 65, 6, 7, 89 ]
                turb_spt = 47
                #alum_trig = [ 0, 0, 1.1, 0, 0, 0 ]
                alum_spt = 1.0        

                # read live data from the Novus units and compile the trigger lists for dp, turb, alum
                # if you want to inhibit each individual reading you must disconenct the wire at present
                #
                dp_trig = []
                turb_trig = []
                for ser_addr in range(1, num_of_units+1):
                    def_addr_novus_d2a = ser_addr                                                                             # default is modbus_novus_procon.NOVS_DR2A_DEF_ADDR 
                    try:                
                        results = read_input_regs(client_novus, modbus_novus_procon.NOVS_DR2A_PV_CH1, mthd, 4, def_addr_novus_d2a)
                        print(results)  
                        dp_trig.append(range_novus(results[0],RNG_DP)
                        turb_trig.append(range_novus(results[1],RNG_TUR)
                    except (ValueError, Exception) as e:
                        print(e)
                        steps_in_error = 0
                        # attempt re-connection once but if already closed just ignore that error
                        while (steps_in_error < 2):
                            if (steps_in_error == 0):
                                try:
                                    # close the connections
                                    client_novus.close() 
                                    steps_in_error = 1
                                except (ValueError, Exception) as e:
                                    # tolerate this error and go into the next stage step which is to try to re-establish communication
                                    print(e)   
                                    steps_in_error = 1
                            elif (steps_in_error == 1):   
                                try:                
                                    # try re-open connections
                                    client_novus = ModbusClient_Novus(method = "rtu", port=NOVUS_SER_PRT,stopbits = 1, bytesize = 8, parity = 'E', baudrate= 115200, timeout= 1)
                                    client_novus.connect() 
                                    steps_in_error = 2
                                    results = read_input_regs(client_novus, modbus_novus_procon.NOVS_DR2A_PV_CH1, mthd, 4, def_addr_novus_d2a)
                                    print(results)  
                                    dp_trig.append(range_novus(results[0],RNG_DP)
                                    turb_trig.append(range_novus(results[1],RNG_TUR)
                                except (ValueError, Exception) as e:
                                    # error should then drop out loop and close connections
                                    print(e) 
                                    dp_trig.append(-1)
                                    turb_trig.append(-1) 
                alum_trig = []                                                                                 
                for ser_addr in range(num_of_units+1, (num_of_units*2)+1):
                    def_addr_novus_d2a = ser_addr                                                                             # default is modbus_novus_procon.NOVS_DR2A_DEF_ADDR 
                    try:                
                        results = read_input_regs(client_novus, modbus_novus_procon.NOVS_DR2A_PV_CH1, mthd, 4, def_addr_novus_d2a)
                        print(results)  
                        alum_trig.append(range_novus(results[0],RNG_AL)
                    except (ValueError, Exception) as e:
                        print(e)
                        steps_in_error = 0
                        # attempt re-connection once but if already closed just ignore that error
                        while (steps_in_error < 2):
                            if (steps_in_error == 0):
                                try:
                                    # close the connections
                                    client_novus.close() 
                                    steps_in_error = 1
                                except (ValueError, Exception) as e:
                                    # tolerate this error and go into the next stage step which is to try to re-establish communication
                                    print(e)   
                                    steps_in_error = 1
                            elif (steps_in_error == 1):   
                                try:                
                                    # try re-open connections
                                    client_novus = ModbusClient_Novus(method = "rtu", port=NOVUS_SER_PRT,stopbits = 1, bytesize = 8, parity = 'E', baudrate= 115200, timeout= 1)
                                    client_novus.connect() 
                                    steps_in_error = 2
                                    results = read_input_regs(client_novus, modbus_novus_procon.NOVS_DR2A_PV_CH1, mthd, 4, def_addr_novus_d2a)
                                    print(results)  
                                    alum_trig.append(range_novus(results[0],RNG_AL)
                                except (ValueError, Exception) as e:
                                    # error should then drop out loop and close connections
                                    print(e) 
                                    alum_trig.append(-1)

                # Read Time in Service from Procon PLC user BBRAM stored saved and often
                # M1200 – M1219 start 41208 1207 end 41220 1219
                service_time_trig = read_holding_regs(client_pro, reg=modbus_novus_procon.PROCON_PL101_BBRAM_START, method="tcp", cc=num_of_units)
                
                # read the manual initiate switches
                di8_1_list = read_disc_inputs(client_pro, reg=modbus_novus_procon.PROCON_PT16DI_DIStart, num=8, method=mthd_pro)        
                # last 2 are unused
                b = di8_1_list.pop()
                b = di8_1_list.pop()
                man_trig = di8_1_list
        
                for i, man in enumerate(man_trig):
                    di_signal_chk(man, man_spt, i+1. man_req) 
                for i, dp in enumerate(dp_trig):
                    ani_signal_chk(dp, dp_spt, i+1. dp_req) 
                for i, ser in enumerate(service_time_trig):
                    ani_signal_chk(ser, st_spt, i+1. service_time_req) 
                for i, tu in enumerate(turb_trig):
                    ani_signal_chk(tu, turb_spt, i+1. alum_req)    
                for i, al in enumerate(alum_trig):
                    ani_signal_chk(al, alum_spt, i+1. turb_req)
                          
                # process the trigger collection data
                process_request_info(man_req, MANUAL)
                process_request_info(dp_req, DIFF_PRESS)
                process_request_info(service_time_req, TIME_EXPIRED)
                process_request_info(turb_req, TURBID)   
                process_request_info(alum_req, ALUM)

                # example of a removal request which could be made to stop that single resource from the queue
                # remove_trig = [ 1, 0, 0, 1, 0, 0 ]
                remove_trig1 = read_holding_regs(client_pro, reg=170, method="tcp", cc=num_of_units, unit=1)
                remove_trig = four_list_or(remove_trig1, INH_HMI, INH_PLC1, INH_PLC2)
                remove_spt = True
                for i, rem in enumerate(remove_trig):
                    di_signal_chk(rem, remove_spt, i+1. remove_list)        

                # remove from all queues any units listed in the remove_list
                for r in remove_list:
                    p1_q = find_delete_from_Q(p1_q, int(r))
                    p2_q = find_delete_from_Q(p2_q, int(r))
                    p3_q = find_delete_from_Q(p3_q, int(r))
                    p4_q = find_delete_from_Q(p4_q, int(r))
                    p5_q = find_delete_from_Q(p5_q, int(r)) 

                # read from the remote units (filter) the actual sequence states - TBD over modbus
                # unit_states = [ 0, 0, 10, 2, 0, 0 ]
                # read from user ram stored not saved  M161 – M199 start 40162 =161, so we start 40163
                # unit_states = read_holding_regs(client_pro, reg=162, method="tcp", cc=num_of_units, unit=1)
                #
                # Read Time in Service from Procon PLC user BBRAM stored saved and often
                # M1200 – M1219 start 41208 1207 end 41220 1219
                unit_states = read_holding_regs(client_pro, reg=modbus_novus_procon.PROCON_PL101_BBRAM_START+30+item, method="tcp", cc=num_of_units) 

                # step 4 == book air blower
                # 5 == opened valves so start blower
                # 6 == stop air blower and unbook

                # step 10 == book ww pump
                # 11 == opened valves so start ww pump
                # 12 == stop ww pump and unbook
                # 99 == filter out of service
                # 0 == normal filtering
                
                # check if we have a free resource
                number_of_operations = 0
                air_blowing = 0
                washing = 0
                for i in range(0,len(unit_states)):
                    if not (int(unit_states[i]) == 0) and not (int(unit_states[i]) == 99):       # not doing operation or out of service
                        number_of_operations += 1
                
                    if unit_states[int(i)] == 4:                                                 # request book air blower resource
                        put_on_Q(ab_q, int(i)+1)                                                 # make air blower request queue
                    if unit_states[int(i)] > 4 and unit_states[int(i)] < 7:                      # something is using the air blower resource
                        airblowing += 1  

                    if unit_states[int(i)] == 10:                                                # request book wash pump resource
                        put_on_Q(ww_q, int(i)+1)                                                 # make wash water pump request queue
                    if unit_states[int(i)] > 10 and unit_states[int(i)] < 13:                    # something is using the wash pump resource
                        washing += 1 
                        
                    if not (unit_state_prev[int(i)] == 0) and unit_states[int(i)] == 0:          # went from washing to normal state
                        find_delete_from_Q(unit_states, int(i)+1)                                # delete from the washing (in_operation) queue                           
                unit_state_prev = unit_states
                
                # handle the single resource (air blower)  
                if unit_states[abon_unit] == 5 and abon == 1:                                    # wait for airblower valves to open
                    abon = 2                                                                     # start to the blower
                    ab_pm_tm = time.time() 
                elif ((unit_states[abon_unit] >= 6) or ((time.time() - ab_pm_tm) > AB_TIMEOUT)) and abon == 2:     # stop air blower step request and unbook the shared resource 
                    abon = 0
                    # send_data(item, resource_booked_ab_reg, 0) 
                    w = write_holding_reg(client_pro, reg=modbus_novus_procon.PROCON_PL101_BBRAM_START+10+abon_unit+1, value=0, method="tcp")
                    abon_unit = 0
                elif len(ab_q) > 0 and airblowing == 0:                                          # we have something in the queue and we are permitted to start
                    item, ab_q = get_item_from_top_of_Q(ab_q)        
                    # send_data(item, resource_booked_ab_reg, 1) 
                    w = write_holding_reg(client_pro, reg=modbus_novus_procon.PROCON_PL101_BBRAM_START+10+item, value=1, method="tcp")                    
                    abon_unit=item-1                                                             # set air blower resource to the unit number which has aquired it
                    abon = 1                                                                     # set start to the air blower routine to wait for valves open

                if abon == 2:
                    pi.write(AB_BLOW_ON, (1 and not ESD_FLG))                                        # gpio to pump
                else:
                    pi.write(AB_BLOW_ON, 0)
            
                # handle the single resource (wash pump) 
                if unit_states[wwon_unit] == 11 and wwon == 1:                                   # wait for wash valves to open
                    wwon = 2                                                                     # start to the pump 
                    ww_pm_tm = time.time()            
                elif ((unit_states[wwon_unit] >= 12) or ((time.time() - ww_pm_tm) > WW_TIMEOUT)) and wwon == 2: # stop wash pump step request and unbook the shared resource 
                    wwon = 3                                                                     # stop the pump and wait for the timer to expire before allowing the resource to re-book
                    # send_data(item, resource_booked_ww_reg, 0) 
                    w = write_holding_reg(client_pro, reg=modbus_novus_procon.PROCON_PL101_BBRAM_START+20+wwon_unit+1, value=0, method="tcp")
                    wwon_unit = 0
                    ww_st_tm = time.time()
                elif (len(ww_q) > 0) and (washing == 0) and not (wwon == 3):                     # we have something in the queue and we are permitted to start
                    item, ww_q = get_item_from_top_of_Q(ww_q)        
                    # send_data(item, resource_booked_ww_reg, 1) 
                    w = write_holding_reg(client_pro, reg=modbus_novus_procon.PROCON_PL101_BBRAM_START+20+item, value=1, method="tcp")                    
                    wwon_unit=item-1                                                             # set air blower resource to the unit number which has aquired it
                    wwon = 1 
                elif wwon == 3 and (time.time() - ww_st_tm) >= WASH_TANK_REFIL_TM:               # wait for tank re-fill time before releasing the resource 
                    wwon = 0

                # wash water pump running controls                
                if wwon == 2:                                                                    # step 2 means run the pump
                    if (wash_tank_low):
                        adj_pmp_tm = time.time() - ww_pm_tm                                      # if laready low keep adjusting pump start
                        ww_pm_tm = time.time() - adj_pmp_tm
                        db_tick_wwp += 1
                        if db_tick_wwp > 10000:                                                  # low for long enough (hysterysis time)                       
                            pi.write(WW_PIMP_ON, 0)
                    else:
                        pi.write(WW_PUMP_ON, (1 and not ESD_FLG))                                    # gpio to pump
                        db_tick_wwp = 0
                        latch1 = False
                        if ESD_FLAG == 1:
                            adj_pmp_tm = time.time() - ww_pm_tm                                  # if esd keep adjusting pump start
                            ww_pm_tm = time.time() - adj_pmp_tm
                elif (db_tick_wwp > 10000 and wwon == 2) or latch1 == True:                      # now back at refill level
                    if (wash_tank_refil):
                        db_tick_wwp -= 1
                        latch1 = True
                        if db_tick_wwp == 0:                                                     # low for long enough                       
                            pi.write(WW_PUMP_ON, (1 and not ESD_FLG)) 
                            latch1 = False
                            adj_pmp_tm = time.time() - ww_pm_tm                                  # adjust pump start
                            ww_pm_tm = time.time() - adj_pmp_tm                            
                else:
                    pi.write(WW_PUMP_ON, 0)

                if (db_tick_wwp > 10000 or latch1 == True) and not wwon == 2:                    # at refill level but pump already been asked to stop
                    db_tick_wwp = 0                
                    latch1 = False  
                            
                if number_of_operations <= MAX_NO_OPS:
                    RESOURCE = Free
                else:   
                    RESOURCE = Booked
                      
            # if the RESOURCE is free then send a new resource for booking if there is anything queued in any queue
            if RESOURCE == Free and UNAVAIL_COMMON_RES == False:

                inhibits = [ ]                                                          # read inhibits from the gpio on this controller
                for ihp in inhibit_pins:
                    inhibits.append(int(read_gpio(pi, int(ihp)))) 
                    
                item = -1
                if len(inhibits) > 0:                                                               
                    if inhibits[0] == 0:        
                        item, p1_q = get_item_from_top_of_Q(p1_q)  
                    if item == -1:                                                      # no item found check the next priority queue and so on
                        if inhibits[1] == 0:            
                            item, p2_q = get_item_from_top_of_Q(p2_q) 
                        if item == -1:   
                            if inhibits[2] == 0:            
                                item, p3_q = get_item_from_top_of_Q(p3_q)   
                            if item == -1: 
                                if inhibits[3] == 0:                
                                    item, p4_q = get_item_from_top_of_Q(p4_q) 
                                if item == -1:  
                                    if inhibits[4] == 0:                    
                                        item, p5_q = get_item_from_top_of_Q(p5_q)  
                                    if item == -1:        
                                        print("all queues are empty")
                if not item == -1:
                    w = write_holding_reg(client_pro, reg=modbus_novus_procon.PROCON_PL101_BBRAM_START+30+item, value=1, method="tcp")                # book the item
                    RESOURCE = Booked
                    UNIT_NOS.append(item)
                    print("item booked == ",item)

            # write the data for the HMI via modbus tcp slave
            if DO_WRITE_SLAVE == 0:
                MANU_TRIG = man_trig                
                DP_TRIG = dp_trig
                RUN_TIMES = service_time_trig
                ALUM_TR = alum_trig
                TURBID_TR = turb_trig                
                DO_WRITE_SLAVE = 1                                               # inititate write to the slave data
            
        except (KeyboardInterrupt) as k: 
            print(k) 
            print("\033[31m closing down...... \033[0m")
            try:        
                client_novus.close()
            except (ValueError, Exception) as e:
                print(e)
            try:
                client_pro.close()
            except (ValueError, Exception) as e:
                print(e)            

# serial modbus rtu data transfer
async def update_datablock(slave1: ModbusSlaveContext, slave2: ModbusSlaveContext):
    count = 0
    print('start of slave serial interface')
    while True:
        INH_PLC1 = slave1.getValues(4, 0, NO_OF_UNITS)                    # input registers
        INH_PLC2 = slave2.getValues(3, 0, NO_OF_UNITS)                    # holding registers
        if pump == 1:
            level = level - 10
        level = level + 1
        CHAN_LVL_PLC2 = slave2.getValues(3, 0, 1)[0]
        CHAN_LVL_PLC2 = min(1000, max(0, CHAN_LVL_PLC2))
        slave1.setValues(4, 10, [CHAN_LVL_PLC2])
        # this data might be float or int 
        regs = slave2.getValues(3, 10, NO_OF_PLC2_FLOATS)       
        decoder = BinaryPayloadDecoder.fromRegisters(regs, byteorder=Endian.Big)
        int16 = decoder.decode_16bit_int()
        FLOAT32ARR = decoder.decode_32bit_float()
        slave1.setValues(4, 20, int16)                                 # send them to PLC1 as ints
        await asyncio.sleep(0.1)

# modbus tcp to HMI data transfer
async def update_datablock2(store: ModbusSlaveContext):
    count = 0
    print('start of tcp slave interface')
    while True:
        INH_HMI = store.getValues(1, 0, 1)[0]                           # get coil registers from HMI
        QUE_INH = store.getValues(1, 0, 1)[0][0]                        # get coil registers from HMI
        
        store.setValues(4, 0, [CHAN_LVL_PLC2])                          # send for displaying info on graphics 
        if DO_WRITE_SLAVE == 1:
            store.setValues(4, 1, DP_TRIG)    
            store.setValues(4, 1+NO_OF_UNITS, RUN_TIMES) 
            store.setValues(4, 1+NO_IR_INTS, ALUM_TR) 
            store.setValues(4, 1+NO_IR_INTS+(NO_OF_UNITS*2), TURBID_TR) # floats are 32 bit so mnultiply by 2
            store.setValues(1, 10, MANU_TRIG)
            DO_WRITE_SLAVE = 0
        store.setValues(4, 1+NO_IR_INTS+(NO_OF_UNITS*2), FLOAT32ARR)    # write them for HMI as floats		
        await asyncio.sleep(0.1)

# run the serial slave with collection		
async def run_serial(do_it):
    if do_it == 1:
        slave1 = ModbusSlaveContext(ir=ModbusSequentialDataBlock(0, [17, 500]))
        slave2 = ModbusSlaveContext(hr=ModbusSequentialDataBlock(0, [17, 0]))
        slaves = {
            MY_SLAVE_RTU1: slave1,
            MY_SLAVE_RTU2: slave2,
        }
        context = ModbusServerContext(slaves=slaves, single=False)

        task = asyncio.create_task(update_datablock(slave1, slave2))
        task.set_name("modbus rtu serial data r/w task started")
    	# serial server
        await StartAsyncSerialServer(context=context, port=MOD_SER_PORT, baudrate=BR, stopbits=1, bytesize=8, parity="N")
        task.cancel()
        return 1
    else:
        return 0

# run the tcp slave with collection     
async def run_tcp(do_it):
    if do_it == 1:
        # we are biulding a custom data block area
        builder = BinaryPayloadBuilder(byteorder=Endian.Big) # we have so many integers and so many floats 
        for i in range(0, NO_IR_INTS):
            builder.add_16bit_int(i)
        for i in range(0, NO_IR_FLOATS):
            builder.add_32bit_float(1.0/float(i))
        block = ModbusSequentialDataBlock(1, builder.to_registers())
        store = ModbusSlaveContext( di=ModbusSequentialDataBlock(0, [17] * 100), co=ModbusSequentialDataBlock(0, [17] * 100), hr=ModbusSequentialDataBlock(0, [17] * 100), ir=block)
        context = ModbusServerContext(slaves=store, single=True)
        identity = ModbusDeviceIdentification()
        identity.VendorName = 'ACP'
        identity.ProductCode = 'PM'
        identity.VendorUrl = 'http://github.com/aircampro'
        identity.ProductName = 'Pymodbus Server'
        identity.ModelName = 'Pymodbus Server'
        identity.MajorMinorRevision = '1.0'
        task1 = asyncio.create_task(update_datablock2(store))
        task1.set_name("modbus tcp r/w task started")
        await StartTcpServer(context, identity=identity, address=(MY_TCP_SLAVE, TCP_SLAVE_PORT))
        task1.cancel()
        return 1
    else:
        return 0

# schedule queue and two slaves as three tasks        
async def order():
    print('running the queue and modbus client (master) with modbus servers (slaves) for tcp and serial') 
    task1 = asyncio.create_task(main_filter_queue_task())
    task2 = asyncio.create_task(run_serial(1)) 
    task3 = asyncio.create_task(run_tcp(1)) 
    r1 = await task1
    r2 = await task2
    r3 = await task3
    
    print(r1, r2, r3)
    
if __name__ == "__main__":
    pymodbus.pymodbus_apply_logging_config("DEBUG") 
    asyncio.run(order(), debug=True )
