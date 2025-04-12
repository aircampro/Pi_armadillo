#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# =====================================================================================
# Example Vending machine Blender
# HMI is modbus slave
# I/O is direct if there is enough pins (need use extended version or reduce additives)
#     or via a modbus slave device
# =====================================================================================
#
from enum import Enum
import RPi.GPIO as GPIO
import time
import ctypes
import pickle 
import argparse
import configparser

from __future__ import absolute_import, unicode_literals
from subprocess import PIPE, Popen

def cmdline(command):
    """
    shell command to run script and check result
    """
    return Popen(
        args=command,
        stdout=PIPE,
        stderr=PIPE,
        shell=True
    )

# functions for modbus HMI which is a slave port (server)
from pymodbus.client.sync import ModbusTcpClient as ModbusClient
MODBUS_TCP_SERVER_IP="10.0.0.1"                                       # hmi
MODBUS_TCP_PORT=5020
# functions for modbus i/o which is a slave port (server)
MODBUS_REMOTE_IO="10.0.0.2"                                           # ip addr for remote i/o

def connectModbusTCP(ip=MODBUS_TCP_SERVER_IP, port=MODBUS_TCP_PORT):
    client = ModbusClient(ip, port=port)
    client.connect()
    return client
   
def read_holding_reg(client, reg=1, method="tcp", unit=1):
    print("Read holding register")
    if method == "tcp":
        rr = client.read_holding_registers(address=reg, count=1)
    else :		
        rr = client.read_holding_registers(address=reg, count=1, unit=unit)
    print(rr.registers)
    return rr.registers

def writeCoilModbus(client,addr,boolValue):
    try:
        rr = client.write_coil(addr, boolValue)          # example True or False
    except Exception as e:
        print("Exception in write_coil = ",e)     
    return readCoilModbus(addr)	
    
def readCoilModbus(client,addr=1,num=1):
    # rr = client.read_coils(1, 1) - would read first coil and return it
    try:
        rr = client.read_coils(addr, num)
    except Exception as e:
        print("Exception in read_coils = ",e)    
	return rr.bits[0]

def writeHoldingRegisterModbus(client, addr, value): 
    try:
        rr = client.write_register(addr, value)
    except Exception as e:
        print("Exception in write_holding_register = ",e)  
    return read_holding_reg(client, addr, 1)
    
def disconnectModbusTCP(client):
    client.close()
    
# list of possible operation steps
class OperationSteps(Enum):
    WAIT_FOR_START = 8
    CHOOSE_PRODUCT = 9
    CHOOSE_PAY = 10
    GET_PAYMENT = 11
    ADD_1 = 0
    ADD_2 = 1
    ADD_3 = 2
    ADD_4 = 3
    ADD_5 = 4	
    ADD_6 = 5
    MAIN_PROD = 6
    CLEAN = 7
    CHOOSE_OVR = 12
    ESTOP = 13
    WAIT_FOR_FILL = 14
	
# class containing atttributes for the additives
class Additive(object):
    def __init__(self):
        self.name = "additive"
        self.used = False
        self.io_ext = False
        self.lvl = 27
        self.top = 26
        self.mtr_time = 10
        self.btm = 25
        self.cost = 10
        self.mtr_pin = 20
        
# class containing atttributes for the main product being blended
class Product(object):
    def __init__(self):
        self.name = "main_blend"		
        self.lvl = 24
        self.pump = 23
        self.time = 100
        self.vol = 1.0
        self.cost = 100
        
# class containing atttributes for the end cleaning process
class Clean(object):
    def __init__(self):
        self.name = "clean"		
        self.flush = 22
        self.time = 50

class HMI(object):
    def __init__(self):
        self.st = 0		
        self.prod = 0
        self.lang = 0
        self.pay_meth = 0
        self.override = 0
        self.step_state = 0
        
# class describing the override actions
class OverrideActions(Enum):
    WAIT_FOR_HMI = 0
    OVR = 1
    CANCEL = 2

# class describing the possible payment options
class PayMthd(Enum):
    NONE = -1
    CANC = 0
    PAYTER = 1
    PAYPAY = 2
    FELICA = 3
    OPENCV = 4
    CASH = 5
    JPAY = 6
    LINEPAY = 7
    WECHAT = 8
    APPLE = 9
    
class IOMode(Enum):
    DIRECT_GPIO = 0
    MDBUS_SLV = 1
    MDBUS_MAS = 2
    MITSI = 3
    
f_name = "vend.pkl"
        
# class operating the vending machine   
class vendingSequence(object):

    def __init__(self, mode=IOMode.DIRECT_GPIO.value):
        self.pay_method = PayMthd.NONE.value
        self.step = 0
        self.cost = 0
        self.lvls = 0
        self.pay_ok = 0
        self.io_mode = mode
                
    # reads the configuration for each possible selected product		
    def read_ini(self, recipe_indx, lang):     
        self.ini_file = recipe_indx + "_" + lang + ".ini"

        # open the .ini config file
        config_ini = configparser.ConfigParser()
        config_ini.read(self.ini_file, encoding='utf-8') 
 
        # read additives operation parameters  
        self.add1 = Additive()
        self.add1.name = str(config_ini['ADD1']['NM'])
        self.add1.used = int(config_ini['ADD1']['USED'])
        self.add1.lvl = int(config_ini['ADD1']['LVL'])
        self.add1.top = int(config_ini['ADD1']['TOP']) 
        self.add1.mtr_time = int(config_ini['ADD1']['MTR_TIME'])
        self.add1.btm = int(config_ini['ADD1']['BTM'])
        self.add1.cost = int(config_ini['ADD1']['COST'])
        self.add1.mtr_pin = int(config_ini['ADD1']['MTR_PIN'])

        self.add2 = Additive()
        self.add2.name = str(config_ini['ADD2']['NM'])
        self.add2.used = int(config_ini['ADD2']['USED'])
        self.add2.lvl = int(config_ini['ADD2']['LVL'])
        self.add2.top = int(config_ini['ADD2']['TOP']) 
        self.add2.mtr_time = int(config_ini['ADD2']['MTR_TIME'])
        self.add2.btm = int(config_ini['ADD2']['BTM'])
        self.add2.cost = int(config_ini['ADD2']['COST'])
        self.add2.mtr_pin = int(config_ini['ADD2']['MTR_PIN'])

        self.add3 = Additive()
        self.add3.name = str(config_ini['ADD3']['NM'])
        self.add3.used = int(config_ini['ADD3']['USED'])
        self.add3.lvl = int(config_ini['ADD3']['LVL'])
        self.add3.top = int(config_ini['ADD3']['TOP']) 
        self.add3.mtr_time = int(config_ini['ADD3']['MTR_TIME'])
        self.add3.btm = int(config_ini['ADD3']['BTM'])
        self.add3.cost = int(config_ini['ADD3']['COST'])
        self.add3.mtr_pin = int(config_ini['ADD3']['MTR_PIN'])

        self.add4 = Additive()
        self.add4.name = str(config_ini['ADD4']['NM'])
        self.add4.used = int(config_ini['ADD4']['USED'])
        self.add4.lvl = int(config_ini['ADD4']['LVL'])
        self.add4.top = int(config_ini['ADD4']['TOP']) 
        self.add4.mtr_time = int(config_ini['ADD4']['MTR_TIME'])
        self.add4.btm = int(config_ini['ADD4']['BTM'])
        self.add4.cost = int(config_ini['ADD4']['COST'])
        self.add4.mtr_pin = int(config_ini['ADD4']['MTR_PIN'])

        self.add5 = Additive()
        self.add5.name = str(config_ini['ADD5']['NM'])
        self.add5.used = int(config_ini['ADD5']['USED'])
        self.add5.lvl = int(config_ini['ADD5']['LVL'])
        self.add5.top = int(config_ini['ADD5']['TOP']) 
        self.add5.mtr_time = int(config_ini['ADD5']['MTR_TIME'])
        self.add5.btm = int(config_ini['ADD5']['BTM'])
        self.add5.cost = int(config_ini['ADD5']['COST'])
        self.add5.mtr_pin = int(config_ini['ADD5']['MTR_PIN'])

        self.add6 = Additive()
        self.add6.name = str(config_ini['ADD6']['NM'])
        self.add6.used = int(config_ini['ADD6']['USED'])
        self.add6.lvl = int(config_ini['ADD6']['LVL'])
        self.add6.top = int(config_ini['ADD6']['TOP']) 
        self.add6.mtr_time = int(config_ini['ADD6']['MTR_TIME'])
        self.add6.btm = int(config_ini['ADD6']['BTM'])
        self.add6.cost = int(config_ini['ADD6']['COST'])
        self.add6.mtr_pin = int(config_ini['ADD6']['MTR_PIN'])

        # read product operation parameters        
        self.main = Product()		
        self.main.name = str(config_ini['MAIN']['NM'])		
        self.main.lvl = int(config_ini['MAIN']['LVL'])
        self.main.pump = int(config_ini['MAIN']['PMP'])
        self.main.time = int(config_ini['MAIN']['TIME'])
        self.main.vol = int(config_ini['MAIN']['VOL'])
        self.main.cost = int(config_ini['MAIN']['COST'])

        # read clean operation parameters
        self.clean = Clean()
        self.name = str(config_ini['CLEAN']['NM'])		
        self.flush = int(config_ini['CLEAN']['LVL'])
        self.time = int(config_ini['CLEAN']['TIME'])
 
        # read modbus config for hmi 
        self.hmi_regs = HMI()
        self.hmi_regs.st = int(config_ini['HMI']['START_PB'])		
        self.hmi_regs.prod = int(config_ini['HMI']['PROD_ID'])
        self.hmi_regs.lang = int(config_ini['HMI']['LNG'])
        self.hmi_regs.pay_meth = int(config_ini['HMI']['PAY_METHOD'])
        self.hmi_regs.override = int(config_ini['HMI']['OVD_PB'])
        self.hmi_regs.step_state = int(config_ini['HMI']['STEP_STATE'])
       
        self.op_list = [ self.add1, self.add2, self.add3, self.add4, self.add5, self.add6, self.main, self.clean ]		

    # check the container levels        
    def check_levels(self):
        if self.io_mode == IOMode.DIRECT_GPIO.value:
            # must return a bit map for the levels that are low (not enough product)
            ret = 0
            for i in range(0,5):
                if not GPIO.input(self.op_list[i].lvl):
                    ret |= pow(2,i)
            if not GPIO.input(self.op_list[OperationSteps.MAIN_PROD.value].lvl:
                ret |= pow(2,6)        
            return ret
        elif self.io_mode == IOMode.MDBUS_SLV.value:
            mrio = connectModbusTCP(ip=MODBUS_REMOTE_IO)
            ret = 0
            for i in range(0,5):
                if not readCoilModbus(mrio, self.op_list[i].lvl):
                    ret |= pow(2,i)
            if not readCoilModbus(mrio, self.op_list[OperationSteps.MAIN_PROD.value].lvl):
                ret |= pow(2,6)   
            disconnectModbusTCP(mrio)                
            return ret

    # calculate the total cost               
    def get_cost(self):
        self.cost = (self.add1.cost * self.add1.used) + (self.add2.cost * self.add2.used) + (self.add3.cost * self.add3.used) 
        self.cost = self.cost + (self.add4.cost * self.add4.used) + (self.add5.cost * self.add5.used) + (self.add6.cost * self.add6.used)  
        self.cost = self.cost + (self.main.cost * self.main.vol)

    # charge the user for the product        
    def get_payment(self):
        self.get_cost()
        if self.pay_method == PayMthd.PAYTER.value :
            self.pay_ok = cmdline(f'Payter_serial_PSP.py {self.cost}').stdout.readline()    # call the payment method with the cost 

    # set data from the hmi via modbus tcp 
    def mdbus_set_to_hmi(self, rr, v):
        c = connectModbusTCP()  
        writeHoldingRegisterModbus(c,rr,v)
        disconnectModbusTCP(c)
        return c

    # choose payment method            
    def choose_method(self, method_hmi):
        self.pay_method = PayMthd.NONE.value	
        while (OperationSteps.CHOOSE_PAY.value == self.step):
            self.pay_method = method_hmi
            r = mdbus_set_to_hmi(self.hmi_regs.step_state, self.step)
            if not self.pay_method == PayMthd.NONE.value:
                if self.pay_method == PayMthd.CANC.value:
                    self.step = OperationSteps.WAIT_FOR_START.value  
                else:                    
                    self.get_payment()      
                    if not str(self.pay_ok).find("RETCODE==0") = -1:
                        self.step = OperationSteps.ADD_1.value    
                    else:
                        self.step = OperationSteps.WAIT_FOR_START.value 

    # get data from the hmi via modbus tcp 
    def mdbus_get_from_hmi(self, rr):
        c = connectModbusTCP()  
        rhr = read_holding_reg(c, reg=rr, method="tcp") 
        disconnectModbusTCP(c)
        return rhr

    # wait for the start signal from the hmi       
    def wait_for_start(self, hmi_start):
        while (OperationSteps.WAIT_FOR_START.value == self.step):
            hmi_start = self.mdbus_get_from_hmi(self.hmi_regs.st)
            r = mdbus_set_to_hmi(self.hmi_regs.step_state, self.step)
            if not hmi_start == 0:
                self.step = OperationSteps.CHOOSE_PRODUCT.value

    # configure the GPIO i/o
    def set_up_io(self):
        if self.io_mode == IOMode.DIRECT_GPIO.value:
            GPIO.setmode(GPIO.BOARD)
            for j in range(0,5):       
                GPIO.setup(self.op_list[j].lvl, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                GPIO.setup(self.op_list[j].top, GPIO.OUT)
                GPIO.setup(self.op_list[j].btm, GPIO.OUT)
                GPIO.setup(self.op_list[j].mtr_pin, GPIO.OUT)
            GPIO.setup(self.op_list[OperationSteps.MAIN_PROD.value].lvl, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.op_list[OperationSteps.MAIN_PROD.value].pump, GPIO.OUT)
            GPIO.setup(self.op_list[OperationSteps.CLEAN.value].flush, GPIO.OUT)
        
    # choose product               
    def choose_product(self, prod_id_hmi, lang_hmi):
        while (OperationSteps.CHOOSE_PRODUCT.value == self.step):
            prod_id_hmi = self.mdbus_get_from_hmi(self.hmi_regs.prod)
            lang_hmi  = self.mdbus_get_from_hmi(self.hmi_regs.lang)    
            r = mdbus_set_to_hmi(self.hmi_regs.step_state, self.step)      
            self.read_ini(prod_id_hmi, lang_hmi)
            self.lvls = self.check_levels()
            if (self.lvls == 0):
                self.step = OperationSteps.CHOOSE_PAY.value
            else:
                self.step = OperationSteps.CHOOSE_OVR.value

    # wait for fill               
    def wait_for_fill(self):
        while (OperationSteps.WAIT_FOR_FILL.value == self.step):
            self.lvls = self.check_levels()
            r = mdbus_set_to_hmi(self.hmi_regs.step_state, self.step)
            if not (self.lvls & 0x64):
                self.step = OperationSteps.CHOOSE_PAY.value
                
    # sets the used parameter of the relevant objects to be not used in the cost calculation of this product due to unavilabilty                
    def set_not_used(self, lvlword):
        if lvlword & 0x1:
            self.op_list[OperationSteps.ADD_1.value].used = 0
        if lvlword & 0x2:
            self.op_list[OperationSteps.ADD_2.value].used = 0
        if lvlword & 0x4:
            self.op_list[OperationSteps.ADD_3.value].used = 0
        if lvlword & 0x8:
            self.op_list[OperationSteps.ADD_4.value].used = 0
        if lvlword & 0x16:
            self.op_list[OperationSteps.ADD_5.value].used = 0   
        if lvlword & 0x32:
            self.op_list[OperationSteps.ADD_6.value].used = 0
        if lvlword & 0x64:
            self.step = OperationSteps.WAIT_FOR_FILL.value
            
    # choose override to continue or cancel order           
    def choose_override(self, override_step_hmi):
        while (OperationSteps.CHOOSE_OVR.value == self.step):
            override_step_hmi = self.mdbus_get_from_hmi(self.hmi_regs.override) 
            r = mdbus_set_to_hmi(self.hmi_regs.step_state, self.step)
            if override_step_hmi == OverrideActions.OVR.value:
                self.step = OperationSteps.CHOOSE_PAY.value
                self.set_not_used(self.lvls)
            elif override_step_hmi == OverrideActions.CANCEL.value:
                self.step = OperationSteps.WAIT_FOR_START.value                 

    # operates the fixed volume slot to put in additive
    def operate_slot(self, add_obj):
        if self.io_mode == IOMode.DIRECT_GPIO.value:
            GPIO.output(add_obj.top,0)                                     # open top slot
            GPIO.output(add_obj.mtr_pin, 1)
            time.sleep(add_obj.mtr_time)
            GPIO.output(add_obj.mtr_pin, 0)

            GPIO.output(add_obj.top,1)                                     # close top slot
            GPIO.output(add_obj.mtr_pin, 1)
            time.sleep(add_obj.mtr_time)
            GPIO.output(add_obj.mtr_pin, 0)

            GPIO.output(add_obj.btm,0)                                     # open botom slot
            GPIO.output(add_obj.mtr_pin, 1)
            time.sleep(add_obj.mtr_time)
            GPIO.output(add_obj.mtr_pin, 0)

            GPIO.output(add_obj.btm,1)                                     # close bottom slot
            GPIO.output(add_obj.mtr_pin, 1)
            time.sleep(add_obj.mtr_time)
            GPIO.output(add_obj.mtr_pin, 0)
            return 1                                                       # if we have feedback check it first
        elif self.io_mode == IOMode.MDBUS_SLV.value:
            mrio = connectModbusTCP(ip=MODBUS_REMOTE_IO)
            writeCoilModbus(mrio, add_obj.top, False)                      # open top slot
            writeCoilModbus(mrio, add_obj.mtr_pin, True)
            time.sleep(add_obj.mtr_time)
            writeCoilModbus(mrio, add_obj.mtr_pin, False)

            writeCoilModbus(mrio, add_obj.top, True)                       # close top slot
            writeCoilModbus(mrio, add_obj.mtr_pin, True)
            time.sleep(add_obj.mtr_time)
            writeCoilModbus(mrio, add_obj.mtr_pin, False)

            writeCoilModbus(mrio, add_obj.btm, False)                      # open botom slot
            writeCoilModbus(mrio, add_obj.mtr_pin, True)
            time.sleep(add_obj.mtr_time)
            writeCoilModbus(mrio, add_obj.mtr_pin, False)

            writeCoilModbus(mrio, add_obj.btm, True)                        # close bottom slot
            writeCoilModbus(mrio, add_obj.mtr_pin, True)
            time.sleep(add_obj.mtr_time)
            writeCoilModbus(mrio, add_obj.mtr_pin, False)  
            disconnectModbusTCP(mrio) 
            return 1
            
    # choco	
    def additive_1(self):
        ok = 0
        while (OperationSteps.ADD_1.value == self.step):
            r = mdbus_set_to_hmi(self.hmi_regs.step_state, self.step)
            if self.op_list[self.step].used == 0:
                self.step += 1
                break
            if (operate_slot(self.op_list[self.step])==1):
                self.step += 1            
            
    # banana
    def additive_2(self):
        while (OperationSteps.ADD_2.value == self.step):
            r = mdbus_set_to_hmi(self.hmi_regs.step_state, self.step)
            if self.op_list[self.step].used == 0:
                self.step += 1
                break
            if (operate_slot(self.op_list[self.step])==1):
                self.step += 1  
                
    # strawberry
    def additive_3(self):	
        while (OperationSteps.ADD_3.value == self.step):
            r = mdbus_set_to_hmi(self.hmi_regs.step_state, self.step)
            if self.op_list[self.step].used == 0:
                self.step += 1
                break
            if (operate_slot(self.op_list[self.step])==1):
                self.step += 1  
                
    # vanilla
    def additive_4(self):	
        while (OperationSteps.ADD_4.value == self.step):
            r = mdbus_set_to_hmi(self.hmi_regs.step_state, self.step)
            if self.op_list[self.step].used == 0:
                self.step += 1
                break
            if (operate_slot(self.op_list[self.step])==1):
                self.step += 1  
                
    # cherry
    def additive_5(self):
        while (OperationSteps.ADD_5.value == self.step):
            r = mdbus_set_to_hmi(self.hmi_regs.step_state, self.step)
            if self.op_list[self.step].used == 0:
                self.step += 1
                break
            if (operate_slot(self.op_list[self.step])==1):
                self.step += 1  
                
    # lime
    def additive_6(self):
        while (OperationSteps.ADD_6.value == self.step):
            r = mdbus_set_to_hmi(self.hmi_regs.step_state, self.step)
            if self.op_list[self.step].used == 0:
                self.step += 1
                break
            if (operate_slot(self.op_list[self.step])==1):
                self.step += 1  
                
    # main product (milk) - consider adding variants e.g. soya / goats / skimmed
    def main_prod(self):
        while (OperationSteps.MAIN_PROD.value == self.step):
            if self.io_mode == IOMode.DIRECT_GPIO.value:
                GPIO.output(self.op_list[self.step].pump,1)
                time.sleep(self.op_list[self.step].time) 
                GPIO.output(self.op_list[self.step].pump,0)
                self.step += 1 
            elif self.io_mode == IOMode.MDBUS_SLV.value:
                mrio = connectModbusTCP(ip=MODBUS_REMOTE_IO)
                writeCoilModbus(mrio, self.op_list[self.step].pump, True)
                time.sleep(self.op_list[self.step].time) 
                writeCoilModbus(mrio, self.op_list[self.step].pump, False)
                disconnectModbusTCP(mrio)             
    # clean
    def clean(self):
        while (OperationSteps.CLEAN.value == self.step):
            r = mdbus_set_to_hmi(self.hmi_regs.step_state, self.step)
            if self.io_mode == IOMode.DIRECT_GPIO.value:
                GPIO.output(self.op_list[self.step].flush,1)
                time.sleep(self.op_list[self.step].time) 
                GPIO.output(self.op_list[self.step].flush,0)
                self.step = OperationSteps.WAIT_FOR_START.value 
            elif self.io_mode == IOMode.MDBUS_SLV.value:
                mrio = connectModbusTCP(ip=MODBUS_REMOTE_IO)
                writeCoilModbus(mrio, self.op_list[self.step].flush, True)
                time.sleep(self.op_list[self.step].time) 
                writeCoilModbus(mrio, self.op_list[self.step].flush, False)
                disconnectModbusTCP(mrio) 

    # save for recall if power fails                
    def save_pickle_file(self, o):
        global f_name
        with open(f_name, 'wb') as f:
            pickle.dump(o, f) 
            
    # sequence of unit operations in the vending machine        
    def main(self, hmi, vm):
        self.wait_for_start(hmi.st)
        self.save_pickle_file(vm)
        self.choose_product(hmi.prod, hmi.lang)
        self.save_pickle_file(vm)
        self.choose_method(hmi.pay_meth)
        self.save_pickle_file(vm)
        self.choose_override(hmi.override)
        self.save_pickle_file(vm)
        self.additive_1()
        self.save_pickle_file(vm)
        self.additive_2()
        self.save_pickle_file(vm)
        self.additive_3()
        self.save_pickle_file(vm)
        self.additive_4()
        self.save_pickle_file(vm)
        self.additive_5()  
        self.save_pickle_file(vm)        
        self.additive_6()   
        self.save_pickle_file(vm)        
        self.main_prod()
        self.save_pickle_file(vm)
        self.clean()   
        self.save_pickle_file(vm)
        self.wait_for_fill()   
        self.save_pickle_file(vm)
        
if __name__ == "__main__":

    global f_name
    try:
        parser = argparse.ArgumentParser(description='command line parse')
        parser.add_argument('-r', '--restore', type=int, dest='cFlag', default=0, help='set to 1 to restore the controller from last saved settings')
        args = parser.parse_args()
    
        if args.cFlag == 1:                                               # if we passed r -1 as arguemnts we re-load the sequence where it last was
            print("\033[36m re-loading saved sequence controller! \033[0m")
            with open(f_name, 'rb') as f:
                vm = pickle.load(f)                                       # load the last saved sequencer from pickle file
                print("restarted using the pickle save file")
        else:                 
            vm = vendingSequence()                                        # for modbus version vm = vendingSequence(IOMode.MDBUS_SLV.value)
        hmi = HMI()                                                       # create the hmi reader object
        vm.set_up_io()
        while True:
            vm.main(hmi, vm)
    except KeyboardInterrupt:	
        print("\033[31m vending machine sequence was killed \033[0m")        
