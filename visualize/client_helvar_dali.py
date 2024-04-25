#!/usr/bin/python3
# helvar net is used to bridge to the dali light networks
#
import socket
import time
import sys
import threading
import weakref
import datetime
import re

HELVAR_PORT=50000

class HelvarNet:
    DIRECT_LEVEL = '>V:1,C:14,L:{0},F:{1},@:{2}#'
    GROUP_LEVEL = '>V:1,C:13,G:{0},L:{1},F:{2}#'
    DAYLIGHT_SAVING = '>V:1,C:245,Y:{0}#'    
    TIMEZONE_DIFF = '>V:1,C:244,Z:{0}#'  
    RST_GRP_EBAT_TIME = '>V:1,C:205,G:{0}#'
    RST_DEV_EBAT_TIME = '>V:1,C:206,@:{0}#'
    RCL_SCENE_ON_GROUP = '>V:1,C:11,G:{0},B:{1},S:{2},F:{3}#'
    RCL_SCENE_ON_DEVICE = '>V:1,C:12,B:{0},S:{1},F:{2},@:{3}#'    
    STORE_SCENE_FOR_GROUP = '>V:1,C:201,G:{0},O:{1},B:{2},S:{3},F:{4}#'
    STORE_SCENE_FOR_DEVICE = '>V:1,C:202,@{0},O:{1},B:{2},S:{3},F:{4}#'
    STORE_NOW_FOR_GROUP = '>V:1,C:203,G:{0},O:{1},B:{2},S:{3}#'
    STORE_NOW_FOR_DEVICE = '>V:1,C:204,@:{0},O:{1},B:{2},S:{3}#'    
    SET_ROUTER_TIME = '>V:1,C:241,T:{0}#'
    DEVICE_POWER_CONS = '>V:1,C:160,@:{0}#'
    GROUP_POWER_CONS = '>V:1,C:161,@:{0}#'
    DEVICE_LOAD_LEVEL = '>V:1,C:152,@:{0}#'
    DEVICE_MEAS = '>V:1,C:150,@:{0}#'
    DEVICE_INPUT_STATE = '>V:1,C:151,@:{0}#'
    DEVICE_FAULTY = '>V:1,C:114,@:{0}#'
    DEVICE_MISSING = '>V:1,C:113,@:{0}#'
    DEVICE_DISABLED = '>V:1,C:111,@:{0}#'
    DEVICE_STATE = '>V:1,C:110,@:{0}#'
    DEVICE_TYPE = '>V:1,C:104,@:{0}#'
    EMER_BAT_CHG = '>V:1,C:174,@:{0}#'
    EMER_BAT_TIME = '>V:1,C:175,@:{0}#'
    EMER_BAT_TOT_LAMP = '>V:1,C:176,@:{0}#'
    EMER_BAT_FAIL = '>V:1,C:129,@:{0}#'
    HELVAR_GATEWAY_TIME = '>V:1,C:185#'
    HELVAR_GATEWAY_LAT = '>V:1,C:186#'
    HELVAR_GATEWAY_LON = '>V:1,C:187#'
    HELVAR_GATEWAY_TIMEZONE = '>V:1,C:188#'
    HELVAR_GATEWAY_DSTIME = '>V:1,C:189#'   
    QUERY_CLUSTERS = '>V:1,C:101#' 
    QUERY_ROUTERS = '>V:1,C:102#' 
     
    def __init__(self, ip, port=50000, retry_no=3):
        self.__s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__s.connect((ip, port))
        self.__t = threading.Thread(target = self.__t_func,
                                    args = (self, weakref.ref(self)))
        self.__lock = threading.RLock()
        self.__ip = ip
        self.__port = port
        self.__retry_no = retry_no
        self.__running = True
        self.__day_on = 1                            # enable daylight saving
        self.__timezone_diff = 0                     # timezone difference in seconds
        self.__epoch = 0                             # store last router sync time
        self.__t.start()
    def get_current_time_epoch():
        epoch = datetime.datetime.now().strftime('%s')
        return str(epoch)
    def set_router_current_time():
        self.__epoch = self.get_current_time_epoch()
        self.__send(self.SET_ROUTER_TIME.format(self.__epoch))        
    def set_direct_level(self, address, level, fade_time = 0):
        self.__send(self.DIRECT_LEVEL.format(level, fade_time, address))
    def get_power_cons_device(self, address):
        received = self.__send_and_rcv(self.DEVICE_POWER_CONS.format(address))
        received = re.search("(?<=\=).*(?=#)", str(received)).group()
        return received
    def get_load_level_device(self, address):
        received = self.__send_and_rcv(self.DEVICE_LOAD_LEVEL.format(address))
        received = re.search("(?<=\=).*(?=#)", str(received)).group()
        return received
     def get_measurement_device(self, address):
        received = self.__send_and_rcv(self.DEVICE_MEAS.format(address))
        received = re.search("(?<=\=).*(?=#)", str(received)).group()
        return received
    def get_input_state_device(self, address):
        received = self.__send_and_rcv(self.DEVICE_INPUT_STATE.format(address))
        received = re.search("(?<=\=).*(?=#)", str(received)).group()
        return received       
    def get_emergency_bat_charge_device(self, address):
        received = self.__send_and_rcv(self.EMER_BAT_CHG.format(address))
        received = re.search("(?<=\=).*(?=#)", str(received)).group()
        return received
    def get_emergency_bat_time_device(self, address):
        received = self.__send_and_rcv(self.EMER_BAT_TIME.format(address))
        received = re.search("(?<=\=).*(?=#)", str(received)).group()
        return received
    def get_emergency_bat_total_lamp_device(self, address):
        received = self.__send_and_rcv(self.EMER_BAT_TOT_LAMP.format(address))
        received = re.search("(?<=\=).*(?=#)", str(received)).group()
        return received   
    def get_emergency_bat_failure_device(self, address):
        received = self.__send_and_rcv(self.EMER_BAT_FAIL.format(address))
        received = re.search("(?<=\=).*(?=#)", str(received)).group()
        return received   
    def get_fault_device(self, address):
        received = self.__send_and_rcv(self.DEVICE_FAULTY.format(address))
        received = re.search("(?<=\=).*(?=#)", str(received)).group()
        return received  
    def get_missing_device(self, address):
        received = self.__send_and_rcv(self.DEVICE_MISSING.format(address))
        received = re.search("(?<=\=).*(?=#)", str(received)).group()
        return received    
    def get_disabled_device(self, address):
        received = self.__send_and_rcv(self.DEVICE_DISABLED.format(address))
        received = re.search("(?<=\=).*(?=#)", str(received)).group()
        return received  
    def get_state_device(self, address):
        received = self.__send_and_rcv(self.DEVICE_STATE.format(address))
        received = re.search("(?<=\=).*(?=#)", str(received)).group()
        return received 
    def get_type_device(self, address):
        received = self.__send_and_rcv(self.DEVICE_TYPE.format(address))
        received = re.search("(?<=\=).*(?=#)", str(received)).group()
        return received        
    def recall_scene_on_device(self, address, block, scene, fade):  
        self.__send(self.RCL_SCENE_ON_DEVICE.format(block, scene, fade, address)) 
    def store_scene_for_device(self, address, force: bool, block, scene, level): 
        if force == True:
            fc = 1
        else:
            fc = 0        
        self.__send(self.STORE_SCENE_FOR_DEVICE.format(address, fc, block, scene, level))
    def store_curr_scene_for_device(self, address, force: bool, block, scene): 
        if force == True:
            fc = 1
        else:
            fc = 0        
        self.__send(self.STORE_NOW_FOR_DEVICE.format(address, fc, block, scene))
    def reset_device_elamp_bat_time(self, address):
        self.__send(self.RST_DEV_EBAT_TIME.format(address))  
    def set_group_level(self, grp, level, fade_time = 0):
        self.__send(self.GROUP_LEVEL.format(grp, level, fade_time)) 
    def get_power_cons_group(self, address):
        received = self.__send_and_rcv(self.GROUP_POWER_CONS.format(address))
        received = re.search("(?<=\=).*(?=#)", str(received)).group()
        return received
    def reset_group_elamp_bat_time(self, group):
        self.__send(self.RST_GRP_EBAT_TIME.format(group))  
    def recall_scene_on_group(self, group, block, scene, fade): 
        self.__send(self.RCL_SCENE_ON_GROUP.format(group, block, scene, fade))  
    def store_scene_for_group(self, group, force: bool, block, scene, level): 
        if force == True:
            fc = 1
        else:
            fc = 0        
        self.__send(self.STORE_SCENE_FOR_GROUP.format(group, fc, block, scene, level))   
    def store_curr_scene_for_group(self, group, force: bool, block, scene): 
        if force == True:
            fc = 1
        else:
            fc = 0        
        self.__send(self.STORE_NOW_FOR_GROUP.format(group, fc, block, scene))     
    def set_gateway_daylight_saving(self, ds=True):
        if ds == True:
            self.__day_on = 1
        else:
            self.__day_on = 0
        self.__send(self.DAYLIGHT_SAVING.format(self.__day_on))     
    def set__gateway_timezone_diff(self, diff=0):
        self.__timezone_diff = diff
        self.__send(self.TIMEZONE_DIFF.format(self.__timezone_diff))  
    def get_gateway_time(self, address):
        received = self.__send_and_rcv(self.HELVAR_GATEWAY_TIME.format(address))
        received = re.search("(?<=\=).*(?=#)", str(received)).group()
        return received  
    def get_gateway_timezone(self, address):
        received = self.__send_and_rcv(self.HELVAR_GATEWAY_TIMEZONE.format(address))
        received = re.search("(?<=\=).*(?=#)", str(received)).group()
        return received   
    def get_gateway_daylight_saving_time(self, address):
        received = self.__send_and_rcv(self.HELVAR_GATEWAY_DSTIME.format(address))
        received = re.search("(?<=\=).*(?=#)", str(received)).group()
        return received     
    def query_network_routers(self, cluster):
        received = self.__send_and_rcv(self.QUERY_ROUTERS.format(cluster))
        received = re.search("(?<=\=).*(?=#)", str(received)).group()
        return received   
    def query_network_clusters(self):
        received = self.__send_and_rcv(self.QUERY_CLUSTERS.format())
        received = re.search("(?<=\=).*(?=#)", str(received)).group()
        return received            
    def get_gateway_lattitude(self, address):
        received = self.__send_and_rcv(self.HELVAR_GATEWAY_LAT.format(address))
        received = re.search("(?<=\=).*(?=#)", str(received)).group()
        return received 
    def get_gateway_longditude(self, address):
        received = self.__send_and_rcv(self.HELVAR_GATEWAY_LON.format(address))
        received = re.search("(?<=\=).*(?=#)", str(received)).group()
        return received           
    def __send(self, str):
        self.__lock.acquire()
        attempt = self.__retry_no
        while attempt:
            try:
                self.__s.send(str.encode())
                break
            except socket.error:
                attempt -= 1
        if [ attempt == 0 ]:                                        # all retries failed then try re-establishing connection
            try:
                self.__s.connect((self.__ip, self.__port))
                self.__s.send(str.encode()) 
            except socket.error:
                print("error on send to helvar gateway")                
        self.__lock.release()
    def __send_and_rcv(self, str):
        self.__lock.acquire()
        self.__s.send(str.encode())
        self.__s.settimeout(10)
        attempt = self.__retry_no
        while attempt:
            try:
                self.__s.send(str.encode())
                break
            except socket.error:
                attempt -= 1
        if [ attempt == 0 ]:                                        # all retries failed then try re-establishing connection
            try:
                self.__s.connect((self.__ip, self.__port))
                self.__s.send(str.encode()) 
            except socket.error:
                print("error on send to helvar gateway") 
                return None                 
        try:   
            data = self.__s.recv(1024) 
            return data  
        except socket.error:
            print("error on recv from helvar gateway")            
            return None                
        self.__lock.release()
    def __keepalive(self):
        self.__send('')
    def __t_func(self, weakself, _):
        while (self.__running):
            time.sleep(1)
            if weakself is None:
                break
            self.__keepalive()
    def __exit__(self):
        self.__running = False
        self.close()
    def __del__(self):
        """Graceful shutdown."""
        self.__running = False
        self.close()
    def close(self):
        """Close UDP socket."""
        self.__s.close()

        
class LedUnit:
    def __init__(self, helvarNet, address):
        self.__net = helvarNet
        self.__addr = address
    def set_device_lvl(self, level, fade_time = 0):
        self.__net.set_direct_level(self.__addr, level, fade_time)
    def recall_scene_device(self, block, scene, fade): 
        self.__net.recall_scene_on_device(self.__addr, block, scene, fade)           
    def store_scene_for_device(self, force: bool, block, scene, level): 
        self.__net.store_scene_for_device(self.__addr, force: bool, block, scene, level)  
    def store_curr_scene_for_device(self, force: bool, block, scene): 
        self.__net.store_curr_scene_for_device(self.__addr, force: bool, block, scene)     
    def reset_device_elamp_bat_time(self):
        self.__net.reset_device_elamp_bat_time(self.__addr)    
    def get_device_power_cons(self):
        pc = self.__net.get_power_cons_device(self.__addr)
        return pc
        
class LedGroup:
    def __init__(self, helvarNet, group):
        self.__net = helvarNet
        self.__grp = group
    def set_group_lvl(self, level, fade_time = 0):
        self.__net.set_group_level(self.__grp, level, fade_time)
    def reset_group_elamp_bat_time(self):
        self.__net.reset_group_elamp_bat_time(self.__grp)     
    def recall_scene_group(self, block, scene, fade): 
        self.__net.recall_scene_on_group(self.__grp, block, scene, fade)            
    def store_scene_for_group(self, force: bool, block, scene, level): 
        self.__net.store_scene_for_group(self.__grp, force: bool, block, scene, level)       
    def store_curr_scene_for_group(self, force: bool, block, scene): 
        self.__net.store_curr_scene_for_group(self.__grp, force: bool, block, scene)     
    def get_group_power_cons(self):
        pc = self.__net.get_power_cons_group(self.__grp)
        return pc     
        
if __name__ == "__main__":
    helvarNet = HelvarNet('10.254.1.2', HELVAR_PORT)                       # connect to the helvar gateway on tcp/ip
    helvarNet.set_router_current_time()                                    # sync the routers time
    helvarNet.set_gateway_daylight_saving(True)                                    # enable daylight saving
 
    # these are individual devices 
    leds = [LedUnit(helvarNet, '1.2.1.1'),
            LedUnit(helvarNet, '1.2.1.2'),
            LedUnit(helvarNet, '1.2.1.3'),
            LedUnit(helvarNet, '1.2.1.4'),
            LedUnit(helvarNet, '1.2.1.5')]

    # these are group devices             
    grps = [ LedGroup(helvarNet, '1'), LedGroup(helvarNet, '2') ]

    # demonstrate store of device scene            
    leds[0].set_device_lvl(60,50)
    block = 1
    scene = 2
    leds[0].store_curr_scene_for_device(True, block, scene)               # save scene to block 1 scene 2
    time.sleep(0.5)
    leds[0].set_device_lvl(0,50)
    leds[0].recall_scene_device(block, scene, 500)                        # recall the scene with 5 seconds fade

    # demonstrate store of group scene            
    grps[1].set_group_lvl(60,50)
    block = 2
    scene = 3
    grps[1].store_curr_scene_for_group(True, block, scene)                 # save scene to block 2 scene 3
    time.sleep(0.5)
    grps[1].set_group_lvl(0,50)
    grps[1].recall_scene_group(block, scene, 500)                          # recall the scene with 5 seconds fade
        
    i = 0
    while (1):                                                           # cycle changing the light levels up and down
        for i in range(0,len(leds)):
            leds[i].set_device_lvl(50, 100)
            time.sleep(0.5)
        for i in range(0,len(grps)):
            grps[i].set_group_lvl(50, 100)
            time.sleep(0.5)
        for i in range(0,len(leds)):
            leds[i].set_device_lvl(0, 100)
            time.sleep(0.5)
        for i in range(0,len(grps)):
            grps[i].set_group_lvl(0, 100)
            time.sleep(0.5)
        for i in range(0,len(leds)): 
            print("--------------------------------------")        
            power_cons = leds[i].get_device_power_cons()
            print("device ",i," power = ",power_cons)
        print(("--------------------------------------") 
        for i in range(0,len(grps)):
            power_cons = grps[i].get_group_power_cons()
            print("device ",i," power = ",power_cons)
        print(("--------------------------------------")        
        
