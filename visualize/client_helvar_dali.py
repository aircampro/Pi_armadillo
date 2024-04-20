#!/usr/bin/python3
# helvar net is used to bridge to the dali light networks
#
import socket
import time
import sys
import threading
import weakref
import datetime

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
           
    def __init__(self, ip, port=50000):
        self.__s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__s.connect((ip, port))
        self.__t = threading.Thread(target = self.__t_func,
                                    args = (self, weakref.ref(self)))
        self.__lock = threading.RLock()
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
    def set_daylight_saving(self, ds=True):
        if ds == True:
            self.__day_on = 1
        else:
            self.__day_on = 0
        self.__send(self.DAYLIGHT_SAVING.format(self.__day_on))     
    def set_timezone_diff(self, diff=0):
        self.__timezone_diff = diff
        self.__send(self.TIMEZONE_DIFF.format(self.__timezone_diff))          
    def __send(self, str):
        self.__lock.acquire()
        self.__s.send(str.encode())
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
     
        
if __name__ == "__main__":
    helvarNet = HelvarNet('10.254.1.2', HELVAR_PORT)                       # connect to the helvar gateway on tcp/ip
    helvarNet.set_router_current_time()                                    # sync the routers time
    helvarNet.set_daylight_saving(True)                                    # enable daylight saving
 
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
        