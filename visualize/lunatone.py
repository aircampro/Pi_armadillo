#!/usr/bin/env python3
# lunatone dali bridge cleint control class
#
import requests
import json
import time
import datetime
import pytz

# LunaToneClient : client to the lunatone dali gateway
# ip_addr = the ip address for the lunatone gateway
#
class LunaToneClient:

    def __init__(self,ip_addr="192.168.0.62"):
        self.ip_add = ip_addr
        
    def lunatone_change_state( self, id, state=True):

        if state == True:
            lights_on="\"switchable\": true"        
            data = { lights_on }
		else:
            lights_off="\"switchable\": false" 
            data = { lights_off }	
			
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+self.ip_add+"/device/"+id+"/control", data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)

    def lunatone_change_level( self, id, level=50):

        data = { "dimmable": level}	
			
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+self.ip_add+"/device/"+id+"/control", data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)	

    def lunatone_recall_last( self, id ):

        gtla="\"gotoLastActive\": true"
        data = {gtla}	
			
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+self.ip_add+"/device/"+id+"/control", data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)	

    def lunatone_recall_scene( self, id, scene_no=1 ):

        data = {"scene": scene_no }	
			
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+self.ip_add+"/device/"+id+"/control", data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)

    def lunatone_store_scene( self, id, scene_no=1 ):

        data = {"saveToScene": scene_no }	
			
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+self.ip_add+"/device/"+id+"/control", data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)

    def lunatone_set_color_rgb( self, id, r=0, g=0.5, b=1.0 ):

        data = { "colorRGB": { "r": r, "g": g, "b": b } }
				
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+self.ip_add+"/device/"+id+"/control", data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)

    def lunatone_set_color_waf( self, id, w=0, a=0.5, f=1.0 ):

        data = { "colorWAF": { "w": w, "a": a, "f": f } }
				
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+self.ip_add+"/device/"+id+"/control", data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)

    def lunatone_set_color_temp( self, id, temp_kel=4000 ):

        data = { "colorKelvin": temp_kel }
				
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+self.ip_add+"/device/"+id+"/control", data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)
			
    def lunatone_set_color_coord( self, id, x=0.432, y=0.1 ):

        data = { "colorXY": { "x": x, "y": y } } 
				
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+self.ip_add+"/device/"+id+"/control", data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)
			
    # example of creating an automation sequence to blink the lights
    #
    def lunatone_create_all_blink_sequence( self, tim_del ):

        loo="\"loop\": false"
        lights_on="\"switchable\": true"
        lights_off="\"switchable\": false"
        data = { "name": "blink_slowly", loo, "steps": [ { "type": "features", 
                 "data": { "targets": [{ "type": "broadcast"}], "features": { lights_on } }, "delay": tim_del},
               { "type": "features", "data": { "targets": [{ "type": "broadcast"}], "features": { lights_off } }, "delay": tim_del} ], }
				
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+self.ip_add+"/automations/sequence", data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)

        headers = {'content-type': 'application/json'}
        try:
            response = requests.get("http://"+self.ip_add+"/automations/sequence", headers=headers)
            response.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)
        json_returned = json.dumps(response.json(), indent=4)
        seq_name = json_returned['sequences']['name']        
        seq_id = json_returned['sequences']['steps']['id']
        seq_state = json_returned['sequences']['steps']['active']
        try:
            len(seq_name)                                      # if there is more than one sequence
            idx = 0
            for s in seq_name:
                if (s == "blink_slowly"):                      # our sequence name
                    break
                idx += 1
            seq_id = seq_id[idx]
            seq_state = seq_state[idx]               
        except:
            seq_id = seq_id
            seq_state = seq_state
		return seq_id, seq_state

    def lunatone_start_sequence( self, s_id ):
				
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+self.ip_add+"/automations/sequence/"+s_id+"/start", headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)

    def lunatone_stop_sequence( self, s_id ):
				
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+self.ip_add+"/automations/sequence/"+s_id+"/stop", headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)
			
    def lunatone_edit_all_blink_sequence( self, seq_id ):

        loo="\"loop\": false"
        data = { loo, "repeat": 5 }
				
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.put("http://"+self.ip_add+"/automations/sequence/"+seq_id, data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)
            
    # example of creating timed schedule
    #
    def lunatone_create_timed_sched_lights_on( self, g_id=0, hr=6, min=30 ):

        sat="\"saturday\": false"
        sun="\"sunday\": false"
        s1 = "\"switchable\": false"
        data =  { "name": "lights-on", "targets": [ { "type": "group", "id": g_id } ], "activeWeekdays": { sat, sun },
                  "recallMode": "timeOfDay",  "recallTime": { "hour": hr "minute": min }, "action": { "type": "features", "data": { "features": {
                  s1 } } } }
				
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+self.ip_add+"/automations/scheduler", data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)

        headers = {'content-type': 'application/json'}
        try:
            response = requests.get("http://"+self.ip_add+"/automations/schedules", headers=headers)
            response.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)
        json_returned = json.dumps(response.json(), indent=4)
        sched_name = json_returned['schedulers']['name']
        sched_id = json_returned['schedulers']['id']
        try:
            len(sched_name)                                      # if there is more than one sequence
            idx = 0
            for s in sched_name:
                if (s == "lights-on"):                      # our sequence name
                    break
                idx += 1
            sched_id = sched_id[idx]
            sched_name = "lights-on"               
        except:
            sched_id = sched_id
            sched_name = "lights-on"
		return sched_name, sched_id
        
    def lunatone_create_timed_sched_lights_off( self, g_id=0, hr=18, min=30 ):

        sat="\"saturday\": false"
        sun="\"sunday\": false"
        s1 = "\"switchable\": false"
        data =  { "name": "lights-off", "targets": [ { "type": "group", "id": g_id } ], "activeWeekdays": { sat, sun },
                  "recallMode": "timeOfDay",  "recallTime": { "hour": hr "minute": min }, "action": { "type": "features", "data": { "features": {
                  s1 } } } }
				
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+self.self.ip_add+"/automations/scheduler", data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)

        headers = {'content-type': 'application/json'}
        try:
            response = requests.get("http://"+self.ip_add+"/automations/schedules", headers=headers)
            response.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)
        json_returned = json.dumps(response.json(), indent=4)
        sched_name = json_returned['schedulers']['name']
        sched_id = json_returned['schedulers']['id']
        try:
            len(sched_name)                                      # if there is more than one sequence
            idx = 0
            for s in sched_name:
                if (s == "lights-off"):                          # our sequence name
                    break
                idx += 1
            sched_id = sched_id[idx]
            sched_name = "lights-off"               
        except:
            sched_id = sched_id
            sched_name = "lights-off"
		return sched_name, sched_id

     # example to edit or revise a schedule to include saturday
     #     
     def lunatone_edit_timed_sched( self, seq_id ):

        sat="\"saturday\": true"
        data = { "activeWeekdays": { sat } }
				
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.put("http://"+self.ip_add+"/automations/scheduler/"+seq_id, data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)
            
    def lunatone_create_circadian( self ):

        data = { "targets": [ { "type": "device", "id": 2 } ], "longest": { "day": 21, "month": 6, "steps": [
               { "hour": 0, "colorKelvin": 2700 }, { "hour": 7, "colorKelvin": 3412 }, { "hour": 11, "colorKelvin": 5685 }, { "hour": 16, "colorKelvin": 4101 }, { "hour": 23, "colorKelvin": 2700 } ] },
               "shortest": { "day": 21, "month": 12, "steps": [ { "hour": 0, "colorKelvin": 2700 }, { "hour": 1, "colorKelvin": 2700 },
               { "hour": 7, "colorKelvin": 3765 }, { "hour": 11, "colorKelvin": 5402 }, { "hour": 17, "colorKelvin": 4085 }, { "hour": 23, "colorKelvin": 2700 } ] } }
				
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+self.ip_add+"/automations/circadian", data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)

        headers = {'content-type': 'application/json'}
        try:
            response = requests.get("http://"+self.ip_add+"/automations/circadians", headers=headers)
            response.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)
        json_returned = json.dumps(response.json(), indent=4)
        sched_name = json_returned['circadians']['steps']['enabled']
        sched_id = json_returned['circadians']['steps']['id']
		return sched_name, sched_id     

     def lunatone_edit_circadian( self, seq_id ):

        data = { "longest": { "day": 21, "month": 6, "steps": [
               { "hour": 0, "colorKelvin": 2700 }, { "hour": 7, "colorKelvin": 3432 }, { "hour": 12, "colorKelvin": 5765 }, { "hour": 17, "colorKelvin": 4001 }, { "hour": 23, "colorKelvin": 2700 } ] } }
				
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.put("http://"+self.ip_add+"/automations/circadian/"+seq_id, data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)        

    def get_month_string( self, month):

        if month == 1:
            ret_s = "January"
        elif month == 2:
            ret_s = "Febraury"
        elif month == 3:
            ret_s = "March"
        elif month == 4:
            ret_s = "April"
        elif month == 5:
            ret_s = "May"
        elif month == 6:
            ret_s = "June"
        elif month == 7:
            ret_s = "July"
        elif month == 8:
            ret_s = "August"
        elif month == 9:
            ret_s = "September"
        elif month == 10:
            ret_s = "October"
        elif month == 11:
            ret_s = "November"     
        elif month == 12:
            ret_s = "December" 
        return ret_s
            
    def lunatone_set_time( self, time_z='Europe/Moscow' ):

        dta = datetime.datetime.now(pytz.timezone(time_z))
        dt_a = str(dta).split(' ')                                                               # split the date and time apart
        dt_array = dt_a[0].split('-')
        date_str = dt_array[2]+". "+self.get_month_string(int(dt_array[1]))+" "+dt_array[0]      # construct date string
        tm_a = dt_a[1].split(':')
        time_str = tm_a[0]+":"+tm_a[1]                                                           # construct the time string
        strings1="\"automatic time\" : true"
        data = { "timezone": time_z, strings1, "date": date_str, "time": time_str }
               				
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+self.ip_add+"/datetime", data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)
            
if __name__ == '__main__':

    lc=LunaToneClient("10.0.0.1")
    lc.lunatone_set_time('Europe/Vienna')          # syncronise the time and timezone
    lc.lunatone_change_state(2, True)              # set id 2 to on
    time.sleep(2)    
    lc.lunatone_change_state(2, False)             # set id 2 to off
    
    # create a sequence to blink every 1/2 second and activate it
    readback_id, readback_state = lc.lunatone_create_all_blink_sequence(0.5)
    if readback_state == "false":
        lc.lunatone_start_sequence(readback_id) 
    time.sleep(120)
    lc.lunatone_stop_sequence(readback_id)    
