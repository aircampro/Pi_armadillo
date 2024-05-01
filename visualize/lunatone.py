#!/usr/bin/env python3
# lunatone dali bridge cleint control class
#
import requests
import json
import time

# the ip address for the lunatone gateway
#
IP_ADD="192.168.0.62"

class LunaToneClient():

    def lunatone_change_state( id, state=True):

        if state == True:	
            data = { "switchable": true }
		else:
            data = { "switchable": false }	
			
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+IP_ADD+"/device/"+id+"/control", data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)

    def lunatone_change_level( id, level=50):

        data = { "dimmable": level}	
			
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+IP_ADD+"/device/"+id+"/control", data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)	

    def lunatone_recall_last( id ):

        data = {"gotoLastActive": true}	
			
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+IP_ADD+"/device/"+id+"/control", data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)	

    def lunatone_recall_scene( id, scene_no=1 ):

        data = {"scene": scene_no }	
			
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+IP_ADD+"/device/"+id+"/control", data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)

    def lunatone_store_scene( id, scene_no=1 ):

        data = {"saveToScene": scene_no }	
			
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+IP_ADD+"/device/"+id+"/control", data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)

    def lunatone_set_color_rgb( id, r=0, g=0.5, b=1.0 ):

        data = { "colorRGB": { "r": r, "g": g, "b": b } }
				
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+IP_ADD+"/device/"+id+"/control", data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)

    def lunatone_set_color_waf( id, w=0, a=0.5, f=1.0 ):

        data = { "colorWAF": { "w": w, "a": a, "f": f } }
				
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+IP_ADD+"/device/"+id+"/control", data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)

    def lunatone_set_color_temp( id, temp_kel=4000 ):

        data = { "colorKelvin": temp_kel }
				
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+IP_ADD+"/device/"+id+"/control", data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)
			
    def lunatone_set_color_coord( id, x=0.432, y=0.1 ):

        data = { "colorXY": { "x": x, "y": y } } 
				
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+IP_ADD+"/device/"+id+"/control", data=senddatajson, headers=headers)
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
    def lunatone_create_all_blink_sequence( tim_del ):

        data = { "name": "blink_slowly", "loop": true, "steps": [ { "type": "features", 
                 "data": { "targets": [{ "type": "broadcast"}], "features": { "switchable": true } }, "delay": tim_del},
                 { "type": "features", "data": { "targets": [{ "type": "broadcast"}], "features": { "switchable": false } }, "delay": tim_del} ], }
				
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+IP_ADD+"/automations/sequence", data=senddatajson, headers=headers)
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
            response = requests.get("http://"+IP_ADD+"/automations/sequence", headers=headers)
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

    def lunatone_start_sequence( s_id ):
				
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+IP_ADD+"/automations/sequence/"+s_id+"/start", headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)

    def lunatone_stop_sequence( s_id ):
				
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+IP_ADD+"/automations/sequence/"+s_id+"/stop", headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)
			
    def lunatone_edit_all_blink_sequence( seq_id ):

        data = { "loop": false, "repeat": 5 }
				
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.put("http://"+IP_ADD+"/automations/sequence/"+seq_id, data=senddatajson, headers=headers)
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
    def lunatone_create_timed_sched_lights_on( g_id=0, hr=6, min=30 ):

        data =  { "name": "lights-on", "targets": [ { "type": "group", "id": g_id } ], "activeWeekdays": { "saturday": false, "sunday": false },
                  "recallMode": "timeOfDay",  "recallTime": { "hour": hr "minute": min }, "action": { "type": "features", "data": { "features": {
                  "switchable": true } } } }
				
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+IP_ADD+"/automations/scheduler", data=senddatajson, headers=headers)
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
            response = requests.get("http://"+IP_ADD+"/automations/schedules", headers=headers)
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
        
    def lunatone_create_timed_sched_lights_off( g_id=0, hr=18, min=30 ):

        data =  { "name": "lights-off", "targets": [ { "type": "group", "id": g_id } ], "activeWeekdays": { "saturday": false, "sunday": false },
                  "recallMode": "timeOfDay",  "recallTime": { "hour": hr "minute": min }, "action": { "type": "features", "data": { "features": {
                  "switchable": false } } } }
				
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.post("http://"+IP_ADD+"/automations/scheduler", data=senddatajson, headers=headers)
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
            response = requests.get("http://"+IP_ADD+"/automations/schedules", headers=headers)
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
                if (s == "lights-off"):                      # our sequence name
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
     def lunatone_edit_timed_sched( seq_id ):

        data = { "activeWeekdays": { "saturday": true } }
				
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.put("http://"+IP_ADD+"/automations/scheduler/"+seq_id, data=senddatajson, headers=headers)
            req.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)
            
    def lunatone_create_circadian(  ):

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
            req = requests.post("http://"+IP_ADD+"/automations/circadian", data=senddatajson, headers=headers)
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
            response = requests.get("http://"+IP_ADD+"/automations/circadians", headers=headers)
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

     def lunatone_edit_circadian( seq_id ):

        data = { "longest": { "day": 21, "month": 6, "steps": [
               { "hour": 0, "colorKelvin": 2700 }, { "hour": 7, "colorKelvin": 3432 }, { "hour": 12, "colorKelvin": 5765 }, { "hour": 17, "colorKelvin": 4001 }, { "hour": 23, "colorKelvin": 2700 } ] } }
				
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Python3"
        }
 
        senddatajson = json.dumps(data).encode("ascii")
        try:
            req = requests.put("http://"+IP_ADD+"/automations/circadian/"+seq_id, data=senddatajson, headers=headers)
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

    lc=LunaToneClient()
    lc.lunatone_change_state(2, True)              # set id 2 to on
    time.sleep(2)    
    lc.lunatone_change_state(2, False)             # set id 2 to off
    
    # create a sequence to blink every 1/2 second and activate it
    readback_id, readback_state = lc.lunatone_create_all_blink_sequence(0.5)
    if readback_state == "false":
        lc.lunatone_start_sequence(readback_id) 
    time.sleep(120)
    lc.lunatone_stop_sequence(readback_id)    
