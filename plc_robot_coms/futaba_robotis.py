#! /usr/bin/env python3
# encoding: utf-8
#
# Example of using GS2D library :- https://github.com/karakuri-products/gs2d?tab=readme-ov-file
# To drive Robotis and Futaba drives on 2 separate usb ports
# ref :- 
#
import sys
import asyncio
import logging
import glob

sys.path.insert(0, '../..')
from gs2d import SerialInterface, Futaba, RobotisP20

logging.basicConfig()
logging.getLogger('gs2d').setLevel(level=logging.DEBUG)

async def main(loop):
    try:
        # set up serial interfaces futaba is on lowest number link
        #
        # si = SerialInterface(device='/dev/tty.usbserial-A601X0TE')
        ttydev=glob.glob("/dev/tty*usb*")
        si = SerialInterface(device=ttydev[0])
        futaba = Futaba(si)
        sir = SerialInterface(device=ttydev[1])
        robotis = RobotisP20(sir)
        

        # ~~~~~~ futaba drive id == fsid ~~~~~~
        fsid = 1
        futaba.reset_memory(sid=fsid)
        futaba.set_baud_rate(Futaba.BAUD_RATE_INDEX_115200, sid=fsid)
        futaba.set_limit_cw_position(150, sid=fsid)
        futaba.set_limit_ccw_position(-10, sid=fsid)
        futaba.write_flash_rom(sid=fsid)
        
        futaba.set_torque_enable(False, sid=fsid)
        
        futaba.set_pid_coefficient(105, sid=fsid)
        futaba.set_speed(0, sid=fsid)

        futaba.set_torque_enable(True, sid=fsid)
        futaba.set_target_position(120, sid=fsid)
        await asyncio.sleep(3)
        futaba.set_burst_target_positions({ fsid: -120 })
        futaba.set_target_position(-60, sid=fsid)
        
        v = futaba.get_pid_coefficient(sid=fsid)
        await asyncio.sleep(0.1)
        print("pid ",v)
        
        data = futaba.get_limit_ccw_position(sid=fsid)
        print('limit_ccw_position ', data)
        data = futaba.get_limit_cw_position(sid=fsid)
        print('limit_cw_position ', data)
        data = futaba.get_limit_temperature(sid=fsid)
        print('limit_temperature ', data)
        data = futaba.get_servo_id(sid=fsid)
        print('servo id ', data)
        data = futaba.get_target_position(sid=fsid)
        print('target_position ', data)
        data = await futaba.get_pid_coefficient_async(sid=fsid, loop=loop)
        print('pid_coefficient ', data)

        # ~~~~~~ robotis drive id == rsid ~~~~~~
        rsid = 2
        ping = await robotis.ping_async(rsid)
        print(ping)
        robotis.set_torque_enable(True, sid=rsid)

        await asyncio.sleep(0.3)

        print('Current position:', robotis.get_current_position(rsid))
        print('Enable Torque?:', robotis.get_torque_enable(sid=rsid))

        for i in range(11):
            print('Current position:', robotis.get_current_position(rsid))
            angle = i * 20 - 100
            print('Angle:', angle, 'deg')
            robotis.set_target_position(angle, sid=rsid)
            print('Target position:', robotis.get_target_position(rsid))
            await asyncio.sleep(0.5)

        # set these joints with 3 motors in the sequence of angles sequence            
        sids = [ 3, 4, 5 ]                                                          # list of joints (motors)
        angles = [ [20, 0 , 10], [50. 40. -10], [20, 50, 0] ]                       # sequence of angle movements
        futaba.set_speed(100, sid=fsid)                                             # futaba drive 1 to full speed
        state = 0
        end_state = 6
        deadband = 0.5
        while state < end_state :
            if state == 0:
                for r in range(0,len(sids)):
                    robotis.set_target_position(angles[state][r], sid=sids[r])  
                state = 1
            elif state == 1:
                ok = 0    
                while not ok == 3:   
                    ok = 0                
                    for r in range(0,len(sids)):
                        if (robotis.get_current_position(sid=sids[r]) >= (robotis.get_target_position(rsid=sids[r]-deadband)) :
                            ok += 1
                state = 2
            elif state == 2:
                for r in range(0,len(sids)):
                    robotis.set_target_position(angles[state-1][len(sids)-r-1], sid=sids[len(sids)-r-1]])  # move the joints in reverse sequence order
                state = 3
            elif state == 3:
                ok = 0 
                while not ok == 3:  
                    ok = 0                
                    for r in range(0,len(sids)):
                        if (robotis.get_current_position(sid=sids[r]) >= (robotis.get_target_position(rsid=sids[r]-deadband)) :
                            ok += 1
                state = 4  
            elif state == 4:
                for r in range(0,len(sids)):
                    robotis.set_target_position(angles[state-2][r], sid=sids[r])  
                state = 5
            elif state == 5:
                ok = 0 
                while not ok == 3:    
                    ok = 0                
                    for r in range(0,len(sids)):
                        if (robotis.get_current_position(sid=sids[r]) >= (robotis.get_target_position(rsid=sids[r]-deadband)) :
                            ok += 1
                state = 6
                
        futaba.set_speed(0, sid=fsid)                                                        # now stop the futaba drive
        await asyncio.sleep(2.0)       
        futaba.close(force=False)
        si.close()
        robotis.close(force=False)
        sir.close()
        
    except Exception as e:
        print('Error', e)

if __name__ == '__main__':

    # Initialize event loop
    lp = asyncio.get_event_loop()
    lp.run_until_complete(main(lp))
    lp.close()
