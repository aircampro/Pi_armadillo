#coding: UTF-8
#
# example of using api on serial to kongo kagaku
#
import sys
from Rcb4BaseLib import Rcb4BaseLib            
import time                   
import datetime

# create a class object for the kongo communications
#
rcb4 = Rcb4BaseLib()      

# series of actions calling the python communicator class
#
def main():
    # open the serial port to the device
    rcb4.open('/dev/ttyAMA0', 115200, 1.3)                               #(portName,baudrate,timeout(s))
    #rcb4.open('/dev/ttyUSB0', 115200, 1.3)

    # Check ACK to device
    if rcb4.checkAcknowledge() == True:
        print ('checkAcknowledge OK')
        print ('Version    -->' ,rcb4.Version)   
    else:
        print ('checkAcknowledge error')
        sys.exit(-1)
		
    id = 1
    sio = 1
    pos = 4500
    frame = 1
    rcb4.setSingleServo(id, sio, pos, frame)                            # set position of servo
    pp=rcb4.getSinglePos(id, sio)
    print("position ",pp)
    dataV = 90
    servo_data_obj = Rcb4BaseLib.ServoData(id,sio,dataV)                # make a servo data object 
    rcb4.setSpeedCmd(servo_data_obj)                                    # set speed == dataV
    dataV2 = 32
    servo_data_obj.itemAdd(id, sio, dataV2)                             # change the data value
    rcb4.setStretchCmd(servo_data_obj)                                  # set Strech to new value
    dataV3 = 6776
    servo_data_obj.itemAdd(id, sio, dataV3)                             # change the data value
    rcb4.setServoPos(servo_data_obj, frame)                             # set Strech to new value
    pp=rcb4.getSinglePos(id,sio)
    print("position ",pp)
    rcb4.setHoldSingleServo(id, sio)
    pp=rcb4.getSinglePos(id,sio)
    print("position ",pp)
    rcb4.setFreeSingleServo(id, sio)
    pp=rcb4.getSinglePos(id, sio)
    print("position ",pp)
    vv=rcb4.getRcb4Voltage()
    print("voltage ",vv)
	
    print ('MotionPlay(1)')
    rcb4.motionPlay(1)  
    to = datetime.timedelta(seconds=1)                           # set time to suspend
    to2 = datetime.timedelta(seconds=4)                          # set time to resume
    start_t = datetime.datetime.now()   
    susp_actv = False
    resume_actv = False    
    while True:    
        motionNum = rcb4.getMotionPlayNum()  
        if motionNum < 0:                    
            print('motion get error mn=',motionNum)
            break
        if motionNum == 0:                    
            print('stop motion or idle')
            break
            
        if ((datetime.datetime.now() - start_t) > to) and not susp_actv:
            rcb4.suspend()
            susp_actv = True
        elif ((datetime.datetime.now() - start_t) > to2) and not resume_actv:
            rcb4.resume()
            resume_actv = True            
        print('play motion -> ',motionNum)
        
        time.sleep(0.1)
		
    rcb4.close()

if __name__ == "__main__":
    main()