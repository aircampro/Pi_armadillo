#!/usr/bin/python
#
# IEC61850 cleint ref:- https://github.com/keyvdir/pyiec61850/blob/master/client.py
#
import sys
import time
import threading
import traceback
import signal
import sys
sys.path.insert(0, "libiec61850/pyiec61850")
import iec61850
from datetime import datetime
# siemens s7 driver https://github.com/FiloCara/pyS7
#
from pyS7 import S7Client

def signal_handler(signal, frame):
    global running
    running =0
    print('You pressed Ctrl+C or sent a close signal to this process with kill!')

if __name__=="__main__":
    now = datetime.now();
    current_time = now.strftime("%H:%M:%S");
    print("Starting Client At Time %s" % current_time);

	# Create Client Connection
    if len(sys.argv) > 1:                                  # connect to the client ip can be first arg to script
        server_ip = str(sys.argv[1])
    else:
        server_ip = "localhost"
    conn_port = 8102
    if len(sys.argv) > 2:                                  # read/write to siemens s7 plc can be second arg of script
        s7plc_add = str(sys.argv[2])
    else:
        s7plc_add = "192.168.5.100"
    con = iec61850.IedConnection_create()
    error = iec61850.IedConnection_connect(con, server_ip, conn_port);

    if (error == iec61850.IED_ERROR_OK):
        [deviceList, error] = iec61850.IedConnection_getLogicalDeviceList(con)
        device = iec61850.LinkedList_getNext(deviceList)
        print("Connected to IEC61850 Server.\n")
        while device:                                 		# Show Logical Node, Logical Device and Data Object inside the Server
            LD_name=iec61850.toCharP(device.data)
            print("LD: %s" % LD_name)
            [logicalNodes, error] = iec61850.IedConnection_getLogicalDeviceDirectory(con, LD_name)
            logicalNode = iec61850.LinkedList_getNext(logicalNodes)
            while logicalNode:
                LN_name=iec61850.toCharP(logicalNode.data)
                print(" LN: %s" % LN_name)
                [LNobjects, error] = iec61850.IedConnection_getLogicalNodeVariables(con, LD_name+"/"+LN_name)
                LNobject = iec61850.LinkedList_getNext(LNobjects)
                while LNobject:
                    print("  DO: %s" % iec61850.toCharP(LNobject.data))
                    LNobject = iec61850.LinkedList_getNext(LNobject)
                iec61850.LinkedList_destroy(LNobjects)
                logicalNode = iec61850.LinkedList_getNext(logicalNode)
            iec61850.LinkedList_destroy(logicalNodes)
            device = iec61850.LinkedList_getNext(device)
        iec61850.LinkedList_destroy(deviceList)
        running = 1;
        signal.signal(signal.SIGINT, signal_handler);
        signal.signal(signal.SIGUSR1, signal_handler);
        signal.signal(signal.SIGUSR2, signal_handler);

        # Create a new 'S7Client' object to connect to S7-300/400/1200/1500 PLC.
        # Provide the PLC's IP address and slot/rack information
        client = S7Client(address=s7plc_add, rack=0, slot=1)

        # Establish connection with the PLC
        client.connect()

        # Define area tags to read
        rdtags = [
            "IW22",         # Read WORD at address 22 in input area
            "DB1,S10.5"     # Read sequence of CHAR of length 5 starting at address 10 of DB1
        ]

        wrtags = [
            "QR24"          # => S7Tag(MemoryArea.OUTPUT, 0, DataType.REAL, 24, 0, 1) - REAL at address 24 in output area
        ]
        while (running):
            # Read Data Object
            theVal = "testmodelSENSORS/TTMP1.Temp1.float"
            theValType = iec61850.IEC61850_FC_MX
            value = iec61850.IedConnection_readFloatValue(con, theVal, theValType);
            print("\n Read Value of TTMP1.Temp1.float: %s" % value[0]);
            assert(value[1]==0)
            newValue= value[0]
            set_values = [ value[0] ]
            client.write(tags=wrtags, values=set_values)
            err = iec61850.IedConnection_writeFloatValue(con, theVal, theValType, newValue)
            assert(err==25)
            # Accessing to ASG values
            theVal = "testmodelSENSORS/TTMP1.TmpSp.setMag.f"
            theValType = iec61850.IEC61850_FC_SP
            setpoint = iec61850.IedConnection_readFloatValue(con, theVal, theValType)
            assert(value[1]==0)
            data = client.read(tags=rdtags)                                                                             # Read the data from the PLC using the specified tag list
            newValue= data[0]
            err = iec61850.IedConnection_writeFloatValue(con, theVal, theValType, newValue)                   			# Write Data Object with new value
            assert(err==0)
            setpoint = iec61850.IedConnection_readFloatValue(con, theVal, theValType)                                   # read back and check
            print(" Write Value of TTMP1.Temp1.float: %s" % setpoint[0])
            assert(setpoint[0]==newValue)
            # Read Data Object
            theVal1 = "testmodelSENSORS/TTMP1.Temp1.string"
            theValType1 = iec61850.IEC61850_FC_DC
            value1 = iec61850.IedConnection_readStringValue(con, theVal1, theValType1);
            print(" Read Value of TTMP1.Temp1.string: %s" % value1[0]);
            assert(value1[1]==0)
            newValue1= value1[0]
            err1 = iec61850.IedConnection_writeVisibleStringValue(con, theVal1, theValType1, newValue1)
            assert(err1==21)
            # Accessing to VSG values
            theVal1 = "testmodelSENSORS/TTMP1.TmpSt.setVal"
            theValType1 = iec61850.IEC61850_FC_SP
            setpoint1 = iec61850.IedConnection_readStringValue(con, theVal1, theValType1)
            assert(value1[1]==0)
            #newValue1= "Networked System Lab"
            newValue1= data[1] 
            err1 = iec61850.IedConnection_writeVisibleStringValue(con, theVal1, theValType1, newValue1)
            assert(err1==0)
			# Write Data Object with new value
            setpoint1 = iec61850.IedConnection_readStringValue(con, theVal1, theValType1)
            print(" Write Value of TTMP1.Temp1.string: %s" % setpoint1[0])
            assert(setpoint1[0]==newValue1)
            time.sleep(5)
    else:
        print("Connection error")
        sys.exit(-1)
    iec61850.IedConnection_close(con)
    iec61850.IedConnection_destroy(con)
    print("\n Client Disconnected.")