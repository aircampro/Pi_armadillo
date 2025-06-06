#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# This module provides a class that communicates with payter apollo terminals
# over serial using Payter Session Protocol (PSP)
# It need pySerial(http://pyserial.sourceforge.net/)
# 
# ref:- https://docs.payter.com/payter-session-protocol-psp-specification/v1/troubleshooting-article
#
import serial
import serial.tools.list_ports
from enum import Enum
import socket
import sys
# for random id generation uncomment ---> import numpy as np
 
# chooses the message to match with the reply message matches the list order of poss_replies
class ReplyMessage(Enum):
    OK_NOK = 0
    SYNC = 1
    CANCEL = 2
    PROTO = 3
    NOK = 4
    DECLINE = 5 
    STAT_OFF = 6 
    STAT_RUN = 7    
    DATA_READ = 100

# state engine for parsing card data
class CardHashCollector(Enum):
    NOTSTARTED = 0
    STARTING = 1
    STARTING2 = 2
    STARTED = 3
    ENDING = 4
    ENDING2 = 5
    ENDED = 6

class TerminalType(Enum):
    APOLLO = 0
    P6X = 1

class CheckumType(Enum):
    ADDITION = 0
    XORING = 1

# functions to provide IP functionality
def connectTCP(host, port):
    TCP_client = socket.socket(socket.AF_INET,socket.SOCK_STREAM)   
    TCP_client.settimeout(10)
    try:
        TCP_client.connect((host, port))
        return TCP_client
    except socket.error:
        TCP_client.close()
        return -1
    
def connectUDP(host, port):
    UDP_client = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)   
    UDP_client.settimeout(10)   
    try:
        UDP_client.connect((host, port))
        return UDP_client
    except socket.error:
        UDP_client.close()
        return -1

def disconnectTCP(TCP_client):
    TCP_client.close()

def disconnectUDP(UDP_client):
    UDP_client.close()

# -------------- class for communication with payter terminals on psp protocol -----------------------
#    
class PayterPSP(object):

    def __init__(self, ttype=TerminalType.APOLLO.value, p6xbyte=0x3C, hst="192.168.0.1", prtno=3201):
        self.myserial = serial.Serial()
        self.mytcp = "not_used"
        self.myudp = "not_used"
        self.comm_method = "serial"
        #print('Generated the serial object for the Payter PSP over serial')
        # Magic Byte
        # For P6X
        # The magic byte can be set to any value for example 0x3C. The terminal will ignore it and will place a configurable value in the location.
        # For APOLLO
        # The magic byte is always 0x3C
        #    
        self.APOLLOMAGIC_NO = 0x3C
        # Preamble Byte
        # The preamble byte is used to provide a device address.
        #
        # For P6X
        # A device will put a configurable address byte here.
        #
        # For APOLLO
        # The preamble to the terminal is 0xCC
        # The preamble to the host is 0xAA
        self.TPRE_APOLLO = 0xCC
        self.HPRE_APOLLO = 0xAA
        # The Apollo terminal has a server-side socket open that accepts an incoming connection on its IP address 
        # or hostname using the default port: 3201
        self.PT_HOST = hst
        self.PT_PORT = prtno
        if ttype == TerminalType.APOLLO.value:
            self.PreByteT = self.TPRE_APOLLO
            self.PreByteH = self.HPRE_APOLLO       
            self.magic = self.APOLLOMAGIC_NO
        elif ttype == TerminalType.P6X.value:      
            self.PreByteT = p6xbyte
            self.PreByteH = p6xbyte       
            self.magic = p6xbyte
            
    def __del__(self):
        if self.comm_method = "serial":
            self.myserial.close()
        elif self.comm_method = "tcp":            
            disconnectTCP(self.mytcp)
        elif self.comm_method = "tcp":            
            disconnectUDP(self.myudp)
            
    def search_com_port(self):
        coms = serial.tools.list_ports.comports()
        list_com = []
        for com in coms:
            list_com.append(com.device)
        #print('Connected COM ports: ' + str(list_com))
        used_port = list_com[0]
        #print('Using first listed COM port: ' + used_port)
        return used_port
    
    def open_port(self, port=None, baudrate=57600, timeout=1):
        ret = 0
        if port == "tcp":
            self.mytcp = connectTCP(self.PT_HOST, self.PT_PORT)
            if not self.mytcp == -1:
                self.comm_method = "tcp"
                #print('Changed the object for the Payter PSP to over tcp')
            else:
                print("cannot open tcp port")     
                ret = -11                
        elif port == "udp":
            self.myudp = connectUDP(self.PT_HOST, self.PT_PORT)    
            if not self.myudp == -1:            
                self.comm_method = "udp" 
                print('Changed the object for the Payter PSP to over tcp')
            else:
                print("cannot open udp port") 
                ret = -12                
        elif port == None:
            port = self.search_com_port()	            
            self.myserial.port = port
            self.myserial.baudrate = baudrate
            self.myserial.timeout = timeout
            self.myserial.parity = serial.PARITY_NONE
            try:
                self.myserial.open()
            except IOError:
                ret = -13
                #raise IOError('Failed to open port, check the device and port number')
            else:
                print('Succeeded to open port: ' + port)
                self.comm_method = "serial"
        else:
            self.myserial.port = port
            self.myserial.baudrate = baudrate
            self.myserial.timeout = timeout
            self.myserial.parity = serial.PARITY_NONE
            try:
                self.myserial.open()
            except IOError:
                ret = -13
                #raise IOError('Failed to open port, check the device and port number')
            else:
                print('Succeeded to open port: ' + port)
                self.comm_method = "serial"            
        return ret
        
    def close_port(self):
        if self.comm_method = "serial":
            self.myserial.close()
        elif self.comm_method = "tcp":            
            disconnectTCP(self.mytcp)
        elif self.comm_method = "tcp":            
            disconnectUDP(self.myudp)

    # set up the port (serial only)
    def set_port(self, baudrate=57600, timeout=0x01):
        self.myserial.baudrate = baudrate
        self.myserial.timeout = timeout
        self.myserial._reconfigurePort()
        #print('Succeeded to set baudrate:%d, timeout:%d' % (baudrate, timeout))

    def enable_terminal(self, return_packet=0x01):
        self._check_range(return_packet, 0, 1, 'return_packet')

        # CCh 02h 3Ch 12h 1Ch
        send = [self.PreByteT,
                0x02,
                self.magic,
                0x12,
                0x1C]
        # send.append(self._calc_checksum(send))
        self._write_command(send)
        if return_packet == 0x00:
            return True
        elif return_packet == 0x01:
            return self._check_ack()

    def disable_terminal(self, return_packet=0x01):
        self._check_range(return_packet, 0, 1, 'return_packet')

        # CCh 02h 3Ch 11h 1Ch
        send = [self.PreByteT,
                0x02,
                self.magic,
                0x11,
                0x1C]
        self._write_command(send)

        if return_packet == 0x00:
            return True
        elif return_packet == 0x01:
            return self._check_ack()

    def reset_terminal(self, return_packet=0x01):
        self._check_range(return_packet, 0, 1, 'return_packet')

        # CCh 02h 3Ch 55h 5Fh
        send = [self.PreByteT,
                0x02,
                self.magic,
                0x55,
                0x5F]
        self._write_command(send)

        if return_packet == 0x00:
            return True
        elif return_packet == 0x01:
            return self._check_ack()            

    def sync_terminal(self, return_packet=0x01):
        self._check_range(return_packet, 0, 1, 'return_packet')

        # CCh 02h 3Ch 24h 2Eh
        send = [self.PreByteT,
                0x02,
                self.magic,
                0x24,
                0x2E]
        self._write_command(send)

        if return_packet == 0x00:
            return True
        elif return_packet == 0x01:
            return self._check_ack(ReplyMessage.SYNC.value)

    # Set protocol version 3 with DOL Authorization enabled
    def set_protocol(self, return_packet=0x01):
        self._check_range(return_packet, 0, 1, 'return_packet')

        # CCh 07h 3Ch 13h 03h 00h 00h 00h 02h 27h
        send = [self.PreByteT,
               0x07,
               self.magic,
               0x13,
               0x03,
               0x00,
               0x00,
               0x00,
               0x02,
               0x27]
        self._write_command(send)

        if return_packet == 0x00:
            return True
        elif return_packet == 0x01:
            return self._check_ack(ReplyMessage.PROTO.value)

    # Setup for 48 hour session time, with commit on timeout.
    def set_up(self, return_packet=0x01):
        self._check_range(return_packet, 0, 1, 'return_packet')

        # CCh 07h 3Ch 23h 00h 02h A3h 00h 00h D7h
        send = [self.PreByteT,
                0x07,
                self.magic,
                0x23,
                0x00,
                0x02,
                0xA3,
                0x00,
                0x00,
                0xD7]
        self._write_command(send)

        if return_packet == 0x00:
            return True
        elif return_packet == 0x01:
            return self._check_ack()           

    # Start session for 2500 cents with session ref 10 and default card data to return.
    def start_sess(self, money_val, ref_no, return_packet=0x01):
        self._check_range(return_packet, 0, 1, 'return_packet')

        # Start session for 2500 cents with session ref 10 and default card data to return
        # CCh 0Ah 3Ch 34h 00h 00h 09h C4h 00h 00h 00h 0Ah 1Dh - TBD verify checksum is 1D
        send = [self.PreByteT,
                0x0A,
                self.magic,
                0x34,
                0x00,
                0x00,
                (money_val & 0xFF00) >> 8,
                money_val & 0xFF,
                0x00,
                0x00,
                0x00,
                ref_no]
        send.append(self._calc_checksum(send))  # -- check if the last byte is correct checksum !
        self._write_command(send)

        if return_packet == 0x00:
            return True
        elif return_packet == 0x01:
            dr = self._check_ack(ReplyMessage.DATA_READ.value)
            return not self._check_ack(ReplyMessage.NOK.value,2,dr) 

    # AAh xxh 3Ch 31h "(User ref)" xxh DFh F0h 06h xxh "(CARDHASH)" DFh CAh 0Bh xxh "(masked PAN)" xxh
    #
    # AAh 41h 3Ch 31h 00h 00h 00h 0Bh 3Ah DFh F0h 06h 28h 33h 30h 31h 32h 30h 34h 31h 30h 39h 32h 42h 45h 42h 
    # 41h 46h 32h 33h 41h 46h 43h 31h 44h 41h 31h 37h 35h 42h 31h 46h 33h 41h 35h 32h 39h 45h 36h 46h 32h 44h 
    # 31h DFh CAh 0Bh 0Ah 30h 31h 30h 30h 30h 30h 30h 30h 31h 38h 3Fh
    def parse_sess_data(self, sess_reply):  
        id = sess_reply[4:][:4]                                                                                          # 4 bytes after 4th byte is the session id
        ln=len(sess_reply)
        state = CardHashCollector.NOTSTARTED.value
        cardhash = []
        maskedpan = []
        for i in range(0,ln):
            if ord(sess_reply[i]) == 0xDF:                                                                                # cardhash first collector byte
                state = CardHashCollector.STARTING.value
            elif state == CardHashCollector.STARTING.value and ord(sess_reply[i]) == 0xF0:                                # got second collector byte
                state = CardHashCollector.STARTING2.value
            elif state == CardHashCollector.STARTING2.value and ord(sess_reply[i]) == 0x06:                               # got third collector byte
                state = CardHashCollector.STARTED.value
            elif state == CardHashCollector.STARTED.value and ord(sess_reply[i]) == 0xDF:                                 # got first end collector byte
                state = CardHashCollector.ENDING.value
            elif state == CardHashCollector.ENDING.value and ord(sess_reply[i]) == 0xCA:                                  # got second collector byte end collection  
                state = CardHashCollector.ENDING2.value
            elif state == CardHashCollector.ENDING2.value and ord(sess_reply[i]) == 0x0B:                                 # got third collector byte end collection  
                state = CardHashCollector.ENDED.value
            elif state == CardHashCollector.ENDING.value :                                                                # got first end but not second byte place last 2 bytes into the cardhash
                append.cardhash(sess_reply[i-1])
                append.cardhash(sess_reply[i])
                state = CardHashCollector.STARTED.value
            elif state == CardHashCollector.ENDING2.value :                                                                # got first & second byte but not third place last 3 bytes into the cardhash
                append.cardhash(sess_reply[i-2])
                append.cardhash(sess_reply[i-1])
                append.cardhash(sess_reply[i])
                state = CardHashCollector.STARTED.value
            elif state == CardHashCollector.STARTED.value and not ord(sess_reply[i]) == 0xDF:                              # read the byte into the cardhash
                append.cardhash(sess_reply[i])
            elif state == CardHashCollector.ENDED.value :  
                append.maskedpan(sess_reply[i])
        return id, cardhash, maskedpan
        
       # This binary didnt seem to add up i cant see the 100 cent so i dont know ?
     def commit_sess(self, money_val, ref_no, return_packet=0x01):
        self._check_range(return_packet, 0, 1, 'return_packet')

        # Commit session for 100 cents with session id 10
        # CCh 0Ah 3Ch 35h 00h 00h 00h 04h 00h 00h 00h 6Ah B5h
        send = [self.PreByteT,
                0x0A,
                self.magic,
                0x35,
                0x00,
                0x00,
                0x00,
                0x04,
                0x00,
                0x00,
                0x00,
                0x6A,
                0xB5]
        #send.append(self._calc_checksum(send))  # TBD add checksum when you know how to form this message from money_val and ref_no ??
        self._write_command(send)

        if return_packet == 0x00:
            return True
        elif return_packet == 0x01:
            return self._check_ack()
            
    def cancel_sess(self, return_packet=0x01):
        self._check_range(return_packet, 0, 1, 'return_packet')

        # CCh 02h 3Ch 37h 41h
        send = [self.PreByteT,
                0x02,
                self.magic,
                0x37,
                0x41]
        self._write_command(send)

        if return_packet == 0x00:
            return True
        elif return_packet == 0x01:
            return self._check_ack(ReplyMessage.CANCEL.value)

    def auth_sess(self, return_packet=0x01):
        self._check_range(return_packet, 0, 1, 'return_packet')

        # CCh 02h 3Ch 36h 40h
        send = [self.PreByteT,
               0x02,
               self.magic,
               0x36,
               0x40]
        self._write_command(send)

        if return_packet == 0x00:
            return True
        elif return_packet == 0x01:
            dr = self._check_ack(ReplyMessage.DATA_READ.value)
            a = self._check_ack(ReplyMessage.NOK.value,2,dr)
            b = self._check_ack(ReplyMessage.DECLINE.value,2,dr)
            if a == True or b == True :
                return False
            else:
                return dr
                
    def void_sess(self, sess_id=10, return_packet=0x01):
        self._check_range(return_packet, 0, 1, 'return_packet')

        # Void session ref 10
        # CCh 06h 3Ch 33h 00h 00h 00h 0Ah 4Bh
        send = [self.PreByteT,
                0x06,
                self.magic,
                0x33,
                0x00,
                0x00,
                0x00,                
                sess_id]
        send.append(self._calc_checksum(send))
        self._write_command(send)

        if return_packet == 0x00:
            return True
        elif return_packet == 0x01:
            return self._check_ack()

    def get_term_status(self, return_packet=0x01):
        #CCh 02h 3Ch 26h 30h
        send = [self.PreByteT,
                0x02,
                self.magic,
                0x26,
                0x30]
        self._write_command(send)

        if return_packet == 0x00:
            return True
        elif return_packet == 0x01:
            ret = False
            if self._check_ack(ReplyMessage.STAT_RUN.value) == True:
                #print("WAIT_CARD & Transaction runnning")
                ret = True
            else:
                if self._check_ack(ReplyMessage.STAT_OFF.value) == True:
                    print("Offline & No transaction runnning")
                else:        
                    print("Invalid ststus returned")  
        return ret
            
    # Send ppse to proprietary card
    def send_ppse(self, return_packet=0x01):
        self._check_range(return_packet, 0, 1, 'return_packet')

        # CCh 16h 3Ch 38h 00h A4h 04h 00h 0Eh 32h 50h 41h 59h 2Eh 53h 59h 53h 2Eh 44h 44h 46h 30h 31h 00h B2h
        send = [self.PreByteT,
                0x16,
                self.magic,
                0x38,
                0x00,
                0xA4,
                0x04,
                0x00,
                0x0E,
                0x32,
                0x50,
                0x41,
                0x59,
                0x2E,
                0x53,
                0x59,
                0x53,
                0x2E,
                0x44,
                0x44,
                0x46,
                0x30,
                0x31,
                0x00,
                0xB2]
        self._write_command(send)

        if return_packet == 0x00:
            return True
        elif return_packet == 0x01:
            return self._check_ack()
            
    # The following functions are provided for use in PRS class
    # A checksum byte must be sent at the end of each command. The checksum byte is a checksum 
    # calculated by adding all bytes in the block together. The checksum byte is not included in 	
    # the summation. The carry bit for checksum additions is ignored since the checksum byte is limited to eight bits   
    # therefore the mode is ADDITION starting at 0 in the message array (default)
    #    
    def _calc_checksum(self, send, j=0, mode=CheckumType.ADDITION.value):
        checksum = send[j]
        for i in range(j+1, len(send)):
            if mode == CheckumType.ADDITION.value:
                checksum += send[i]
            elif mode == CheckumType.XORING.value:
                checksum ^= send[i]
        if mode == CheckumType.ADDITION.value:
            checksum = checksum & 0xFF
        return checksum

    def _check_range(self, value, lower_range, upper_range, name='value'):
        if value < lower_range or value > upper_range:
            raise ValueError(name + ' must be set in the range from ' + str(lower_range) + ' to ' + str(upper_range))

    # checks the ack message from either a read or a message array
    def _check_ack(self, type=ReplyMessage.OK_NOK.value, readp=0, read_val):
        if readp == 0:
            if self.comm_method = "tcp": 
                receive = self.mytcp.recv(1024)
            elif self.comm_method = "udp": 
                receive = self.myudp.recv(1024)  
            elif self.comm_method = "serial":            
                if self.myserial.in_waiting >0: receive = self.myserial.read_all()
        else:
            receive = read_val       
            length = len(receive)
        ok = [ self.PreByteH, 0x03, self.magic, 0x00, 0x00, 0xE9 ]
        nok = [ self.PreByteH, 0x03, self.magic, 0x00, 0x01, 0xEA ]  
        sync = [ self.PreByteH, 0x02, self.magic, 0x24, 0x0C ]  
        # card read AAh xxh 3Ch 34h xxh DFh F0h 06h "(CARDHASH)" DFh CAh 0Bh "(masked PAN)" xxh
        #
        # session appr AAh xxh 3Ch 31h "(User ref)" xxh DFh F0h 06h xxh "(CARDHASH)" DFh CAh 0Bh xxh "(masked PAN)" xxh
        #
        sess_canc = [ self.PreByteH, 0x02, self.magic, 0x33, 0x1B ]
        sess_decl = [ self.PreByteH, 0x02, self.magic, 0x32, 0x1A ]
        proto = [ self.PreByteH, 0x07, self.magic, 0x13, 0x03, 0x00, 0x00, 0x00, 0x02, 0x05 ]
        stat_offline = [ self.PreByteH, 0x04, self.magic, 0x26, 0x01, 0x00, 0x11 ]
        stat_running = [ self.PreByteH, 0x04, self.magic, 0x26, 0x10, 0x01, 0x21 ]
        poss_replies = [ ok, sync, sess_canc, proto, nok, sess_decl, stat_offline, stat_running ]       
        if type == ReplyMessage.DATA_READ.value:  
            return receive       
        else:        
            if length >= 1 :
                for i in range(0,length):
                    ack = ord(receive[i])
                    if not (ack == poss_replies[type][i]):
                        return False
                return True
            else:
                if type == ReplyMessage.NOK.value:
                    return True
                else:                
                    return False

    def _write_command(self, send):
        if self.comm_method = "serial":
            self.myserial.flushOutput()
            self.myserial.flushInput()
            self.myserial.write(bytearray(send))
        elif self.comm_method = "tcp":
            self.mytcp.send(bytearray(send)) 
        elif self.comm_method = "udp":
            self.myudp.send(bytearray(send)) 

# UNIT TEST MODULE :: 
#
def main(mval=100):
    ret = 0
    rpts = 0
    money = mval                                                              # ammount of money in cents
    id = 10                                                                   # session id to use a random e.g. round(np.random.rand()*1000)
    ppsp = PayterPSP()                                                        # create psp communication object
    ret = ppsp.open_port() 
    if ret < 0:                                                               # open serial port
        return ret
    ppsp.set_port()                                                           # set port parmeters
    if not ppsp.reset_terminal() == True:                                     # reset terminal
        print("could not reset the terminal")
        ret = -1
    if not ppsp.sync_terminal() == True:                                      # syncronise terminal
        print("could not syncronise the terminal")  
        ret = -2        
    while rpts < 2:
        if rpts == 0:                                                         # first run try to do transaction
            if (ppsp.enable_terminal() == True):                              # enable terminal mode
                ret, msg = ppsp.start_sess(money, id)                         # start session  
                print("please tap card")                
                if ret == True:
                    id, ch, mp = ppsp.parse_sess_data(msg)                    # parse data returned 
                    print(f"id={id} ch={ch} mp={mp}") 
                    ret, msg = ppsp.auth_sess()                               # autherize session
                    if ret == True:
                        id, ch, mp = ppsp.parse_sess_data(msg)  
                        print(f"id={id} ch={ch} mp={mp}") 
                        if ppsp.commit_sess(money, id) == True:               # commit session
                            print("session comitted")
                        else:
                            print("commit session error")
                            ret = 1
                    else:
                        print("error in auth sess")
                        ret = 2                        
                else:
                    print("error starting session") 
                    ret = 3                    
        else:                                                                  # second run disable the terminal
            if (ppsp.disable_terminal() == True):
                print("terminal disabled")  
            else:
                print("failed to diable the terminal")
                ret = -3                
        rpts += 1        
    ppsp.close_port()	
    # same as del ppsp
    print(f"RETCODE=={ret}")
    return ret
    
# set this parameter True to enable the unit test on library load or False to disable it
LIB_TEST_ON = True	
if __name__ == "__main__":
    if LIB_TEST_ON == True:
        if len(sys.argv) == 1:
            return main()
        else:
            return main(int(sys.argv[1]))