#!/usr/bin/python3
#
import datetime
import logging

import serial
import numpy as np
import time

import sys

# this allows this code to run on python 2.x and 3.x
from __future__ import print_function

# set to 1 if you want to use encryption
AES_ENCRYPT = 1

#Diffie-Hellman Key Exchange Algorithm
"""
This program tries to implement the standard Deffie-Hellman key exchange algorithm
that mathematically helps the networking entities to derieve the key pairs without
the actual physical key sharing.
"""
import random

class DHKE:
    def __init__(self,G,P):
        self.G_param = G
        self.P_param = P

    def generate_privatekey(self):
        self.pk = random.randrange(start = 1,stop = 10,step = 1)

    def generate_publickey(self):
        self.pub_key = pow(self.G_param,self.pk) % self.P_param

    def exchange_key(self,other_public):
        self.share_key = pow(other_public,self.pk) % self.P_param

# this is a DH Enpoint class for sending encrypted messages using Diffie Hellman 
# the manual for eSSP suggests that in our case
# we use DH for key exchange and instead used AES encrypt/decrypt but if needed it has been included here
# the usage is :-
#     s_public = key generated (g), m_public = modulus generated (p), s_private = shared key received back
#     my_dh_endpoint_client = DH_Endpoint(s_public, m_public, s_private)
#     msg_to_send = my_dh_endpoint_client.encrypt_message(my_data_to_send) <-- msg to send formulation
#     msg_rcv = my_dh_endpoint_client.decrypt_message(my_data_came_in)     <-- msg received decryption
#
class DH_Endpoint(object):
    def __init__(self, public_key1, public_key2, private_key):
        self.public_key1 = public_key1
        self.public_key2 = public_key2
        self.private_key = private_key
        self.full_key = None
    def generate_partial_key(self):
        partial_key = self.public_key1**self.private_key
        partial_key = partial_key%self.public_key2
        return partial_key
    def generate_full_key(self, partial_key_r):
        full_key = partial_key_r**self.private_key
        full_key = full_key%self.public_key2
        self.full_key = full_key
        return full_key
    def encrypt_message(self, message):
        encrypted_message = ""
        key = self.full_key
        for c in message:
            encrypted_message += chr(ord(c)+key)
        return encrypted_message
    def decrypt_message(self, encrypted_message):
        decrypted_message = ""
        key = self.full_key
        for c in encrypted_message:
            decrypted_message += chr(ord(c)-key)
        return decrypted_message
        
class eSSPError(IOError):  # noqa
    """Generic error exception for eSSP problems."""

    pass

class eSSPTimeoutError(eSSPError):  # noqa
    """Indicates a timeout while communicating with the eSSP device."""

    pass


class eSSP(object):  # noqa
    """General class for talking to an eSSP device."""

    def __init__(self, serialport='/dev/ttyUSB0', eSSPId=0, timeout=None):  # noqa
        """
        Initialize a new eSSP object.

        The timeout parameter corresponds to the pySerial timeout parameter,
        but is used a bit different internally. When the parameter isn't set
        to None (blocking, no timeout) or 0, (non-blocking, return directly),
        we set a timeout of 0.1 seconds on the serial port, and perform reads
        until the specified timeout is expired. When the timeout is reached
        before the requested data is read, a eSSPTimeoutError will be raised.
        """
        if timeout is None or timeout == 0:
            serial_timeout = timeout
        else:
            serial_timeout = 0.1
        self.timeout = timeout
        self.__ser = serial.Serial(serialport, 9600, timeout=serial_timeout)
        self.__eSSPId = eSSPId
        self.__sequence = '0x80'

        self._logger = logging.getLogger(__name__)
        self._logger.debug("Startup at " + str(datetime.datetime.now()))

    # smart payout, BANK NOTE VALIDATOR (NV9USB, NV10USB, BV20, BV50, BV100, NV200) 
    def reset(self):
        """Reset the device completely."""
        result = self.send([self.getseq(), '0x1', '0x1'])
        return result

    def set_inhibits(self, lowchannels, highchannels):
        # lowchannels: Channel 1 to 8
        # highchannels: Channel 9 to 16
        # takes a bitmask
        # For more ease: use easy_inhibit() as helper
        result = self.send([self.getseq(), '0x3', '0x2',
                           lowchannels, highchannels])
        return result

    def bulb_on(self):
        """Illuminate bezel."""
        result = self.send([self.getseq(), '0x1', '0x3'])
        return result

    def bulb_off(self):
        """Nox bezel."""
        result = self.send([self.getseq(), '0x1', '0x4'])
        return result

    def setup_request(self):
        # Response consits of
        # Unit-Type (0 = BNV)
        # Firmware-Version
        # Country-Code
        # Value Multiplier
        # Number of channels
        # Channels Value array()
        # Security of Channels array()
        # Real value Multiplier
        # Protocol Version
        # 1 = Low Security
        # 2 = Std Security
        # 3 = High Security
        # 4 = Inhibited
        result = self.send([self.getseq(), '0x1', '0x5'], False)

        unittype = int(result[4], 16)

        fwversion = ''
        for i in range(5, 9):
            fwversion += chr(int(result[i], 16))

        country = ''
        for i in range(9, 12):
            country += chr(int(result[i], 16))

        valuemulti = int('00', 16)
        for i in range(12, 15):
            valuemulti += int(result[i], 16)

        channels = int(result[15], 16)

        values = []
        for i in range(0, channels):
            values.append(int(result[i + 16], 16))

        security = []
        for i in range(0, channels):
            security.append(int(result[i + 16 + channels], 16))

        multiplier = 0
        for i in range(16 + 2 * channels, 16 + 2 * channels + 3):
            multiplier += int(result[i], 16)

        protocol = int(result[16 + 2 * channels + 3], 16)

        unit_data = [unittype, fwversion, country, valuemulti, channels,
                     values, security, multiplier, protocol]

        return unit_data

    # The set host protocol version command (0x06) allows the slave device to know what 
    # protocol it should report certain events in. Later protocols can include extra commands, 
    # extra response data or different response formatting. For this reason the protocol version is 
    # set immediately after the key negotiation so no commands or data are missed.
    # if it is not supported a generic Fail 
    # (0xF8) response will be given.
    #
    def host_protocol(self, host_protocol=0x7):
       result = self.send([self.getseq(), '0x2', '0x6', host_protocol])
       return result

    # ch_end = in binary = 00000111 0x7
    def enable_chan1_3(self, ch_end=0x7, ch_start=0x0):
       result = self.send([self.getseq(), '0x3', '0x2', ch_end, ch_start])
       return result

    def poll(self):
        """
        Poll the device.

        0xF1 = Slave Reset (right after booting up)
        0xEF = Read Note + Channel Number array()
        0xEE = Credit Note + Channel Number array()
        0xED = Rejecting
        0xEC = Rejected
        0xCC = Stacking
        0xEB = Stacked
        0xEA = Safe Jam
        0xE9 = Unsafe Jam
        0xE8 = Disabled
        0xE6 = Fraud attempt + Channel Number array()
        0xE7 = Stacker full
        0xE1 = Note cleared from front at reset (Protocol v3)
                 + Channel Number array()
        0xE2 = Note cleared into cashbox at reset (Protocol v3)
                 + Channel Number array()
        0xE3 = Cash Box Removed (Protocol v3)
        0xE4 = Cash Box Replaced (Protocol v3)
        """
        result = self.send([self.getseq(), '0x1', '0x7'], False)

        poll_data = []
        for i in range(3, int(result[2], 16) + 3):
            if result[i] in ('0xef', '0xee', '0xe6', '0xe1', '0xe2'):
                poll_data.append([result[i], int(result[i + 1], 16)])
                i += 1
            else:
                poll_data.append(result[i])

        return poll_data

    def reject_note(self):
        """Reject the current note."""
        result = self.send([self.getseq(), '0x1', '0x8'])
        return result

    def disable(self):
        """
        Disable the device.

        Will resume to work only when beeing enable()'d again.
        """
        result = self.send([self.getseq(), '0x1', '0x9'])
        return result

    def enable(self):
        """Resume from disable()'d state."""
        result = self.send([self.getseq(), '0x1', '0xA'])
        return result

    def empty(self):
        """command causes the NV11 to empty all its stored notes to the cashbox."""
        result = self.send([self.getseq(), '0x1', '0x3F'])
        return result

    def get_note_positions(self):
        """causes the validator to report the number of notes stored and the value of the note in each position."""
        result = self.send([self.getseq(), '0x1', '0x41'])
        return result
        
    def parse_get_chan_pos(self, dat):
        resp = dat[3] == 0xF0
        num = int(dat[4],16) 
        for zz in range(0,num):
            print("channel ",int(dat[5+zz],16))
        return resp

    def parse_get_note_pos(self, dat):
        resp = dat[3] == 0xF0
        num = int(dat[4],16)                         # the number of notes
        for zz in range(0,(num*4),4):
            note = int(dat[5+zz],16) | int(dat[6+zz],16) << 8 | int(dat[7+zz],16) << 16 | int(dat[8+zz],16) << 24
            print("note ammount ",note)
        return resp
        
    def payout_note(self):
        """causes the validator to payout the next available note stored in the NV11 payout device, this will be the last note that was paid in"""
        result = self.send([self.getseq(), '0x1', '0x42'])
        return result

    def stack_note(self):
        """causes the validator to send the next available note from storage to the cashbox."""
        result = self.send([self.getseq(), '0x1', '0x43'])
        return result

    # type 0=by_value 1=by_channel
    def set_value_reporting_type(self, typ):
        """changes the way the validator reports the values of notes. There are two options, by channel or by value"""
        result = self.send([self.getseq(), '0x2', '0x45', typ])
        return result

    def get_note_counter(self):
        """causes validator to report a set of global note counters that track various note statistics."""
        result = self.send([self.getseq(), '0x1', '0x58'])
        return result
        
    def enable_payout(self):
        """Enables the recycler unit for payout, stacking and storage.
		 If 0xF0 is returned, the recycler is enabled. 
         If 0xF5 (command cannot be processed) is returned, an error code will follow. See 
        the relevant command in section 9.2 for more details on these codes.
        o 0x01 No Note Float connected
        o 0x02 Invalid Currency
        o 0x03 Device busy
        o 0x04 Empty only
        o 0x05 Note float device error
         If the command was sent unencrypted then Parameter Out Of Range (0xF4) will be 
        returned."""
        result = self.send([self.getseq(), '0x2', '0x5C', '0x1'])
        return result

    def disable_payout(self):
        """Enables the recycler unit for payout, stacking and storage.
            If 0xF5 (command cannot be processed) is returned, an error code will follow. See 
           the relevant command in section 9.2 for more details on these codes.
           o 0x01 No Note Float connected
           o 0x03 Device busy
            If the command was sent unencrypted then Parameter Out Of Range (0xF4) will be 
           returned.."""
        result = self.send([self.getseq(), '0x1', '0x5B'])
        return result

    def get_note_positions(self):
        """Used to get the value of each of the notes in the recycler.."""
        result = self.send([self.getseq(), '0x1', '0x41'])
        return result

    def dispense_note(self):
        """Used to dispense the notes in the recycler.."""
        result = self.send([self.getseq(), '0x1', '0x42'])
        return result

    def stack_note(self):
        """Used to stack the notes in the recycler.."""
        result = self.send([self.getseq(), '0x1', '0x43'])
        return result

    def get_serial_number(self):
        """ causes the unit to report its unique serial number """
        result = self.send([self.getseq(), '0x1', '0x0C'])
        if len(result) >= 8:
            return [result[3], int(result[7],16)<<24|int(result[6],16)<<16|int(result[5],16)<<8|int(result[4],16)]           # f4=ok, serial number
        else:
            return [0,0]

    def get_unit_data(self):
        """causes the validator to return information about itself"""
        result = self.send([self.getseq(), '0x1', '0x0D'])
        return result

    def get_chan_value_data(self):
        """causes the validator to return the number of channels it is using followed by the value of each channel."""
        result = self.send([self.getseq(), '0x1', '0x0E'])
        return result

    def get_last_reject_code(self):
        """causes the validator to return the number of channels it is using followed by the value of each channel."""
        result = self.send([self.getseq(), '0x1', '0x17'])
        return result

    def get_bar_code_reader_config(self):
        """causes the validator to return the configuration data for attached bar code readers if there is one present."""
        result = self.send([self.getseq(), '0x1', '0x24'])
        return result
        
    # Enabling both top and bottom barcode readers =3 bot=2 top=1 , interleaved 2 of 5 = 0x1 else 0, Number of characters 0xA=10
    def set_bar_code_reader_config(self, top_bot=0x03, inter=0x0, chars=0x0A):
        """sets up the validator’s bar code reader configuration"""
        result = self.send([self.getseq(), '0x4', '0x23', top_bot, inter, chars])
        return result

    def get_bar_code_inhib(self):
        """ causes validator to get the current bar code/currency inhibit status"""
        result = self.send([self.getseq(), '0x1', '0x25'])
        return result
        
    def set_bar_code_inhib(self, mask):
        """ causes validator to set the current bar code/currency inhibit status"""
        result = self.send([self.getseq(), '0x2', '0x26', mask])
        return result

    def get_bar_code_data(self):
        """ causes validator to return the last valid barcode ticket data"""
        result = self.send([self.getseq(), '0x1', '0x27'])
        return result

    # mode is 1 EEPROM 0 RAM storage
    def conf_bezel(self, red=0xFF, green=0x11, blu=0x7F, mode=1):
        """ sets the colour of the bezel to a specified RGB colour """
        result = self.send([self.getseq(), '0x5', '0x54', red, green, blu, mode])
        return result

    def poll_with_ACK(self):
        """ causes the validator to respond to a poll in the same way as normal 
            but specified events will need to be acknowledged by the host using the EVENT ACK before 
            the validator will allow any further note action. """
        result = self.send([self.getseq(), '0x1', '0x56'])
        return result

    def event_ACK(self):
        """ causes validator to continue with operations after it has been 
            sending a repeating Poll ACK response.. """
        result = self.send([self.getseq(), '0x1', '0x57'])
        return result
        
    def hold_vailidator(self):
        """causes the validator to hold the current accepted note"""
        result = self.send([self.getseq(), '0x1', '0x18'])
        return result
        
    def empty_recycler(self):
        """Used to empty the recycler.."""
        result = self.send([self.getseq(), '0x1', '0x3F'])
        return result

    def get_all_levels(self):
        """command that causes the SMART Payout to report the amount of notes stored for all denominations"""
        result = self.send([self.getseq(), '0x1', '0x22'])
        return result

    def halt_payout(self):
        """command that causes the current payout to stop."""
        result = self.send([self.getseq(), '0x1', '0x38'])
        return result

    def smart_empty(self):
        """causes the validator to empty all its stored notes to the cashbox and also keep a count of the value emptied"""
        result = self.send([self.getseq(), '0x1', '0x52'])
        return result

    def cashbox_payout_operation_data(self):
        """instructs the validator to return the amount emptied from the payout to the cashbox in the last dispense"""
        result = self.send([self.getseq(), '0x1', '0x53'])
        return result

    def get_note_counters(self):
        """causes validator to report a set of global note counters that track various note statistics"""
        result = self.send([self.getseq(), '0x1', '0x58'])
        return result

    def reset_note_counters(self):
        """causes the validator to reset all of its internal note counters to zero"""
        result = self.send([self.getseq(), '0x1', '0x59'])
        return result

    def send_command_calib(self, mode=0):
        """causes the validator to reset all of its internal note counters to zero"""
        if mode == 0:
            md = 0x0            # auto
        else:
            md = 0x01           # command
        result = self.send([self.getseq(), '0x2', '0x47', md])
        return result

    def set_smart_hooper_options(self, cash=1, motor=0, level_chk=1, pay=0):
        """ sets various options on the SMART Hopper. These options are  volatile and will not persist in 
        memory after a reset. This command is only available in firmware 6.04+"""
        reg = cash<<3 | motor<<2 | level_chk<<1 | pay
        result = self.send([self.getseq(), '0x3', '0x50', reg, 0x0])
        return result

    def get_smart_hooper_options(self):
        """ gets various options on the SMART Hopper. These options are  volatile and will not persist in 
        memory after a reset. This command is only available in firmware 6.04+"""
        result = self.send([self.getseq(), '0x1', '0x51'])
        return result

    def coin_mech_global_inhib(self):
        """ gets various options on the SMART Hopper. These options are  volatile and will not persist in 
        memory after a reset. This command is only available in firmware 6.04+"""
        result = self.send([self.getseq(), '0x1', '0x49'])
        return result

    def smart_empty(self):
        """ causes the validator to empty all its stored coins to the cashbox and also keep a count of the value emptied """
        result = self.send([self.getseq(), '0x1', '0x52'])
        return result

    def cashbox_payout_operation_data(self):
        """  instructs the SMART Hopper to return the amount emptied from the payout to the cashbox in the last dispense """
        result = self.send([self.getseq(), '0x1', '0x53'])
        return result

    def coin_mech_options(self, cctalk=0x01):
        """  set options currently you can set Coin Mech error events to 1 = cctalk format or 0 = Coin mech jam and Coin return mech open only"""
        result = self.send([self.getseq(), '0x2', '0x5A', cctalk])
        return result
        
    def dec2snd(num):
        """convert the decimal currency value into bytes which we send in the right order """
        s=str(hex(num))
        if s[4:6] == '':
            s[4:6] = 0x0
        return ([s[4:6], s[2:4])

    def dec2snd4(num):
        """convert the decimal currency value into bytes which we send in the right order """
        s=str(hex(num))
        if s[4:6] == '':
            s[4:6] = 0x0
        if s[6:8] == '':
            s[6:8] = 0x0
        if s[8:10] == '':
            s[8:10] = 0x0
        return [[s[8:10], s[6:8], [s[4:6], s[2:4]]

    def float_ammount(self, note1amt, note2amt, mode=1):
        """causes the validator to keep a set amount “floating” in the payout and specifies a minimum payout value"""
        if (mode == 1):
            byte_pay = [0x58]
        else:
            byte_pay = [0x19]
        denom1 = dec2snd4(note1amt*100)
        denom2 = dec2snd4(note2amt*100)
        end_denom = [0x45, 0x55, 0x52]       # currency The country code when converted to ASCII characters is EUR
        s_arr = [self.getseq(), '0x12', '0x3D'] + denom1 + denom2 + end_denom + byte_pay
        lb=hex(len(s_arr)-2)	
        s_arr[1]=lb		                                   # message length
        result = self.send(s_arr)
        return result

    def get_routing_chan(self, chan=0x1):
        """causes the validator to return the routing for a specific channel."""
        end_denom = [0x45, 0x55, 0x52]       # currency The country code when converted to ASCII characters is EUR
        s_arr = [self.getseq(), '0x05', '0x3C'] + [chan] + end_denom 	
        lb=hex(len(s_arr)-2)	
        s_arr[1]=lb		                                   # message length        
        result = self.send(s_arr)       
        return result

    # takes dat as result from above,,,, 1= not recycle 0= recycle
    def parse_routing_data(self, dat):
        reponse = dat[3] == 0xF0                          # true if good data
        route = dat[4] & 0xFF
        return (reponse, route)
        
    def set_routing_chan(self, recyc=0x1, ch=0x1):
        """causes the validator to return the routing for a specific channel"""
        end_denom = [0x45, 0x55, 0x52]                     # currency The country code when converted to ASCII characters is EUR
        s_arr = [self.getseq(), '0x05', '0x3B'] + [recyc] + [ch] + end_denom 	
        lb=hex(len(s_arr)-2)	
        s_arr[1]=lb		                                   # message length        
        result = self.send(s_arr)
        return result

    def get_routing_note(self, amt):
        """causes the validator to return the routing for a specific note/ """
        end_denom = [0x45, 0x55, 0x52]                     # currency The country code when converted to ASCII characters is EUR
        denom1 = dec2snd4(amt*100)
        s_arr = [self.getseq(), '0x05', '0x3C'] + denom1 + end_denom 	
        lb=hex(len(s_arr)-2)	
        s_arr[1]=lb		                                   # message length        
        result = self.send(s_arr)
        return result

    def set_routing_note(self, recyc=0x1, amt):
        """causes the validator to return the routing for a specific note"""
        end_denom = [0x45, 0x55, 0x52]                     # currency The country code when converted to ASCII characters is EUR
        denom1 = dec2snd4(amt*100)
        s_arr = [self.getseq(), '0x05', '0x3B'] + [recyc] + denom1 + end_denom 	
        lb=hex(len(s_arr)-2)	
        s_arr[1]=lb		                                   # message length        
        result = self.send(s_arr)
        return result
        
    def set_coin_mech_inhibits(self, coinamt, inh=1):
        """causes the SMART Hopper to disable or enable acceptance of individual coin denominations by an attached coin mechanism"""
        if (inh == 1):
            byte_inh = [0x01]
        else:
            byte_inh = [0x00]
        tup = dec2snd(coinamt)
        denom1 = [ tup[0], tup[1] ]
        end_denom = [0x45, 0x55, 0x52]                      # currency The country code when converted to ASCII characters is EUR
        s_arr = [self.getseq(), '0x07', '0x40'] + byte_inh + denom1 + end_denom 
        result = self.send(s_arr)
        return result
        
    def get_min_payout(self):
        """causes the validator to report its current minimum payout of a specific currency"""
        end_denom = [0x45, 0x55, 0x52]                      # currency The country code when converted to ASCII characters is EUR
        s_arr = [self.getseq(), '0x12', '0x3E'] + end_denom 
        lb=hex(len(s_arr)-2)	
        s_arr[1]=lb		                                    # message length
        result = self.send(s_arr)
        return result

    def set_refill_mode(self, write=1, refil=1):
        """Five or six byte command sequence which causes the payout to change or report its refill mode"""
        if write == 1:
            md = [0x11]                                      # write mode on
        else:
            md - [0x01]    
        if refil == 1:
            rf = [0x1]
        else:
            rf = [0x0]        
        end_denom = [0x05, 0x81, 0x10]       
        s_arr = [self.getseq(), '0x12', '0x30'] + end_denom + md + rf 
        lb=hex(len(s_arr)-2)	
        s_arr[1]=lb		                                   # message length
        result = self.send(s_arr)
        return result

    # ------------- Ciphers / Cryptography -----------------
    # AES 128 bit encrtption is used when encryption has been enabled.
    if AES_ENCRYPT == 1:
        from Crypto.Cipher import AES
        from Crypto.Random import get_random_bytes
            
        def aes_gcm_encrypt(key, iv, text):
            cipher = AES.new(key, AES.MODE_GCM, nonce=iv)
            ciphertext, mac = cipher.encrypt_and_digest(text)
            return ciphertext, mac
        
        def aes_gcm_decrypt(key, iv, ciphertext, mac):
            plaintext = 0
            cipher = AES.new(key, AES.MODE_GCM, nonce=iv)
            try:
                plaintext = cipher.decrypt_and_verify(ciphertext,mac)
            except (ValueError, KeyError):
                print("Incorrect decryption")
            return plaintext
        
        def send_rcv_with_aes(msgdata, key, nonce):
            if key == None:
                key = get_random_bytes(16)
            if nonce == None:                
                nonce = get_random_bytes(12)
            ciphertext, mac = self.aes_gcm_encrypt(key, nonce, msgdata)
            indata = self.send((ciphertext)
            return self.aes_gcm_decrypt(key, nonce, indata, mac)

        def payout_by_denomination_with_aes(self, key, nonce, note_amt_list, mode=1):
            """which allows the user to specify exactly which notes are paid out.
               this routine is when encryption has been enabled using AES """
            i = 0
            end_currency = [0x45, 0x55, 0x52]                                   # currency The country code when converted to ASCII characters is EUR
            # end_currency = [hex(ord("C")), hex(ord("H")), hex(ord("F"))]      # uncomment to swap to swiss franks
            tot_num = hex(i)
            if (mode == 1):
                byte_pay = [0x58]
            else:
                byte_pay = [0x19]
            send_arr = [self.getseq(), '0x12', '0x46', tot_num]
            for noteamt, note_num in note_amt_list:
                tupv = dec2snd(noteamt*100)
                denom = [tupv[0], tupv[1], 0x00, 0x00]
                num = [hex(note_num)]
                send_arr = send_arr + num + demon + end_currency	
                i = i + 1	
            send_arr = send_arr + byte_pay
            tot_num = hex(i)                                       # number of different notes
            send_arr[3]=tot_num		
            lb=hex(len(send_arr)-2)	
            send_arr[1]=lb		                                   # message length
            result = self.send_rcv_with_aes(send_arr, key, nonce)
            return result

        def payout_by_value_with_aes(self, key, nonce, noteamt, mode=1):
            """command that instructs the payout device to payout a specified amount"""
            end_currency = [0x45, 0x55, 0x52]                                   # currency The country code when converted to ASCII characters is EUR
            # end_currency = [hex(ord("C")), hex(ord("H")), hex(ord("F"))]      # uncomment to swap to swiss franks
            # end_currency = [hex(ord("R")), hex(ord("U")), hex(ord("B"))]      # uncomment to swap to rubles
            if (mode == 1):
                byte_pay = [0x58]
            else:
                byte_pay = [0x19]
            send_arr = [self.getseq(), '0x12', '0x33']
            arr4_value = dec2snd4(noteamt*100)
            send_arr = send_arr + arr4_value + end_currency + byte_pay	
            lb=hex(len(send_arr)-2)	
            send_arr[1]=lb		                                   # message length
            result = self.send_rcv_with_aes(send_arr, key, nonce)
            return result
            
    # Diffie - Hellman key exchange

    # send the dh key exchange
    def send_dh_key_exchange(self, pub_key):
        msg = [ self.getseq(), '0x9', '0x4C' ]
        for z in range(0, 8):
            byt = [ (pub_key >> int(8 * z)) & 0xFF ]
            msg = msg + byt		
        result = self.send(msg)
        return result
      
    #Simulating the Diffie-Hellman Key Exchange b/w two entities. 
    #
    def generate_dh_key(base=5, primeRandom=23):
        MyKey = DHKE(base, primeRandom)
        MyKey.generate_privatekey()

        print("------------Private Keys------------------\n")
        print("My Private Key Generated is ",MyKey.pk,"\n")

        MyKey.generate_publickey()

        print("------------Public Keys------------------\n")
        print("MyKey Public Key Generated is ",MyKey.pub_key,'\n')

        # send the pub_key
        # A = g^a mod p
        ans = self.send_dh_key_exchange(MyKey.pub_key)

        # parse the message for the key returned
        resp, key_dat = self.parse_key_exchange(ans)
        
        if (resp != 0xF0):
            print("invalid response to key exchange")
            sys.exit(-3)
            
        # calculate the share key from the answer    
        MyKey.exchange_key(key_dat)

        print("------------Shared Key Derieved------------------\n")
        print("Shared Key Generated now by MyKey : ",MyKey.share_key,'\n')
        return MyKey
        
    def primesInRange(x, y):
        prime_list = []
        for n in range(x, y):
            isPrime = True

            for num in range(2, n):
                if n % num == 0:
                    isPrime = False

            if isPrime:
                prime_list.append(n)

        return prime_list

    # get a random prime number in the range
    def get_random_prime(st=0, en=255):
        p = self.primesInRange(st, en)                          # get all prime numbers between start and end
        rp = p[np.random.choice((len(p)-1),1)]                  # select one of them randomly
        return [rp]

    # send the generator	
    def send_generator(self):
        msg = [ self.getseq(), '0x9', '0x4A' ]
        g = 0
        for z in range(0,8):
            rp_num = self.get_random_prime()
            msg = msg + rp_num
            g = g | (rp_num[0] << (8*z))            
        result = self.send(msg)
        return result, g

    # send the modulus
    def send_modulus(self):
        msg = [ self.getseq(), '0x9', '0x4B' ]
        p = 0
        for z in range(0,8):
            rp_num = self.get_random_prime()
            msg = msg + rp_num
            p = p | (rp_num[0] << (8*z))            
        result = self.send(msg)
        return result, p

    # send the key exchange - this just sends random primes but with diffie hellman you use send_dh_key_exchange instead
    def send_key_exchange(self):
        msg = [ self.getseq(), '0x9', '0x4C' ]
        for z in range(0,8):
            msg = msg + self.get_random_keys()		
        result = self.send(msg)
        return result

    def parse_key_exchange(self, dat):
        resp = dat[3] == 0xF0
        num = 8 
        for zz in range(0,num):
            z += int(dat[4+zz],16) << (8*zz)
        return (resp, z)    

    # need to understand how to duplicate the generation of the 2 encryption keys and do encryption using these on the message
    #// set full encryption key in command structure
    #commandStructure->Key.FixedKey = 0x0123456701234567;
    #commandStructure->Key.EncryptKey = keys->KeyHost;
    
    # note ammounts are in euro for mode values are == 0 test 1 real payout
    def payout_by_denomination(self, note_amt_list, mode=1):
        """which allows the user to specify exactly which notes are paid out."""
        i = 0
        end_currency = [0x45, 0x55, 0x52]                                   # currency The country code when converted to ASCII characters is EUR
        # end_currency = [hex(ord("C")), hex(ord("H")), hex(ord("F"))]      # uncomment to swap to swiss franks
        tot_num = hex(i)
        if (mode == 1):
            byte_pay = [0x58]
        else:
            byte_pay = [0x19]
        send_arr = [self.getseq(), '0x12', '0x46', tot_num]
        for noteamt, note_num in note_amt_list:
            tupv = dec2snd(noteamt*100)
            denom = [tupv[0], tupv[1], 0x00, 0x00]
            num = [hex(note_num)]
            send_arr = send_arr + num + demon + end_currency	
            i = i + 1	
        send_arr = send_arr + byte_pay
        tot_num = hex(i)                                       # number of different notes
        send_arr[3]=tot_num		
        lb=hex(len(send_arr)-2)	
        send_arr[1]=lb		                                   # message length
        result = self.send(send_arr)
        return result

    # note ammounts are in euro for mode values are == 0 test 1 real payout
    def float_by_denomination(self, note_amt_list, mode=1):
        """instructs the validator to float individual quantities of a denomination in the SMART payout. It follows a similar format to the Payout by 
           Denomination command"""
        i = 0
        end_currency = [0x45, 0x55, 0x52]                                   # currency The country code when converted to ASCII characters is EUR
        # end_currency = [hex(ord("C")), hex(ord("H")), hex(ord("F"))]      # uncomment to swap to swiss franks
        tot_num = hex(i)
        if (mode == 1):
            byte_pay = [0x58]
        else:
            byte_pay = [0x19]
        send_arr = [self.getseq(), '0x12', '0x44', tot_num]
        for noteamt, note_num in note_amt_list:
            denom = dec2snd4(noteamt*100)
            num = [hex(note_num)]
            send_arr = send_arr + num + demon + end_currency	
            i = i + 1	
        send_arr = send_arr + byte_pay
        tot_num = hex(i)                                       # number of different notes
        send_arr[3]=tot_num		
        lb=hex(len(send_arr)-2)	
        send_arr[1]=lb		                                   # message length
        result = self.send(send_arr)
        return result
        
    def payout_by_value(self, noteamt, mode=1):
        """command that instructs the payout device to payout a specified amount"""
        end_currency = [0x45, 0x55, 0x52]                                   # currency The country code when converted to ASCII characters is EUR
        # end_currency = [hex(ord("C")), hex(ord("H")), hex(ord("F"))]      # uncomment to swap to swiss franks
        if (mode == 1):
            byte_pay = [0x58]
        else:
            byte_pay = [0x19]
        send_arr = [self.getseq(), '0x12', '0x33']
        arr4_value = dec2snd4(noteamt*100)
        send_arr = send_arr + arr4_value + end_currency	+ byte_pay	
        lb=hex(len(send_arr)-2)	
        send_arr[1]=lb		                                   # message length
        result = self.send(send_arr)
        return result

    def get_note_ammount(self, noteamt, mode=1):
        """causes the validator to report the amount of notes stored of a specified denomination in the payout unit"""
        end_currency = [0x45, 0x55, 0x52]                                   # currency The country code when converted to ASCII characters is EUR
        # end_currency = [hex(ord("C")), hex(ord("H")), hex(ord("F"))]      # uncomment to swap to swiss franks
        if (mode == 1):
            byte_pay = [0x58]
        else:
            byte_pay = [0x19]
        send_arr = [self.getseq(), '0x12', '0x35']
        arr4_value = dec2snd4(noteamt*100)
        send_arr = send_arr + arr4_value + end_currency	+ byte_pay	
        lb=hex(len(send_arr)-2)	
        send_arr[1]=lb		                                   # message length
        result = self.send(send_arr)
        return result

    def set_coin_ammount(self, no_of_coins, value_of_coin):
        """causes the validator to report the amount of notes stored of a specified denomination in the payout unit"""
        end_currency = [0x45, 0x55, 0x52]                                   # currency The country code when converted to ASCII characters is EUR
        # end_currency = [hex(ord("C")), hex(ord("H")), hex(ord("F"))]      # uncomment to swap to swiss franks
        send_arr = [self.getseq(), '0x09', '0x34']
        nofc = dec2snd(no_of_coins)
        nofca = [nofc[0], nofc[1]]
        valc = dec2snd4(value_of_coin)
        send_arr = send_arr + nofca + valc + end_currency	
        result = self.send(send_arr)
        return result
		
	# The command sent is 0x0B, this is a two byte command, the second byte contains the 
    # programming the validator can expect to receive. In this case the second byte should be 
    # 0x03, this is the RAM programming command. The RAM file is transferred to the validator 
    # before the firmware/dataset file is transferred. The validator updates itself based on this 
    # RAM code rather than on code stored previously in the validator
    def send_prog_cmd(self):
        """ SENDING THE PROGRAMMING COMMAND AND RETRIEVING THE BLOCK SIZE """
        result = self.send([self.getseq(), '0x2', '0x0B', '0x03'])
        return result	

    def serial_number(self):
        """Return formatted serialnumber."""
        result = self.send([self.getseq(), '0x1', '0xC'], False)
        serial = 0
        for i in range(4, 8):
            serial += int(result[i], 16) << (8 * (7 - i))
        return serial

    def unit_data(self):
        # Response consits of
        # Unit-Type (0 = BNV)
        # Firmware-Version
        # Country-Code
        # Value-Multiplier
        # Protocol-Version
        result = self.send([self.getseq(), '0x1', '0xD'], False)

        unittype = int(result[4], 16)

        fwversion = ''
        for i in range(5, 9):
            fwversion += chr(int(result[i], 16))

        country = ''
        for i in range(9, 12):
            country += chr(int(result[i], 16))

        valuemulti = int('00', 16)
        for i in range(12, 15):
            valuemulti += int(result[i], 16)

        protocol = int(result[15], 16)

        unit_data = [unittype, fwversion, country, valuemulti, protocol]

        return unit_data

    def channel_values(self):
        """
        Return the real values of the channels.

        - Number of Channels
        - Values of Channels
        """
        result = self.send([self.getseq(), '0x1', '0xE'], False)

        channels = int(result[4], 16)

        unitdata = self.unit_data()

        values = []
        for i in range(0, channels):
            values.append(int(result[5 + i], 16) * unitdata[3])

        channel_data = [channels, values]
        return channel_data

    def channel_security(self):
        """
        Return the security settings of all channels.

        Number of Channels
        Security Data array()
        1 = Low Security
        2 = Std Security
        3 = High Security
        4 = Inhibited
        """
        result = self.send([self.getseq(), '0x1', '0xF'], False)

        channels = int(result[4], 16)

        security = []
        for i in range(0, channels):
            security.append(int(result[i + channels + 1], 16))

        security_data = [channels, security]
        return security_data

    def channel_reteach(self):
        """
        Return the (somewhat un-useful?) Re-Teach Data by Channel.

        Number of Channels
        Value of Reteach-Date array()
        """
        result = self.send([self.getseq(), '0x1', '0x10'], False)

        channels = int(result[4], 16)

        reteach = []
        for i in range(0, channels):
            reteach.append(int(result[i + channels + 1], 16))

        reteach_result = [channels, reteach]
        return reteach_result

    def sync(self):
        """
        Reset Sequence to be 0x00.

        Set ssp_sequence to 0x00, so next will be 0x80 by default
        """
        self.__sequence = '0x00'

        result = self.send([self.getseq(), '0x1', '0x11'])
        return result

    def sync_start(self, timeout=100):
       ss = time.time()
       ready = 0
       while ready == 0:
           result = self.send([self.getseq(), '0x1', '0x11']) 
           if len(result) >= 4:
               if result[3] == 0xF0:
                   ready = 1
           s = time.time()
           if (s - ss) > timeout: 
               ready = 2
       return ready
               
    # SSP_CMD_DISPENSE 0x12 not implented

    # SSP_CMD_PROGRAM_STATUS 0x16 not implented

    def last_reject(self):
        """Get reason for latest rejected banknote.

        0x00 = Note Accepted
        0x01 = Note length incorrect
        0x02 = Reject reason 2
        0x03 = Reject reason 3
        0x04 = Reject reason 4
        0x05 = Reject reason 5
        0x06 = Channel Inhibited
        0x07 = Second Note Inserted
        0x08 = Reject reason 8
        0x09 = Note recognised in more than one channel
        0x0A = Reject reason 10
        0x0B = Note too long
        0x0C = Reject reason 12
        0x0D = Mechanism Slow / Stalled
        0x0E = Striming Attempt
        0x0F = Fraud Channel Reject
        0x10 = No Notes Inserted
        0x11 = Peak Detect Fail
        0x12 = Twisted note detected
        0x13 = Escrow time-out
        0x14 = Bar code scan fail
        0x15 = Rear sensor 2 Fail
        0x16 = Slot Fail 1
        0x17 = Slot Fail 2
        0x18 = Lens Over Sample
        0x19 = Width Detect Fail
        0x1A = Short Note Detected
        """
        result = self.send([self.getseq(), '0x1', '0x17'], False)
        return result[4]

    def hold(self):
        result = self.send([self.getseq(), '0x1', '0x18'])
        return result

    # SPP_CMD_MANUFACTURER 0x30 not implented, collides with SSP_CMD_EXPANSION?

    # SSP_CMD_EXPANSION 0x30 not implented, collides with SSP_CMD_MANUFACTURER?

    def enable_higher_protocol(self):
        """Enable functions from implemented with version >= 3."""
        result = self.send([self.getseq(), '0x1', '0x19'])
        return result

# End Of Definition Of SSP_CMD_* Commands

    def getseq(self):
        # toggle SEQ between 0x80 and 0x00
        if (self.__sequence == '0x80'):
            self.__sequence = '0x00'
        else:
            self.__sequence = '0x80'

        returnseq = hex(self.__eSSPId | int(self.__sequence, 16))
        return returnseq

    def crc(self, command):
        length = len(command)
        seed = int('0xFFFF', 16)
        poly = int('0x8005', 16)
        crc = seed
        # self._logger.debug( " 1 || " + hex(crc) )

        for i in range(0, length):
            # self._logger.debug( " 2 || " + str(i) )
            crc ^= (int(command[i], 16) << 8)
            # self._logger.debug( " 3 || " + command[i] )
            # self._logger.debug( " 4 || " + hex(crc) )

            for j in range(0, 8):
                # self._logger.debug( " 5 || " + str(j) )

                if (crc & int('0x8000', 16)):
                    # self._logger.debug( " 6 || " + hex(crc) )
                    crc = ((crc << 1) & int('0xffff', 16)) ^ poly
                    # self._logger.debug( " 7 || " + hex(crc) )
                else:
                    crc <<= 1
                    # self._logger.debug( " 8 || " + hex(crc) )

        crc = [hex((crc & 0xFF)), hex(((crc >> 8) & 0xFF))]
        return crc

    def send(self, command, process=True):
        crc = self.crc(command)

        prepedstring = '7F'

        command = command + crc

        for i in range(0, len(command)):
            if (len(command[i]) % 2 == 1):
                prepedstring += '0'

            prepedstring += command[i][2:]

        self._logger.debug("OUT: 0x" + ' 0x'.join([prepedstring[x:x + 2]
                           for x in range(0, len(prepedstring), 2)]))

        prepedstring = prepedstring.decode('hex')

        self.__ser.write(prepedstring)

        response = self.read(process)
        return response

    def read(self, process=True):
        """Read the requested data from the serial port."""
        bytes_read = []
        # initial response length is only the header.
        expected_bytes = 3
        timeout_expired = datetime.datetime.now() + datetime.timedelta(seconds=self.timeout)
        while True:
            byte = self.__ser.read()
            if byte:
                bytes_read += byte
            else:
                # when the socket doesn't give us any data, evaluate the timeout
                if datetime.datetime.now() > timeout_expired:
                    raise eSSPTimeoutError('Unable to read the expected response of {} bytes within {} seconds'.format(
                                           expected_bytes, self.timeout))

            if expected_bytes == 3 and len(bytes_read) >= 3:
                # extract the actual message length
                expected_bytes += ord(bytes_read[2]) + 2

            if expected_bytes > 3 and len(bytes_read) == expected_bytes:
                # we've read the complete response
                break

        response = self.arrayify_response(bytes_read)
        self._logger.debug("IN:  " + ' '.join(response))

        if process:
            response = self.process_response(response)
        return response

    def arrayify_response(self, response):
        array = []
        for i in range(0, len(response)):
            array += [hex(ord(response[i]))]
        return array

    def process_response(self, response):
        # Answers seem to be always in lowercase

        # Error-Codes
        # 0xf0   OK
        # 0xf2   Command not known
        # 0xf3   Wrong number of parameters
        # 0xf4   Parameter out of range
        # 0xf5   Command cannot be processed
        # 0xf6   Software Error
        # 0xf8   FAIL
        # 0xFA   Key not set

        # Default: Something failed
        processed_response = '0xf8'

        if response[0] == '0x7f':
            crc_command = []
            for i in range(1, int(response[2], 16) + 3):
                crc_command.append(response[i])

            crc = self.crc(crc_command)

            if (response[len(response) - 2] != crc[0]) & \
                    (response[len(response) - 1] != crc[1]):
                self._logger.debug("Failed to verify crc.")
            else:
                processed_response = response[3]
                if response[3] != '0xf0':
                    self._logger.debug("Error " + response[3])
        return processed_response

    def easy_inhibit(self, acceptmask):
        channelmask = []
        bitmask = int('00000000', 2)

        channelmask.append(int('00000001', 2))
        channelmask.append(int('00000010', 2))
        channelmask.append(int('00000100', 2))
        channelmask.append(int('00001000', 2))
        channelmask.append(int('00010000', 2))
        channelmask.append(int('00100000', 2))
        channelmask.append(int('01000000', 2))
        channelmask.append(int('10000000', 2))

        for i in range(0, len(acceptmask)):
            if acceptmask[i] == 1:
                bitmask = bitmask + channelmask[i]

        bitmask = hex(bitmask)
        return bitmask
		
