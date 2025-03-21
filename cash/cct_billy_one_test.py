
#!/usr/bin/python
# -*- mode: python -*-
"""
Script to demonstrate coin_messenger functions.
"""
# The python-cctalk package allows one to send ccTalk messages and decode replies from a coin validator. 
license_text = "(C) 2011 David Schryer GNU GPLv3 or later."
__copyright__ = license_text

from cct_coin_messenger import CoinMessenger
from cctalk.tools import make_serial_object

def get_coin_type(coin_messenger):       

    # Maps the coin value to the number the coin validator outputs.
    coin_messenger.accept_coins(True)
    data_buffer = coin_messenger.read_buffer()
    
    coin_dic = {4:0.1, 5:0.2, 6:0.50, 7:1, 8:2}
    try:
        coin_type = coin_dic[data_buffer[1]]
    except TypeError:
        return None
    return coin_type
        
if __name__ == '__main__':

    coin_validator_connection = make_serial_object('/dev/ttyUSB0')
    coin_messenger = CoinMessenger(coin_validator_connection)
    
    coin_type = get_coin_type(coin_messenger)

    coin_messenger.request('reset_device')
    coin_messenger.request('data_storage_availability')

    # tests for the billy one machine
    #
    coin_messenger.request('curr_rev')
    coin_messenger.request('bill_op_mode')
    coin_messenger.request('set_bill_mode')
    coin_messenger.request('route_bill')
    coin_messenger.request('country_scale')
    coin_messenger.request('bill_id')
    coin_messenger.request('buff_bill')
    coin_messenger.request('alarm_counter')
    coin_messenger.request('fraud_counter')
    coin_messenger.request('reject_counter')
    coin_messenger.request('bank_select')
    coin_messenger.request('rom_checksum')
  
    coin_messenger.read_buffer()
    coin_messenger.set_accept_limit(25)

    coin_messenger.change_model(2)               # change the dictionary
    coin_messenger.request('enter_pin')
    coin_messenger.request('enter_new_pin')
    coin_messenger.request('enter_pin')

