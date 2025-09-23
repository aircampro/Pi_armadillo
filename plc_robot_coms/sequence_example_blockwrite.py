#!/usr/bin/env python
#-*- coding:utf-8 -*-
#
# simple i/o sequencer to make block write values to go to PLC or io block.
# 
import math

# pin list e.g. [ 2, 6, 4, 8, 7] to output block
#
def pin_list_on_to_output( pin_list=[ 2, 6, 4, 8, 7], offset=1, block_out=0 ):
    pin_list.sort()
    for pin_no in pin_list:
        block_out |= int(math.pow(2,(pin_no-offset)))
    return block_out

def pin_list_off_to_output( pin_list=[ 2, 6, 4, 8, 7], offset=1, block_out=0, bits=0xF ):
    pin_list.sort()
    for pin_no in pin_list:
        block_out &= (not(int(math.pow(2,(pin_no-offset)))) & bits)
    return block_out

def pin_list_toggle_to_output( pin_list=[ 2, 6, 4, 8, 7], offset=1, block_out=0 ):
    pin_list.sort()
    for pin_no in pin_list:
        block_out ^= int(math.pow(2,(pin_no-offset)))
    return block_out

# returns a block write number to output pin numbers
def block_to_outpins(block_out=0, bits=0xF, offset=1):
    for i in range(0, int(bits)):
	    if (block_out & i) == i:
		    print(f"output {i+offset} on")

if __name__ == '__main__':

    # step 1
	drive = [ 1, 8 ]
    o_block_1 = pin_list_on_to_output( drive )
	drive = [ 9, 12 ]	
    o_block_2 = pin_list_on_to_output( drive )	

    time.sleep(100)
	
    # step 2
	drive = [ 2, 5 ]
    o_block_1 = pin_list_on_to_output( drive )
	drive = [ 15, 10 ]	
    o_block_2 = pin_list_on_to_output( drive )	