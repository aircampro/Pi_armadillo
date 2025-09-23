#!/usr/bin/env python
#-*- coding:utf-8 -*-
#
# Let's say you have a binary log, where each record is 4 bytes of time (uint32), 
# 2 bytes of event code (uint16), 1 byte of flag (bool), and 8 bytes of data (double). 
#
import struct
BINARY_FL_NM='log.bin'
record_fmt = '<IHBd'                                        # binary records as described uint32, uint16, uint8, double
record_size = struct.calcsize(record_fmt)
with open(BINARY_FL_NM, 'rb') as f:
  while chunk := f.read(record_size):
    timestamp, code, flag, value = struct.unpack(record_fmt, chunk)
    print(timestamp, code, flag, value)