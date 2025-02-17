################################################################################################
# Project:  Modbus Library
# Module:   ModbusDataConvs.py
# Purpose:  Extended Modbus data types.
# Language: Python 2/3
# Date:     07-Jun-2009.
# Ver:      17-Feb-2025 ACP modified just to do data type conversions between list and value.
# Author:   M. Griffin.
# Copyright: 2009 - Michael Griffin <m.os.griffin@gmail.com>
# This library is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
################################################################################################
from __future__ import division
import struct

################################################################################################
"""
Read and write extended data types to a Modbus compatible data table. Extended
data types are data types that are stored in more than one register. These include:

- 32 bit signed integers.
- Single precision 4 byte floating point.
- Double precision 8 byte floating point.
- String with 2 characters per register.
- String with 1 character per register.

The parameters and return values for each function are described in terms
implying a certain data size (e.g. 32 bit integer). However, all numeric
values are native Python types. The size refers to the range of values
permitted when they are stored in the data table.

"""

################################################################################################
class DataTransferHelpers:

    ########################################################
    def __init__(self):

    ########################################################
    def GetHRegInt32(self, reglist=[23,44]):

        return struct.unpack('@i', struct.pack('@2h', *reglist))[0]

    ########################################################
    def GetInpRegInt32(self, reglist=[23,44]):

        return struct.unpack('@i', struct.pack('@2h', *reglist))[0]

    ########################################################
    def SetHRegInt32(self, datavalue):

        # Store the result in a pair of registers.
        return list(struct.unpack('@2h', struct.pack('@i', datavalue)))

    ########################################################
    def SetInpRegInt32(self, datavalue):

        # Store the result in a pair of registers.
        return list(struct.unpack('@2h', struct.pack('@i', datavalue)))

    ########################################################
    def GetHRegFloat32(self, reglist=[23,44]):

        return struct.unpack('@f', struct.pack('@2h', *reglist))[0]

    ########################################################
    def GetInpRegFloat32(self, reglist=[23,44]):

        return struct.unpack('@f', struct.pack('@2h', *reglist))[0]

    ########################################################
    def SetHRegFloat32(self, datavalue):

        # This needs range checking.
        try:
            reglist = list(struct.unpack('@2h', struct.pack('@f', datavalue)))
        except:
            reglist = [0, 0]
        return reglist

    ########################################################
    def SetInpRegFloat32(self, reglist=[23,44]):

        # This needs range checking.
        try:
            reglist = list(struct.unpack('@2h', struct.pack('@f', datavalue)))
        except:
            reglist = [0, 0]
        return reglist

    ########################################################
    def GetHRegFloat64(self, reglist=[23,44,32,4]):

        return struct.unpack('@d', struct.pack('@4h', *reglist))[0]

    ########################################################
    def GetInpRegFloat64(self, reglist=[23,44,32,4]):

        return struct.unpack('@d', struct.pack('@4h', *reglist))[0]

    ########################################################
    def SetHRegFloat64(self, datavalue):

        # This needs range checking.
        try:
            reglist = list(struct.unpack('@4h', struct.pack('@d', datavalue)))
        except:
            reglist = [0, 0, 0, 0]
        return reglist

    ########################################################
    def SetInpRegFloat64(self, datavalue):

        # This needs range checking.
        try:
            reglist = list(struct.unpack('@4h', struct.pack('@d', datavalue)))
        except:
            reglist = [0, 0, 0, 0]
        return reglist

    ########################################################
    def GetHRegStr8(self, reglist):

        return struct.pack('>%sH' % len(reglist), *reglist)

    ########################################################
    def GetInpRegStr8(self, reglist):

        return struct.pack('>%sH' % len(reglist), *reglist)

    ########################################################
    def SetHRegStr8(self, datavalue):

        # Compensate for 2 characters per register.
        reglen = strlen * 2
        # Pad or truncate the string to the full specified length.
        if (len(datavalue) < reglen):
            datavalue = '%s%s' % (datavalue, '\x00' * (reglen - len(datavalue)))
        elif (len(datavalue) > reglen):
            datavalue = datavalue[:reglen]
        # If the string is of an odd length, pad it out to an even length.
        if ((len(datavalue) % 2) != 0):
            datavalue = '%s\x00' % datavalue
        reglist = list(struct.unpack('>%sh' % (len(datavalue) // 2), datavalue))
        return reglist

    ########################################################
    def SetInpRegStr8(self, datavalue):

        # Compensate for 2 characters per register.
        reglen = strlen * 2
        # Pad or truncate the string to the full specified length.
        if (len(datavalue) < reglen):
            datavalue = '%s%s' % (datavalue, '\x00' * (reglen - len(datavalue)))
        elif (len(datavalue) > reglen):
            datavalue = datavalue[:reglen]
        # If the string is of an odd length, pad it out to an even length.
        if ((len(datavalue) % 2) != 0):
            datavalue = '%s\x00' % datavalue
        reglist = list(struct.unpack('>%sh' % (len(datavalue) // 2), datavalue))
        return reglist

    ########################################################
    def GetHRegStr16(self, reglist):

        # Mask off the upper byte in the register.
        reglim = [x & 0xff for x in reglist]
        return struct.pack('>%sB' % len(reglist), *reglim)

    ########################################################
    def GetInpRegStr16(self, reglist):

        return struct.pack('>%sB' % len(reglist), *reglist)

    ########################################################
    def SetHRegStr16(self, datavalue):

        # Pad or truncate the string to the full specified length.
        if (len(datavalue) < strlen):
            datavalue = '%s%s' % (datavalue, '\x00' * (strlen - len(datavalue)))
        elif (len(datavalue) > strlen):
            datavalue = datavalue[:strlen]
        reglist = list(struct.unpack('>%sb' % len(datavalue), datavalue))
        return reglist

    ########################################################
    def SetInpRegStr16(self, datavalue):

        # Pad or truncate the string to the full specified length.
        if (len(datavalue) < strlen):
            datavalue = '%s%s' % (datavalue, '\x00' * (strlen - len(datavalue)))
        elif (len(datavalue) > strlen):
            datavalue = datavalue[:strlen]
        reglist = list(struct.unpack('>%sb' % len(datavalue), datavalue))
        return reglist