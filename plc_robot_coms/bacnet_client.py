#!/usr/bin/env python 
# -*- coding: utf-8 -*- 
# .... bacnet client (publisher) example .....   
# we create 10 random values to publisher (untested)
#  
# install as :-
# pip install bacpypes
# pip install bacpypes[all]
# pip install bacpypes[console]
#
from bacpypes.core import run  
from bacpypes.primitivedata import Real 
from bacpypes.object import AnalogValueObject, Property, register_object_type  
from bacpypes.app import BIPSimpleApplication 
from bacpypes.service.device 
import LocalDeviceObject  
import random                                                                                     # # RandomValueProperty # 

class RandomValueProperty(Property):                                                              #     # constructor     #     
    def __init__(self, identifier):         
    Property.__init__(self, identifier, Real, default=0.0, optional=True, mutable=False)          #    read    #     

    def ReadProperty(self, obj, arrayIndex=None):         #         
        # access an array         #         
        print arrayIndex         
        if arrayIndex is not None:             
            raise ExecutionError(errorClass='property', errorCode='propertyIsNotAnArray')          #    return a random value         #         
        value = random.random() * 100.0         
        return value                                                                               #    return value    #     

    def WriteProperty(self, obj, value, arrayIndex=None, priority=None, direct=False):         
        raise ExecutionError(errorClass='property', errorCode='writeAccessDenied')  
    
# Random Value Object Type # 
class RandomAnalogValueObject(AnalogValueObject):     
    properties = [         RandomValueProperty('presentValue'),     ]      
    def __init__(self, **kwargs):         
        AnalogValueObject.__init__(self, **kwargs)                                                 
        register_object_type(RandomAnalogValueObject)                                              # # Entry Point # 
        
if __name__ == '__main__':                                                                         #    main     #     
    device_name  = 'my_bacnet_device'     
    device_id    = 1234     
    vendor_id    = 1234     
    maxApduLengthAccepted = 1024     
    segmentationSupported = 'segmentedBoth'                                                         #     # Defining the network     #    
    address = '10.2.10.17/24'                                                                       #     # ip     #     
    device = LocalDeviceObject( objectName = device_name, objectIdentifier = ('device', device_id), maxApduLengthAccepted=maxApduLengthAccepted,  segmentationSupported=segmentationSupported, vendorIdentifier=vendor_id, vendorName="ACP",)
    app = BIPSimpleApplication(device, address)                                                     #     Configure services supported by the application on the device     #    
    services_supported = app.get_services_supported()     
    device.protocolServicesSupported = services_supported.value                                     #      add object to device     #    
    my_values = { 1, 2, 3, 4, 5 }    
    for i in my_values:                                                                             #      Object definition        #         
        ravo = RandomAnalogValueObject( objectName ='Random-%d' % (i,), objectIdentifier =('analogValue', i),  )  # オブジェクト の 登録         #         
        app.add_object(ravo)                                                                                      #     # サーバの起動     #     
    run()
