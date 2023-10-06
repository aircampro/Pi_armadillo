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

class RandomValueProperty(Property):                                                              #     # コンストラクタ     #     
    def __init__(self, identifier):         
    Property.__init__(self, identifier, Real, default=0.0, optional=True, mutable=False)          #     # 読み込み     #     

    def ReadProperty(self, obj, arrayIndex=None):         #         
        # access an array         #         
        print arrayIndex         
        if arrayIndex is not None:             
            raise ExecutionError(errorClass='property', errorCode='propertyIsNotAnArray')          #         # return a random value         #         
        value = random.random() * 100.0         
        return value                                                                               #     # 書き込み     #     

    def WriteProperty(self, obj, value, arrayIndex=None, priority=None, direct=False):         
        raise ExecutionError(errorClass='property', errorCode='writeAccessDenied')  
    
# Random Value Object Type # 
class RandomAnalogValueObject(AnalogValueObject):     
    properties = [         RandomValueProperty('presentValue'),     ]      
    def __init__(self, **kwargs):         
        AnalogValueObject.__init__(self, **kwargs)                                                 # # オブジェクトを登録 # 
        register_object_type(RandomAnalogValueObject)                                              # # Entry Point # 
        
if __name__ == '__main__':                                                                         #     # デバイスの設定情報の定義     #     
    device_name  = 'my_bacnet_device'     
    device_id    = 1234     
    vendor_id    = 1234     
    maxApduLengthAccepted = 1024     
    segmentationSupported = 'segmentedBoth'                                                         #     # ネットワークの定義     #    
    address = '10.2.10.17/24'                                                                       #     # デバイスの定義     #     
    device = LocalDeviceObject( objectName = device_name, objectIdentifier = ('device', device_id), maxApduLengthAccepted=maxApduLengthAccepted,  segmentationSupported=segmentationSupported, vendorIdentifier=vendor_id, vendorName="ACP",)
    app = BIPSimpleApplication(device, address)                                                     #     # アプリケーションがサポートするサービスをデバイスに設定     #    
    services_supported = app.get_services_supported()     
    device.protocolServicesSupported = services_supported.value                                     #     # デバイスにオブジェクトを追加     #    
    my_values = { 1, 2, 3, 4, 5 }    
    for i in my_values:                                                                             #         # オブジェクト の 定義         #         
        ravo = RandomAnalogValueObject( objectName ='Random-%d' % (i,), objectIdentifier =('analogValue', i),  )  # オブジェクト の 登録         #         
        app.add_object(ravo)                                                                                      #     # サーバの起動     #     
    run()