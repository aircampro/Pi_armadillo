#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# UDS Client Library for can with isotp using udsoncan
#
import json
import queue
import sys
import ctypes
import time
import os

# although no GUI we use Qt threading
from PyQt5.QtCore import QThread, QCoreApplication, Qt, pyqtSignal, QRegExp, QStringListModel

# can libraries
import can
from can.interfaces.vector import VectorBus, xldefine, get_channel_configs, VectorBusParams, VectorCanParams, \
    VectorCanFdParams
from udsoncan import NegativeResponseException, TimeoutException, UnexpectedResponseException, InvalidResponseException, \
    services, Request, DidCodec, ConfigError, MemoryLocation, DataFormatIdentifier
from udsoncan.services import RequestDownload, TransferData
import isotp
from udsoncan.connections import PythonIsoTpConnection
from udsoncan.client import Client
import udsoncan.configs
import configparser

class dataUtils:

    def chunk_the_data(self, data, NumberOfBlockLength):
        """chunk data

        Args:
          data: the data you want to chunk
          block_size: the block size of each chunk

        Returns:
          data_chunks: chunk data(s) in a list
        """
        data_chunks = []
        start = 0
        while start < len(data):
            end = min(start + NumberOfBlockLength, len(data))  
            data_chunks.append(data[start:end])
            start += NumberOfBlockLength
        return data_chunks

class canTesterPresentThread(QThread):

    def __init__(self, conn):
        super().__init__()
        self.conn = conn
        self.daemon = True  
        self.flag_3e = False
        self.flag_stop = False

    def run(self):
        req = Request(services.TesterPresent, subfunction=0, suppress_positive_response=True)
        while True:
            time.sleep(3)
            if self.flag_stop:
                break
            if self.flag_3e:

                try:
                    self.conn.send_request(req)
                except Exception as e:
                    print(e)
            else:
                pass

    def set_3e_flag(self, flag):
        self.flag_3e = flag

    def stop_thread(self):
        self.flag_stop = True

class canUDSClientThread(QThread):

    send_data = pyqtSignal(object)
    rec_data = pyqtSignal(object)
    sig_send_state = pyqtSignal(object)
    sig_flag_3e = pyqtSignal(object)
    sig_ecu_name = pyqtSignal(object)

    def __init__(self, conn, send_queue, dll_path, ecus):
        super().__init__()
        self.daemon = True 
        self.conn = conn
        self.ecus = ecus
        self.send_queue = send_queue
        self.stop_flag = 0;
        self.send_state = 'Normal'
        self.dll_path = dll_path
        self.dll_lib = None
        self.generated_key = None
        self.is_ascii = False
        self.ecu_name = None

    def run(self):
        self.load_dll(self.dll_path)

        # client=Client(conn=self.conn, config=self.config,request_timeout=2)
        while True:
            try:
                if self.stop_flag == 1:
                    break

                req = self.send_queue.get(block=True, timeout=1)  
                self.send_data.emit(req.get_payload())
                try:
                    response = self.conn.send_request(req)
                    data_raw = response.original_payload
                    if data_raw[0] == 0x62:                                                                      # ReadDataByIdentifier
                        did = hex(data_raw[1] * 0x100 + data_raw[2])
                        try:
                            isascii = self.ecus[self.ecu_name]['ReadDataByIdentifier'][did]
                            if isascii == 'ascii':
                                self.is_ascii = True
                            else:
                                self.is_ascii = False

                        except:
                            self.is_ascii = False

                    if self.is_ascii:
                        self.rec_data.emit(f"Positive: {data_raw[3:].decode('ascii')}\r")
                        self.is_ascii = False
                    else:
                        self.rec_data.emit(f"Positive: {data_raw.hex(' ')}\r")

                    if data_raw[0] == 0x50 and data_raw[1] > 1:
                        self.sig_flag_3e.emit(True)
                    elif data_raw[0] == 0x50 and data_raw[1] == 1:
                        self.sig_flag_3e.emit(False)

                    if data_raw[0] == 0x67 and data_raw[1] == req.get_payload()[1]:
                        length = len(data_raw)
                        seed_bytes = data_raw[2:length]
                        seed_array = (ctypes.c_ubyte * len(seed_bytes))(*seed_bytes)
                        seed_length = ctypes.c_uint(len(seed_bytes))
                        security_level = ctypes.c_uint(data_raw[1])
                        variant_string = None
                        Options_string = None

                        max_key_size = length - 2
                        key_array = (ctypes.c_ubyte * max_key_size)()
                        actual_key_size = ctypes.c_uint()

                        try:
                            self.dll_lib.GenerateKeyEx.restype = ctypes.c_int
                            self.dll_lib.GenerateKeyEx.argtypes = [
                                ctypes.POINTER(ctypes.c_ubyte),                                                  # ipSeedArray
                                ctypes.c_uint,                                                                   # iSeedArraySize
                                ctypes.c_uint,                                                                   # iSecurityLevel
                                ctypes.POINTER(ctypes.c_char),                                                   # ipVariant
                                ctypes.POINTER(ctypes.c_ubyte),                                                  # iopKeyArray
                                ctypes.c_uint,                                                                   # iMaxKeyArraySize
                                ctypes.POINTER(ctypes.c_uint)                                                    # oActualKeyArraySize
                            ]

                            result = self.dll_lib.GenerateKeyEx(
                                seed_array,                                                                     # ipSeedArray
                                seed_length,                                                                    # iSeedArraySize
                                security_level,                                                                 # iSecurityLevel
                                ctypes.POINTER(ctypes.c_char)(),                                                # ipVariant (None or empty)
                                key_array,                                                                      # iopKeyArray
                                ctypes.c_uint(max_key_size),                                                    # iMaxKeyArraySize
                                ctypes.byref(actual_key_size)                                                   # oActualKeyArraySize
                            )

                            if result == 0:
                                self.generated_key = bytearray(key_array)[:actual_key_size.value]
                            else:
                                self.generated_key = None

                        except AttributeError as e:
                            self.generated_key = None

                        except Exception as e:
                            self.generated_key = None

                        if self.generated_key == None:
                            try:
                                self.dll_lib.GenerateKeyExOpt.restype = ctypes.c_int
                                self.dll_lib.GenerateKeyExOpt.argtypes = [
                                    ctypes.POINTER(ctypes.c_ubyte),                                            # ipSeedArray
                                    ctypes.c_uint,                                                             # iSeedArraySize
                                    ctypes.c_uint,                                                             # iSecurityLevel
                                    ctypes.POINTER(ctypes.c_char),                                             # ipVariant
                                    ctypes.POINTER(ctypes.c_char),                                             # ipOptions
                                    ctypes.POINTER(ctypes.c_ubyte),                                            # iopKeyArray
                                    ctypes.c_uint,                                                             # iMaxKeyArraySize
                                    ctypes.POINTER(ctypes.c_uint)                                              # oActualKeyArraySize
                                ]

                                result = self.dll_lib.GenerateKeyExOpt(
                                    seed_array,                                                                # ipSeedArray
                                    seed_length,                                                               # iSeedArraySize
                                    security_level,                                                            # iSecurityLevel
                                    ctypes.POINTER(ctypes.c_char)(),                                           # ipVariant (None or empty)
                                    ctypes.POINTER(ctypes.c_char)(),                                           # ipVariant (None or empty)
                                    key_array,                                                                 # iopKeyArray
                                    ctypes.c_uint(max_key_size),                                               # iMaxKeyArraySize
                                    ctypes.byref(actual_key_size)                                              # oActualKeyArraySize
                                )

                                if result == 0:
                                    self.generated_key = bytearray(key_array)[:actual_key_size.value]
                                else:
                                    self.generated_key = None

                            except AttributeError as e:
                                self.generated_key = None

                            except Exception as e:
                                self.generated_key = None

                        if self.generated_key == None:
                            self.generated_key = bytes([0x00] * (length - 2))

                        req1 = Request(services.SecurityAccess, subfunction=data_raw[1] + 1, data=bytes(self.generated_key))
                        self.send_data.emit(req1.get_payload())
                        response = self.conn.send_request(req1)
                        data_raw = response.original_payload
                        self.rec_data.emit(f"Positive: {data_raw.hex(' ')}\r")
                        print(f'level {data_raw[1]}')

                except NegativeResponseException as e:
                    print(e)
                    self.rec_data.emit(f"Negative response: {e.response.code_name}\r")
                except InvalidResponseException as e:
                    print(e)
                    data_raw = e.response.original_payload
                    self.rec_data.emit(f"Invalid Response: {data_raw.hex(' ')}\r")
                except UnexpectedResponseException as e:
                    print(e)
                    data_raw = e.response.original_payload
                    self.rec_data.emit(f"Unexpected Response: {data_raw.hex(' ')}\r")
                except ConfigError as e:
                    self.rec_data.emit(str(e) + '\r')
                except TimeoutException as e:
                    if self.send_state != 'Normal':
                        self.send_state = 'Normal'
                        pass
                    else:
                        self.rec_data.emit(str(e) + '\r')
                except Exception as e:
                    self.rec_data.emit(str(e) + '\r')
            except queue.Empty:
                pass

    def load_dll(self, dll_path):
        try:
            self.dll_lib = ctypes.WinDLL(dll_path)
        except:
            self.dll_lib = None

    def stop_thread(self):
        self.stop_flag = 1

    def set_send_state(self, error):
        self.send_state = error

    def set_ecu_name(self, ecu_name):
        self.ecu_name = ecu_name


class MyCodec(DidCodec):
    def encode(self, val):
        return bytes(val)  

    def decode(self, payload):
        return list(payload)  
		
class udsCanClient(QThread):
    sig_dll_path = pyqtSignal(object)
    
    def __init__(self, chann=1):
        super().__init__()
        self.vectorConfigs = []
        self.dll_path = None
        self.channel_choose = 0
        self.appName = 'UDS Client'
        self.vectorChannelCanParams = None
        self.is_run = False
        self.channel_choose = chann
        self.ecu_name = None
       
    def refresh_drive(self):
        self.__vectorAvailableConfigs = get_channel_configs()
        self.vectorConfigs.clear()
        for channel_list in self.__vectorAvailableConfigs:
            if (channel_list.channel_bus_capabilities & xldefine.XL_BusCapabilities.XL_BUS_ACTIVE_CAP_CAN):
                self.vectorConfigs.append({
                    'name': channel_list.name,
                    'transceiver_name': channel_list.transceiver_name,
                    'channel_index': channel_list.channel_index,
                    'hw_type': channel_list.hw_type,
                    'hw_index': channel_list.hw_index,
                    'hw_channel': channel_list.hw_channel,
                    'serial_number': channel_list.serial_number,
                    'is_on_bus': channel_list.is_on_bus,
                    'is_support_canfd': bool(channel_list.channel_capabilities.value & \
                                             xldefine.XL_ChannelCapabilities.XL_CHANNEL_FLAG_CANFD_BOSCH_SUPPORT.value),
                    'can_op_mode': channel_list.bus_params.can.can_op_mode
                })
                
    def read_ecu_config(self):
    
        self.refresh_drive()
        
        config_dir = './SecurityAccessDLL'

        if not os.path.exists(config_dir):
            os.makedirs(config_dir)

        config = configparser.ConfigParser()
        config_file = './ECUConfig/DIDList.ini'
        if not os.path.exists(config_file):
            os.makedirs(os.path.dirname(config_file), exist_ok=True)
            content = """
# Normal_11bits = 0
# Normal_29bits = 1
# NormalFixed_29bits = 2
# Extended_11bits = 3
# Extended_29bits = 4
# Mixed_11bits = 5
# Mixed_29bits = 6
[viu_f]
AddressingMode:0
uds_on_can_request_id : 0x701
uds_on_can_response_id : 0x601
uds_on_can_function_id : 0x7df

[viu_f:dll]
dll:./SecurityAccessDLL/SeednKey.dll

[viu_f:ReadDataByIdentifier]
0xF195:ascii
0xF194:raw

[viu_f:DIDs]
read sw version:22F195
write f189:2ef18900112233445566

[viu_ml]
AddressingMode:0
uds_on_can_request_id : 0x702
uds_on_can_response_id : 0x602
uds_on_can_function_id : 0x7df

[viu_ml:ReadDataByIdentifier]
0xF195:ascii
0xF194:raw

[viu_ml:dll]
dll:./SecurityAccessDLL/send.dll

[viu_ml:DIDs]
read sw version:22F195
write f189:2ef18900112233445577

    """

            with open(config_file, 'w') as file:
                file.write(content.strip())
        config.read(config_file)
        ecu_data = {}
        for ecu_name in config.sections():
            if ":" not in ecu_name: 
                AddressingMode = None
                try:
                    AddressingMode = int(config[ecu_name]["AddressingMode"])
                except:
                    AddressingMode = 0
                try:
                    uds_on_can_request_id = config[ecu_name]["uds_on_can_request_id"]
                except:
                    uds_on_can_request_id = 0x701
                try:
                    uds_on_can_response_id = config[ecu_name]["uds_on_can_response_id"]
                except:
                    uds_on_can_response_id = 0x702
                try:
                    uds_on_can_function_id = config[ecu_name]["uds_on_can_function_id"]
                except:
                    uds_on_can_function_id = 0x7df

                ecu_data[ecu_name] = {}
                #  ECU 
                ecu_data[ecu_name]["AddressingMode"] = AddressingMode
                ecu_data[ecu_name]["uds_on_can_request_id"] = uds_on_can_request_id
                ecu_data[ecu_name]["uds_on_can_response_id"] = uds_on_can_response_id
                ecu_data[ecu_name]["uds_on_can_function_id"] = uds_on_can_function_id

                #  dll 
                try:
                    dll = config[f"{ecu_name}:dll"]["dll"]
                except:
                    dll = None
                ecu_data[ecu_name]["dll"] = dll

                # ReadDataByIdentifier 
                ecu_data[ecu_name]["ReadDataByIdentifier"] = {}
                try:
                    for key, value in config.items(f"{ecu_name}:ReadDataByIdentifier"):
                        ecu_data[ecu_name]["ReadDataByIdentifier"][key] = value
                except:
                    pass
                # DIDs 
                ecu_data[ecu_name]["DIDs"] = {}
                try:
                    for key, value in config.items(f"{ecu_name}:DIDs"):
                        ecu_data[ecu_name]["DIDs"][key] = value
                except:
                    pass

        return ecu_data

    def set_canParams(self):
        can_params = VectorCanParams(
            bitrate=5000,
            sjw=1,
            tseg1=1,
            tseg2=1,
            sam=1,
            output_mode=xldefine.XL_OutputMode.XL_OUTPUT_MODE_NORMAL,
            can_op_mode=xldefine.XL_CANFD_BusParams_CanOpMode.XL_BUS_PARAMS_CANOPMODE_CAN20
        )
        canfd_params = VectorCanFdParams(
            bitrate=500000,
            data_bitrate=2000000,
            sjw_abr=16,
            tseg1_abr=63,
            tseg2_abr=16,
            sam_abr=1,
            sjw_dbr=4,
            tseg1_dbr=15,
            tseg2_dbr=4,
            output_mode=xldefine.XL_OutputMode.XL_OUTPUT_MODE_NORMAL,
            can_op_mode=xldefine.XL_CANFD_BusParams_CanOpMode.XL_BUS_PARAMS_CANOPMODE_CANFD
        )

        self.vectorChannelCanParams = VectorBusParams(
            bus_type=xldefine.XL_BusTypes.XL_BUS_TYPE_CAN,
            can=can_params,
            canfd=canfd_params
        )

    def set_ecu_diag_id(self):
        for e in self.ecus.keys(): 
            ecu_name = e
        if not ecu_name == None:    
            self.ecu_name = ecu_name
            self.dll_path = self.ecus[ecu_name]['dll']
            self.sig_dll_path.emit(self.ecus[ecu_name]['dll'])
        else:
            print("no ecu name has been read/set")

    def update_flash_cfg_list(self):
        try:
            with open("FlashConfig/FlashConfig.json", "r") as f:
                self.flash_cfg = json.load(f)
        except Exception as e:
            pass
            
    def init(self):
        try:
            self.ecus = self.read_ecu_config()
            self.set_ecu_diag_id()
            self.update_flash_cfg_list()
        except:
            self.ecus = None
        self.send_queue = queue.Queue()
        self.dll_lib=ctypes.WinDLL("./SeednKey.dll")
        self.set_canParams()

        # self.dll_lib=ctypes.WinDLL("./GenerateKeyExImpl.dll")

    # runs can bus thread - use to start it		
    def run_bus(self):
        if self.is_run:
            self.canudsthread.stop_thread()
            self.canTesterPresentThread.stop_thread()
            self.send_queue.empty()
            self.conn.close()
            self.stack.stop()
            self.notifier.stop()
            self.canbus.shutdown()
            self.is_run = False
        else:
            self.connect_vector_can_interfaces()
            self.create_uds_client(self.canbus)

    # connect vector can interfaces
    def connect_vector_can_interfaces(self, dl="short"):

        VectorBus.set_application_config(
            app_name=self.appName,
            app_channel=self.vectorConfigs[self.channel_choose]['channel_index'],
            hw_type=self.vectorConfigs[self.channel_choose]['hw_type'],
            hw_index=self.vectorConfigs[self.channel_choose]['hw_index'],
            hw_channel=self.vectorConfigs[self.channel_choose]['hw_channel'],
        )
        if self.checkBox_bustype.isChecked():
            busParams_dict = self.vectorChannelCanParams.canfd._asdict()
        else:
            busParams_dict = self.vectorChannelCanParams.can._asdict()

        if self.vectorConfigs[self.channel_choose]["is_on_bus"]:
            self.canbus = VectorBus(
                channel=self.vectorConfigs[self.channel_choose]['channel_index'],
                app_name=self.appName,
                fd=(dl =="long"))
        else:
            self.canbus = VectorBus(
                channel=self.vectorConfigs[self.channel_choose]['channel_index'],
                app_name=self.appName,
                fd=(dl =="long"),
                **busParams_dict)

    def get_diag_id_by_str(self, id_str):
        try:
            id = int(id_str, 10)
            return id
        except ValueError:
            try:
                id = int(id_str, 16)
                return id
            except ValueError:
                print(' cannot translate this value ',id_str)

    def load_dll(self, dll_path):
        try:
            self.dll_lib = ctypes.WinDLL(dll_path)
        except:
            self.dll_lib = None

    def print_tx(self, data):
        text = 'Tx:' + data.hex(" ").upper() + '\r'
        print(text)

    def print_rx(self, data):
        text = 'Rx:' + data.hex(" ").upper() + '\r'
        print(text)

    def send_ecu_name(self):
        if self.is_run:
            self.canudsthread.sig_ecu_name.emit(self.ecu_name)

    def handle_error(self, error):
        self.canudsthread.sig_send_state.emit(error)
        text = 'Tx error:' + str(error) + '\r'
        print(text)

    # creates the uds client        
    def create_uds_client(self, bus, mode=29, txid=0x18DA05F1, rxid=0x18DAF105, functional_id=0x18DB33F1, dl="short", notif=True, flg=True):

        # default if you set them to None
        if txid is None:
            txid = 0x123
        if rxid is None:
            rxid = 0x234
        if functionalid is None:
            functionalid = 0x345

        # tp_addr = isotp.Address(isotp.AddressingMode.Normal_29bits, txid=0x18DA05F1, rxid=0x18DAF105, functional_id=0x18DB33F1)
        #
        # create isotp address
        #
        if mode == 29:
            tp_addr = isotp.Address(isotp.AddressingMode.Normal_29bits, txid=txid, rxid=rxid, functional_id=functionalid)
        elif mode == 11:
            tp_addr = isotp.Address(isotp.AddressingMode.Normal_11bits, txid=txid, rxid=rxid, functional_id=functionalid)	
			
        if dl == "long":
            tx_data_length = 64
        else:
            tx_data_length = 8
			
        isotpparams = {
            'blocking_send': False,
            'stmin': 32,
            # Will request the sender to wait 32ms between consecutive frame. 0-127ms or 100-900ns with values from 0xF1-0xF9
            'blocksize': 8,
            # Request the sender to send 8 consecutives frames before sending a new flow control message
            'wftmax': 0,                                                                   # Number of wait frame allowed before triggering an error
            'tx_data_length': tx_data_length,                                              # Link layer (CAN layer) works with 8 byte payload (CAN 2.0)
            # Minimum length of CAN messages. When different from None, messages are padded to meet this length. Works with CAN 2.0 and CAN FD.
            'tx_data_min_length': 8,
            'tx_padding': 0,                                                               # Will pad all transmitted CAN messages with byte 0x00.
            'rx_flowcontrol_timeout': 1000,
            # Triggers a timeout if a flow control is awaited for more than 1000 milliseconds
            'rx_consecutive_frame_timeout': 1000,
            # Time in seconds to wait between consecutive frames when transmitting.
            # When set, this value will override the receiver stmin requirement.
            # When None, the receiver stmin parameter will be respected.
            # This parameter can be useful to speed up a transmission by setting a value of 0 (send as fast as possible)
            # on a system that has low execution priority or coarse thread resolution
            'override_receiver_stmin': 0,
            'max_frame_size': 4095,                                                          # Limit the size of receive frame.
            'can_fd': (dl == "long"),                                                        # Does not set the can_fd flag on the output CAN messages
            'bitrate_switch': False,                                                         # Does not set the bitrate_switch flag on the output CAN messages
            'rate_limit_enable': False,                                                      # Disable the rate limiter
            'rate_limit_max_bitrate': 1000000,
            # Ignored when rate_limit_enable=False. Sets the max bitrate when rate_limit_enable=True
            'rate_limit_window_size': 0.2,
            # Ignored when rate_limit_enable=False. Sets the averaging window size for bitrate calculation when rate_limit_enable=True
            'listen_mode': False,                                                            # Does not use the listen_mode which prevent transmission.
        }

        uds_config = udsoncan.configs.default_client_config.copy()                           # load default config

        if notif == True:
            self.notifier = can.Notifier(bus, [])                                             # Add a debug listener that print all messages
            self.stack = isotp.NotifierBasedCanStack(bus=bus, notifier=self.notifier, address=tp_addr, params=isotpparams, error_handler=self.handle_error)  # Network/Transport layer (IsoTP protocol). Register a new listenenr
        else:
            self.stack = isotp.CanStack(bus=bus, address=tp_addr, params=isotpparams)         # isotp v1.x has no notifier support
            
        self.conn = PythonIsoTpConnection(self.stack)                                         # interface between Application and Transport layer

        with Client(conn, config=uds_config) as client:                                       # Application layer (UDS protocol)
            client.change_session(1)
        conn.close()
        stack.stop()

        config = dict(udsoncan.configs.default_client_config)
        config['p2_timeout'] = 1.5

        config['data_identifiers'] = { 'default': '>H',                                                      # Default codec is a struct.pack/unpack string. 16bits little endian
                                       0xF190: udsoncan.AsciiCodec(15),                                      # Codec that read ASCII string. We must tell the length of the string
                                       0xf110:MyCodec()  }

        # ---------- create uds client using the protocol stack defined above ------------------
        with Client(conn, request_timeout=2, config=config) as client:
            response = client.read_data_by_identifier([0xF190])
            print(response.service_data.values[0xF190])                                                       # This is a dict of DID:Value
            # Or, if a single DID is expected, a shortcut to read the value of the first DID
            vin = client.read_data_by_identifier_first(0xF190)
            print("VIN = ",str(vin))
            
        self.uds_client = Client(self.conn, config=config)
        self.uds_client.open()

        try:
            response = self.uds_client.test_data_identifier([0xF195])
            print(response)
        except TimeoutException as e:
            print(e)
        req = Request(services.WriteDataByIdentifier, data=bytes([0x00,0x11,0x22,0x33,0x44,0x55,0x66]))
        try:
            response = self.uds_client.send_request(req)
            if response.positive:
                data = response.data
                print(f"Data: {data.hex()}")
            else:
                print(f"Negative response: {response.code_name}")
        except TimeoutException as e:
            print(e)
        except:
            print('error')

        self.canudsthread = canUDSClientThread(conn=self.uds_client, send_queue=self.send_queue, dll_path=self.dll_path, ecus=self.ecus)
        self.canTesterPresentThread = canTesterPresentThread(conn=self.uds_client)

        self.canudsthread.send_data.connect(self.print_tx)
        self.canudsthread.rec_data.connect(self.print_rx)
        self.sig_dll_path.connect(self.canudsthread.load_dll)
        self.canudsthread.sig_send_state.connect(self.canudsthread.set_send_state)
        self.canudsthread.sig_ecu_name.connect(self.canudsthread.set_ecu_name)

        self.canudsthread.start()
        self.canTesterPresentThread.start()
        self.canudsthread.sig_flag_3e.connect(self.canTesterPresentThread.set_3e_flag(flg))

        self.is_run = True
        self.send_ecu_name()


		