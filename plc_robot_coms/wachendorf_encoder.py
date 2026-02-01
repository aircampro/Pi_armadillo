# 
# driver for wachendorf
# https://www.manualslib.com/manual/1552378/Wachendorff-Wdga-Canopen.html?page=76#manual
#
#
import canopen

class eeprom_mode:
    all = 1
    comm = 2
    app = 3
    manu = 4

class Wachendorf:

	def __init__(self, slave:int=127, name:str, conn_method:int=0):
		self.name = name
        self.network = canopen.Network()
        self.node = canopen.RemoteNode(slave, 'wdga-st-co_2013-04-22.eds')
        self.network.add_node(node)
        if conn_method == 0:
            self.network.connect()
        elif conn_method == 1:
            self.network.connect(interface='socketcan', channel='can0')
        elif conn_method == 2:
            self.network.connect(interface='kvaser', channel=0, bitrate=250000)
        elif conn_method == 3:
            self.network.connect(interface='pcan', channel='PCAN_USBBUS1', bitrate=250000)
        elif conn_method == 4:
            self.network.connect(interface='ixxat', channel=0, bitrate=250000)
        elif conn_method == 5:
            self.network.connect(interface='vector', app_name='CANalyzer', channel=0, bitrate=250000)
        elif conn_method == 6:
            self.network.connect(interface='nican', channel='CAN0', bitrate=250000)
        self.node.nmt.state = 'OPERATIONAL'
        self.emode = eeprom_mode()

    def sync_net(self):
	    self.network.sync.start(0.1)

    def sync_stop(self):
	    self.network.sync.stop()

    def discon_net(self):
        self.network.disconnect()

    def get_velocity(self):
		return self.node.sdo[0x6030].raw 

    def get_v1(self):
		return self.node.sdo[0x6030.1].raw 

    def get_v2(self):
		return self.node.sdo[0x6030.2].raw 

    def get_position(self):
		return self.node.sdo[0x6004].raw

    def set_resolution(self, res:int):
		return self.node.sdo[0x6001].raw = res

    def set_integration_time_ms(self, res:int=1000):
		return self.node.sdo[0x2105.2].raw = res

    def set_cam_config(self, res:int=0x7):                       # default first 3 cams
		return self.node.sdo[0x6301].raw = res

    def set_speed_scale(self, res:int):
		return self.node.sdo[0x2106.1].raw = res

    def set_freq_lim(self, res:int):
		return self.node.sdo[0x2107].raw = res
      
    def set_meas_range(self, res:int):
		return self.node.sdo[0x6002].raw = res

    def set_pos_preset(self, res:int):
		return self.node.sdo[0x6003].raw = res

    def eeprom_save(self, md:int=self.emode.all):
		return self.node.sdo[0x1010.md].raw = 0x65766173

    def eeprom_recall(self, md:int=self.emode.all):
		return self.node.sdo[0x1011.md].raw = 0x65766173