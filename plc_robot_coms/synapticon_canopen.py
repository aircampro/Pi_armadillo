# 
# driver for syanpticon (SOMANET Drive) ref:- https://doc-legacy.synapticon.com/software/41/object_dict/all_objects/index.html
#
import canopen

class Synapticon:

	def __init__(self, slave:int, name:str, rev_units:float=360, conn_method=0):
		super().__init__(slave)
		self.name = name
		self.rev_units = rev_units
		self.pos_factor = 4096 / rev_units
		self.pos_offset = conf.getfloat(name, 'offset', fallback=0.0)
        self.network = canopen.Network()
        self.node = canopen.RemoteNode(6, '/path/to/object_dictionary.eds')
        self.network.add_node(node)
        # (see https://python-can.readthedocs.io/en/latest/bus.html).
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

    # https://doc-legacy.synapticon.com/software/41/documentation_html/object_htmls/6075/index.html
	def initialize(self):
        self.max_current = self.node.sdo[0x6075].raw 

    def sync_net(self):
	    self.network.sync.start(0.1)

    def sync_stop(self):
	    self.network.sync.stop()

    def discon_net(self):
        self.network.disconnect()

    # https://doc-legacy.synapticon.com/software/41/documentation_html/object_htmls/6064/index.html
	def get_internal_pos(self):
		"""6064h: Position Actual Internal Value"""
		return self.node.sdo[0x6064].raw / self.pos_factor

    # 0x6077 Torque Value
    def get_actual_torque(self):
		return self.node.sdo[0x6077].raw 

    def get_velocity(self):
		return self.node.sdo[0x606C].raw 

    # https://doc-legacy.synapticon.com/software/41/documentation_html/object_htmls/2401/index.html
    def get_ani1(self):
		return self.node.sdo[0x2401].raw 
    def get_ani2(self):
		return self.node.sdo[0x2402].raw
    def get_ani3(self):
		return self.node.sdo[0x2403].raw
    def get_ani4(self):
		return self.node.sdo[0x2404].raw
	
    # https://doc-legacy.synapticon.com/software/41/documentation_html/object_htmls/6073/index.html	
	def set_current(self, torque:float=100):
        current = int(self.max_current * torque / 100)
        self.node.sdo[0x6073].raw = current

    # https://doc-legacy.synapticon.com/software/41/documentation_html/object_htmls/6071/index.html
	def set_torque(self, torque:float=100):
        self.node.sdo[0x6071].raw = torque

	def set_max_torque(self, mtorque:float=100):
        self.node.sdo[0x6072].raw = mtorque

    # https://doc-legacy.synapticon.com/software/41/documentation_html/object_htmls/607A/index.html
	def set_position(self, pos:float=100):
        self.node.sdo[0x607A].raw = pos

	def set_mtr_shaft_rev(self, pos:float=100):
        self.node.sdo[0x6091].raw = pos

    # https://doc-legacy.synapticon.com/software/41/documentation_html/object_htmls/60FF/index.html
	def set_velocity(self, vel:float=100):
        self.node.sdo[0x60FF].raw = vel

    # https://doc-legacy.synapticon.com/software/41/documentation_html/object_htmls/6060/index.html
    def set_mode(self, md:int=8):                          # position is default
        self.node.sdo[0x6060].raw = md

    # Command value for Digital IO of SOMANET Drive. Only available if not used otherwise, e.g. for SPI
    # https://doc-legacy.synapticon.com/software/41/documentation_html/object_htmls/2601/index.html
    def set_do1(self, b=1):
        self.node.sdo[0x2601].raw = b
    def set_do2(self, b=1):
        self.node.sdo[0x2602].raw = b
    def set_do3(self, b=1):
        self.node.sdo[0x2603].raw = b
    def set_do4(self, b=1):
        self.node.sdo[0x2604].raw = b
    # https://doc-legacy.synapticon.com/software/41/documentation_html/object_htmls/6040/index.html
    def set_cw(self, b=0b10):                                          # default quickstop
        self.node.sdo[0x6040].raw = b
    def switch_on_enab(self, b=0b1111):                                 # switch on and enable                              
        self.node.sdo[0x6040].raw = b
    def shutdown_drive(self, b=0b110):                                  # default shutdown
        self.node.sdo[0x6040].raw = b
    def fault_reset(self):                                
        self.node.sdo[0x6040].raw |= 0b10000000
	
	def set_following_error(self, error:float|None=None):
		if error is None:
			error = 0.1 * self.rev_units
        error_drive = int(error * self.pos_factor)
        self.node.sdo[0x6065].raw = error_drive		

	def set_baud_rate(self, baud_rate:Literal[1000,500,250]):
		"""
		Set the baud rate of the drive in the CANopen network.
		
		:param baud_rate: CAN Baud rate in Kbit/s
		"""
        values = {
			1000:	0,
			500:	2,
			250:	3,
		}
        self.node.sdo[0x2F1F].raw = values[baud_rate]			
		self._save()

	def _save(self):
        self.node.sdo[0x1010].raw = 0x65766173
