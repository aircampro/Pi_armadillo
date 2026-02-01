# 
# driver for syanpticon (SOMANET Drive) ref:- https://doc-legacy.synapticon.com/software/41/object_dict/all_objects/index.html
# https://doc.synapticon.com/circulo/sw5.1/object_dict/all_objects.html
#
#
import canopen
import struct

class brakeObjPt1:
    def __init__(self, a, b, c, d, e ):
        self.pull_v_32 = a
        self.hold_v_32 = b
        self.pull_t_16 = c
        self.rs_8 = d
        self.dis_delay_16 = e

class encoder_config:
    def __init__(self):
        self.Sensor_port = 0
        self.Type = 0
        self.Resolution = 0
        self.Zero_vel_thres = 0
        self.Polarity = 0
        self.Singleturn_offset = 0
        self.Access_signal_type = 0
        self.Clock_freq = 0
        self.Frame_size = 0
        self.Multiturn_bits = 0
        self.Multiturn_first_bit_pos = 0
        self.Singleturn_bits = 0
        self.Singleturn_first_bit_pos = 0
        self.Timeout = 0
        self.CRC_polynomial = 0
        self.Maximum_tbusy = 0
        self.Status_bits_actval = 0
        self.Parity_type = 0
        self.First_clock_delay = 0
        self.Data_ordering = 0
        self.Endianness = 0
        self.Index_avail = 0
        self.Hall_sensor_port = 0
        self.Sinewave_cycles_rev = 0
        self.Sinewave_resolution = 0
        self.Sine_out_volt = 0
        self.Filter = 0
        self.Sampling_freq = 0

class brakeState:
    none = 0
    engaged = 1
    released = 2

class Synapticon:

	def __init__(self, slave:int=6, name:str, rev_units:float=360, conn_method=0):
		self.name = name
		self.rev_units = rev_units
		self.pos_factor = 4096 / rev_units
		self.pos_offset = conf.getfloat(name, 'offset', fallback=0.0)
        self.network = canopen.Network()
        self.node = canopen.RemoteNode(slave, '/path/to/object_dictionary.eds')
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
        self.node.nmt.state = 'OPERATIONAL'
        self.bs = brakeState()

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

    # Bit 0: Shows Negative Limit Switch Status (1 = Active, 0 = Inactive )
    # Bit 1: Shows Positive Limit Switch Status (1 = Active, 0 = Inactive )
    # Bit 2: Shows Home Switch Status (1 = Active, 0 = Inactive )
    # Bit 3: Shows Interlock Status (1 = Drive cannot be enabled, 0 = Drive can be enabled)
    # Bit 4 to 15: Reserved
    # The manufacturer-specific bits (16 to 31) show the state of the Drive's digital inputs.
    # Bit 16: Digital input 1
    # Bit 17: Digital input 2
    def get_di(self):
		return self.node.sdo[0x60FD].raw 

    # https://doc-legacy.synapticon.com/software/41/documentation_html/object_htmls/2401/index.html
    def get_ani1(self):
		return self.node.sdo[0x2401].raw 
    def get_ani2(self):
		return self.node.sdo[0x2402].raw
    def get_ani3(self):
		return self.node.sdo[0x2403].raw
    def get_ani4(self):
		return self.node.sdo[0x2404].raw
    def get_core_temp(self):
		return self.node.sdo[0x2030.1].raw
    def get_drive_temp(self):
		return self.node.sdo[0x2031.1].raw

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

    def send_brake1_cmd1(self, pu:int=15, ho:int=10, tm=100, rs=self.bs.engaged, dl:int=100):                      # not sure if we can set this way pls try it.
        b = brakeObjPt1(pu, ho, tm, rs, 100)                                                                       # set-up brake object
        v = struct.pack('IIHBH', b.pull_v_32, b.hold_v_32, b.pull_t_16, b.rs_8, b.dis_delay_16)                    # not sure if we can just bulk set with a byte-stream (try here)
        self.node.sdo[0x2004].raw = v

    def send_brake1_cmd2(self, pu:int=15, ho:int=10, tm:int=100, rs:int=self.bs.engaged, dl:int=100):
        b = brakeObjPt1(pu, ho, tm, rs=, dl)                                                                       # set-up brake object
        self.node.sdo[0x2004.1].raw = b.pull_v_32
        self.node.sdo[0x2004.2].raw = b.hold_v_32
        self.node.sdo[0x2004.3].raw = b.pull_t_16
        self.node.sdo[0x2004.4].raw = b.rs_8
        self.node.sdo[0x2004.5].raw = b.dis_delay_16

    def send_brake2_cmd2(self, rs:int=self.bs.engaged, md:int, prc:int, ov:int, sf:int):
        self.node.sdo[0x2004.7].raw = rs
        self.node.sdo[0x2004.8].raw = md
        self.node.sdo[0x2004.9].raw = prc
        self.node.sdo[0x2004.10].raw = ov
        self.node.sdo[0x2004.11].raw = sf

    def send_homing(self, a:int, b:int, c:int, d:int):
        self.node.sdo[0x2005.1].raw = a
        self.node.sdo[0x2005.2].raw = b
        self.node.sdo[0x2005.3].raw = c
        self.node.sdo[0x2005.4].raw = d

    def get_encoder_fb(self, enc_no:int):
        if enc_no == 1:
            raw_pos = self.node.sdo[0x2111.1].raw
            adj_pos = self.node.sdo[0x2111.2].raw
            velo = self.node.sdo[0x2111.3].raw
        elif enc_no == 2:
            raw_pos = self.node.sdo[0x2113.1].raw
            adj_pos = self.node.sdo[0x2113.2].raw
            velo = self.node.sdo[0x2113.3].raw

    def send_protection(self, a:int, b:int, c:int):
        self.node.sdo[0x2006.1].raw = a
        self.node.sdo[0x2006.2].raw = b
        self.node.sdo[0x2006.3].raw = c

    def cogging_torque(self, a:int):
        self.node.sdo[0x2008.2].raw = a

    def commutation_offset(self, a:int, b:int, c:int, d:float, e:float, f:float):
        self.node.sdo[0x2009.1].raw = a
        self.node.sdo[0x2009.2].raw = b
        self.node.sdo[0x2009.3].raw = c
        self.node.sdo[0x2009.4].raw = d
        self.node.sdo[0x2009.5].raw = e
        self.node.sdo[0x2009.6].raw = f

    def set_external_scaled_meas(self, a:int, b:int, c:float, d:float, e:float, f:float, g:float, h:float, i:int, j:float, k:float):
        self.node.sdo[0x2038.2].raw = a
        self.node.sdo[0x2038.3].raw = b
        self.node.sdo[0x2038.4].raw = struct.pack('f',c)
        self.node.sdo[0x2038.5].raw = struct.pack('f',d)
        self.node.sdo[0x2038.6].raw = struct.pack('f',e)
        self.node.sdo[0x2038.7].raw = struct.pack('f',f)
        self.node.sdo[0x2038.8].raw = struct.pack('f',g)
        self.node.sdo[0x2038.9].raw = struct.pack('f',h)
        self.node.sdo[0x2038.10].raw = i
        self.node.sdo[0x2038.11].raw = struct.pack('f',j)
        self.node.sdo[0x2038.12].raw = struct.pack('f',k)

    def set_encoder1(self, a:encoder_config):
        self.node.sdo[0x2110.1].raw = a.Sensor_port
        self.node.sdo[0x2110.2].raw = a.Type
        self.node.sdo[0x2110.3].raw = a.Resolution
        self.node.sdo[0x2110.4].raw = a.Zero_vel_thres
        self.node.sdo[0x2110.5].raw = a.Polarity
        self.node.sdo[0x2110.6].raw = a.Singleturn_offset
        self.node.sdo[0x2110.7].raw = a.Access_signal_type
        self.node.sdo[0x2110.8].raw = a.Clock_freq
        self.node.sdo[0x2110.9].raw = a.Frame_size
        self.node.sdo[0x2110.10].raw = a.Multiturn_bits
        self.node.sdo[0x2110.11].raw = a.Multiturn_first_bit_pos
        self.node.sdo[0x2110.12].raw = a.Singleturn_bits
        self.node.sdo[0x2110.13].raw = a.Singleturn_first_bit_pos
        self.node.sdo[0x2110.14].raw = a.Timeout
        self.node.sdo[0x2110.15].raw = a.CRC_polynomial
        self.node.sdo[0x2110.16].raw = a.Maximum_tbusy
        self.node.sdo[0x2110.17].raw = a.Status_bits_actval
        self.node.sdo[0x2110.18].raw = a.Parity_type
        self.node.sdo[0x2110.19].raw = a.First_clock_delay
        self.node.sdo[0x2110.20].raw = a.Data_ordering
        self.node.sdo[0x2110.21].raw = a.Endianness
        self.node.sdo[0x2110.22].raw = a.Index_avail
        self.node.sdo[0x2110.23].raw = a.Hall_sensor_port
        self.node.sdo[0x2110.24].raw = a.Sinewave_cycles_rev
        self.node.sdo[0x2110.25].raw = a.Sinewave_resolution
        self.node.sdo[0x2110.26].raw = a.Sine_out_volt
        self.node.sdo[0x2110.27].raw = a.Filter
        self.node.sdo[0x2110.28].raw = a.Sampling_freq

    def set_encoder2(self, a:encoder_config):
        self.node.sdo[0x2112.1].raw = a.Sensor_port
        self.node.sdo[0x2112.2].raw = a.Type
        self.node.sdo[0x2112.3].raw = a.Resolution
        self.node.sdo[0x2112.4].raw = a.Zero_vel_thres
        self.node.sdo[0x2112.5].raw = a.Polarity
        self.node.sdo[0x2112.6].raw = a.Singleturn_offset
        self.node.sdo[0x2112.7].raw = a.Access_signal_type
        self.node.sdo[0x2112.8].raw = a.Clock_freq
        self.node.sdo[0x2112.9].raw = a.Frame_size
        self.node.sdo[0x2112.10].raw = a.Multiturn_bits
        self.node.sdo[0x2112.11].raw = a.Multiturn_first_bit_pos
        self.node.sdo[0x2112.12].raw = a.Singleturn_bits
        self.node.sdo[0x2112.13].raw = a.Singleturn_first_bit_pos
        self.node.sdo[0x2112.14].raw = a.Timeout
        self.node.sdo[0x2112.15].raw = a.CRC_polynomial
        self.node.sdo[0x2112.16].raw = a.Maximum_tbusy
        self.node.sdo[0x2112.17].raw = a.Status_bits_actval
        self.node.sdo[0x2112.18].raw = a.Parity_type
        self.node.sdo[0x2112.19].raw = a.First_clock_delay
        self.node.sdo[0x2112.20].raw = a.Data_ordering
        self.node.sdo[0x2112.21].raw = a.Endianness
        self.node.sdo[0x2112.22].raw = a.Index_avail
        self.node.sdo[0x2112.23].raw = a.Hall_sensor_port
        self.node.sdo[0x2112.24].raw = a.Sinewave_cycles_rev
        self.node.sdo[0x2112.25].raw = a.Sinewave_resolution
        self.node.sdo[0x2112.26].raw = a.Sine_out_volt
        self.node.sdo[0x2112.27].raw = a.Filter
        self.node.sdo[0x2112.28].raw = a.Sampling_freq
    
    def get_external_scaled_meas(self):
        return self.node.sdo[0x2038.1].raw 

    def i2t_stall(self, a:int, b:int):
        self.node.sdo[0x200A.1].raw = a
        self.node.sdo[0x200A.2].raw = b

    def torque_window(self, a:int, b:int):
        self.node.sdo[0x2014.1].raw = a
        self.node.sdo[0x2014.2].raw = b

    def velocity_feedforward(self, a:int, b:int):
        self.node.sdo[0x2015.1].raw = a
        self.node.sdo[0x2015.2].raw = b

    def velocity_feedback_filter(self, enab:int, freq:int):
        self.node.sdo[0x2021.1].raw = enab
        self.node.sdo[0x2021.2].raw = freq

    def position_feedback_filter(self, enab:int, freq:int):
        self.node.sdo[0x2022.1].raw = enab
        self.node.sdo[0x2022.2].raw = freq

    def notch_filter(self, enab:int, freq:int, band:int):
        self.node.sdo[0x2023.1].raw = enab
        self.node.sdo[0x2023.2].raw = freq
        self.node.sdo[0x2023.3].raw = band

    def control_input_FIR(self, enab:int, order:int):
        self.node.sdo[0x2027.1].raw = enab
        self.node.sdo[0x2027.2].raw = order

    def set_max_power(self, a:int):
        if a <= 0 or a > 2147483647:
            return
        self.node.sdo[0x200B].raw = a

    def following_error(self, a:int):
        if a <= 0 or a > 2:
            return
        self.node.sdo[0x2017.1].raw = a

    def torque_controller(self, p:float, i:float, d:float, a:int, b:int, c:int, dd:int, e:int, f:int, g:int, h:int, i:int):
        self.node.sdo[0x2010.1].raw = struct.pack('f',p)
        self.node.sdo[0x2010.2].raw = struct.pack('f',i)
        self.node.sdo[0x2010.3].raw = struct.pack('f',d)
        self.node.sdo[0x2010.4].raw = a
        self.node.sdo[0x2010.5].raw = b
        self.node.sdo[0x2010.6].raw = c
        self.node.sdo[0x2010.7].raw = dd
        self.node.sdo[0x2010.8].raw = e
        self.node.sdo[0x2010.9].raw = f
        self.node.sdo[0x2010.10].raw = g
        self.node.sdo[0x2010.11].raw = h
        self.node.sdo[0x2010.12].raw = i

    def position_controller(self, p:float, i:float, d:float, a:int, pp:float, ii:float, dd:float, e:int, f:int):
        self.node.sdo[0x2012.1].raw = struct.pack('f',p)
        self.node.sdo[0x2012.2].raw = struct.pack('f',i)
        self.node.sdo[0x2012.3].raw = struct.pack('f',d)
        self.node.sdo[0x2012.4].raw = a
        self.node.sdo[0x2012.5].raw = struct.pack('f',pp)
        self.node.sdo[0x2012.6].raw = struct.pack('f',ii)
        self.node.sdo[0x2012.7].raw = struct.pack('f',dd)
        self.node.sdo[0x2012.8].raw = e
        self.node.sdo[0x2012.9].raw = f

    def velocity_controller(self, p:float, i:float, d:float, a:int, b:int):
        self.node.sdo[0x2011.1].raw = struct.pack('f',p)
        self.node.sdo[0x2011.2].raw = struct.pack('f',i)
        self.node.sdo[0x2011.3].raw = struct.pack('f',d)
        self.node.sdo[0x2011.4].raw = a
        self.node.sdo[0x2011.5].raw = b

