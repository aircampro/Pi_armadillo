# the starter for this class was taken from here 
# https://github.com/cpvalente/stupidArtnet/blob/master/stupidArtnet/StupidArtnet.py
#
"""Implementation of Artnet.

Python Version: 3.6
Source: http://artisticlicence.com/WebSiteMaster/User%20Guides/art-net.pdf

NOTES
- For simplicity: NET and SUBNET not used by default but optional

"""

import socket
import _thread
from time import sleep

class Artnet_4():
    """implementation of Artnet4 protocol to dmx over ethernet."""

    UDP_PORT = 6454
    # artnet4 op codes
    ARTNET_POLL = 0x2000                     #  This is an ArtPoll packet, no other data is contained in this UDP packet.
    ARTNET_REPLY = 0x2100                    # This is an ArtPollReply Packet. It contains device status information.
    ARTNET_OpDiagData = 0x2300               # Diagnostics and data logging packet
    ARTNET_OpCommand = 0x2400                # Used to send text based parameter commands.
    ARTNET_DMX = 0x5000                      # This is an ArtDmx data packet. It contains zero start code DMX512 information for a single Universe
    ARTNET_OpNzs = 0x5100                    # This is an ArtNzs data packet. It contains non-zero start code (except RDM) DMX512 information for a single Universe
    ARTNET_OpSync = 0x5200                   # This is an ArtSync data packet. It is used to force synchronous transfer of ArtDmx packets to a node’s output
    ARTNET_ADDRESS = 0x6000                  # This is an ArtAddress packet. It contains remote programming information for a Node.
    ARTNET_INPUT = 0x7000                    # This is an ArtInput packet. It contains enable – disable data for DMX inputs
    ARTNET_TODREQUEST = 0x8000               # This is an ArtTodRequest packet. It is used to request a Table of Devices (ToD) for RDM discovery.
    ARTNET_TODDATA = 0x8100                  # This is an ArtTodData packet. It is used to send a Table of Devices (ToD) for RDM discovery.
    ARTNET_TODCONTROL = 0x8200               # This is an ArtTodControl packet. It is used to send RDM discovery control messages
    ARTNET_RDM = 0x8300                      # This is an ArtRdm packet. It is used to send all non discovery RDM messages.
    ARTNET_OpRdmSub = 0x8400                 # This is an ArtRdmSub packet. It is used to send compressed, RDM Sub-Device data.
    ARTNET_VIDEOSTEUP = 0xa010               # This is an ArtVideoSetup packet. It contains video screen setup information for nodes that implement the extended video features
    ARTNET_VIDEOPALETTE = 0xa020             # This is an ArtVideoPalette packet. It contains colour palette setup information for nodes that implement the extended video features.
    ARTNET_VIDEODATA = 0xa040                # This is an ArtVideoData packet. It contains display data for nodes that implement the extended video features.
    ARTNET_MACMASTER = 0xf000                # This packet is deprecated
    ARTNET_MACSLAVE = 0xf100                 # This packet is deprecated
    ARTNET_FIRMWAREMASTER = 0xf200           # This is an ArtFirmwareMaster packet. It is used to upload new firmware or firmware extensions to the Node
    ARTNET_FIRMWAREREPLY = 0xf300            # This is an ArtFirmwareReply packet. It is returned by the node to acknowledge receipt of an ArtFirmwareMaster packet or ArtFileTnMaster packet.
    ARTNET_OpFileTnMaster = 0xf400           # Uploads file to the node
    ARTNET_OpFileFnMaster = 0xf500           # Downloads user file from node.
    ARTNET_OpFileFnReply = 0xf600            # Server to Node acknowledge for download packets.
    ARTNET_IPPROG = 0xf800                   # This is an ArtIpProg packet. It is used to reprogramme the IP address and Mask of the Node.
    ARTNET_IPREPLY = 0xf900                  # This is an ArtIpProgReply packet. It is returned by the node to acknowledge receipt of an ArtIpProg packet.
    ARTNET_MEDIA = 0x9000                    # This is an ArtMedia packet. It is Unicast by a Media Server and acted upon by a Controller.
    ARTNET_OpMediaPatch = 0x9100             # This is an ArtMediaPatch packet. It is Unicast by a Controller and acted upon by a Media Server
    ARTNET_MEDIAPATCH = 0x9200               # This is an ArtMediaControl packet. It is Unicast by a Controller and acted upon by a Media Server
    ARTNET_MEDIACONTROLREPLY = 0x9300        # This is an ArtMediaControlReply packet. It is Unicast by a Media Server and acted upon by a Controller.
    ARTNET_OpTimeCode = 0x9700               # This is an ArtTimeCode packet. It is used to transport time code over the network.
    ARTNET_OpTimeSync = 0x9800               # Used to synchronise real time date and clock
    ARTNET_OpTrigger = 0x9900                # Used to send trigger macros
    ARTNET_OpDirectory = 0x9a00              # Requests a node's file list
    ARTNET_OpDirectoryReply = 0x9b00         # Replies to OpDirectory with file list

    # port types
    ARTNET_PORT_DMX512 =0b000000                                             # DMX512
    ARTNET_PORT_MIDI =0b000001                                               # Midi
    ARTNET_PORT_AVAB =0b000010                                               # Avab
    ARTNET_PORT_CMX =0b000011                                                # ColorTran CMX
    ARTNET_PORT_ADB62 =0b000100                                              # ADB 62.5
    ARTNET_PORT_ARTNET =0b000101                                             # ARTNET
    ARTNET_PORT_INPUT_ON =(1u<<6u)                                           # Set if this channel can input onto the Art-Net Network.
    ARTNET_PORT_OUTPUT_ON =(1u<<7u)                                          # Set is this channel can output data from the Art-Net Network. 
    PHYS_PORT=ARTNET_PORT_DMX512    

    # node types
    ARTNET_SRV = 0                                                           # An ArtNet server (transmitts DMX data)
    ARTNET_NODE = 1                                                          # An ArtNet node   (dmx reciever) 
    ARTNET_MSRV = 2                                                          # A Media Server 
    ARTNET_ROUTE = 3                                                         # No Effect currently 
    ARTNET_BACKUP = 4                                                        # No Effect currently 
    ARTNET_RAW = 5                                                           # Raw Node - used for diagnostics     
    
    def __init__(self, target_ip='127.0.0.1', universe=0, packet_size=512, fps=30,
                 even_packet_size=True, broadcast=False, source_address=None, artsync=False):
        """Initializes Art-Net Client.

        Args:
        targetIP - IP of receiving device
        universe - universe to listen
        packet_size - amount of channels to transmit
        fps - transmition rate
        even_packet_size - Some receivers enforce even packets
        broadcast - whether to broadcast in local sub
        artsync - if we want to synchronize buffer

        The driver has all headers but some commands are untried
        
        Returns:
        None

        """
        # Instance variables
        self.target_ip = target_ip
        self.sequence = 0                                                      # consider stroing and reading from a file store
        self.physical = 0
        self.universe = universe
        self.subnet = 0
        self.if_sync = artsync
        self.net = 0
        self.packet_size = packet_size
        self.packet_header = bytearray()
        self.buffer = bytearray(self.packet_size)
        self.vlc_packet = bytearray(self.packet_size)
        self.rdm_packet = bytearray(231)                                        # rdm is only 231 bytes       
        self.make_even = even_packet_size
        self.is_simplified = True		                                        # simplify use of universe, net and subnet

        # UDP SOCKET
        self.socket_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        if broadcast:
            self.socket_client.setsockopt(
                socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        # Allow speciying the origin interface
        if source_address:
            self.socket_client.setsockopt(
                socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket_client.bind(source_address)

        # Timer
        self.fps = fps
        self.__clock = None

        self.make_artdmx_header()
        
        if self.if_sync:
            self.artsync_header = bytearray()
            self.make_artsync_header()

    def __exit__(self):
        """Graceful shutdown."""
        self.stop()
        self.close()
        
    def __del__(self):
        """Graceful shutdown."""
        self.stop()
        self.close()

    def __str__(self):
        """Printable object state."""
        state = "===================================\n"
        state += "Artnet 4 is initialized\n"
        state += f"Target IP: {self.target_ip} : {self.UDP_PORT} \n"
        state += f"Universe: {self.universe} \n"
        if not self.is_simplified:
            state += f"Subnet: {self.subnet} \n"
            state += f"Net: {self.net} \n"
        state += f"Packet Size: {self.packet_size} \n"
        state += "==================================="

        return state

    def clamp(self, number, min_val, max_val):
        """Utility method: sets number in defined range.

        Args:
        number - number to use
        range_min - lowest possible number
        range_max - highest possible number

        Returns:
        number - number in correct range
        """
        return max(min_val, min(number, max_val))
    
    def set_even(self, number):
        """Utility method: ensures number is even by adding.

        Args:
        number - number to make even

        Returns:
        number - even number
        """
        if number % 2 != 0:
            number += 1
        return number

    def put_in_range(self, number, range_min, range_max, make_even=True):
        """Utility method: sets number in defined range.

        Args:
        number - number to use
        range_min - lowest possible number
        range_max - highest possible number
        make_even - should number be made even

        Returns:
        number - number in correct range

        """
        number = self.clamp(number, range_min, range_max)
        if make_even:
            number = self.set_even(number)
        return number
    
    def shift_this(num_in, endian_swap_byte=False):
        first_byte = hex(num_in & 0xFF)                    # LSB
        second_byte = hex(((num_in & 0xFF00)>>8))          # MSB
        if not endian_swap_byte:
            return first_byte, second_byte
        else:
            return second_byte, first_byte
        
    def get_op_code(self, msg_typ, endian_swap_byte=False):
        first_byte = hex(msg_typ & 0xFF)
        second_byte = hex(((msg_typ & 0xFF00)>>8))
        if not endian_swap_byte:
            return first_byte, second_byte
        else:
            return second_byte, first_byte

    def init_packet_sizes(self, packet_size=512, rdm_pkt_size=231):
        self.packet_size = self.put_in_range(packet_size, 2, 512, self.make_even)
        rdm_pkt_sz = self.put_in_range(rdm_pkt_size, 1, 231, False)
        self.packet_header = bytearray()
        self.buffer = bytearray(self.packet_size)
        self.vlc_packet = bytearray(self.packet_size)
        self.rdm_packet = bytearray(rdm_pkt_sz) 
        
    # At power on or reset a node shall operate in non-synchronous mode. 
    # This means that ArtDmx packets will be immediately processed and output.
    # When a node receives an ArtSync packet it should transfer to synchronous operation. 
    # This means that received ArtDmx packets will be buffered and output when the next ArtSync is received.
    # In order to allow transition between synchronous and non-synchronous modes, a node shall time out 
    # to non-synchronous operation if an ArtSync is not received for 4 seconds or more.
        
    def make_artdmx_header(self):
        """Make packet header."""
        # 0 - id (7 x bytes + Null)
        self.packet_header = bytearray()
        self.packet_header.extend(bytearray('Art-Net', 'utf8'))
        self.packet_header.append(0x0)
        # 8 - opcode (2 x 8 low byte first)
        b1, b2 = self.get_op_code(self.ARTNET_DMX)         # ArtDmx data packet
        self.packet_header.append(b1)
        self.packet_header.append(b2)          
        #self.packet_header.append(0x00)
        #self.packet_header.append(0x50)  
        # 10 - prototocol version (2 x 8 high byte first)
        self.packet_header.append(0x0)
        self.packet_header.append(14)
        # 12 - sequence (int 8), NULL for not implemented
        # The sequence number is used to ensure that ArtDmx packets are used in the correct order. 
        # When Art-Net is carried over a medium such as the Internet, it is possible that ArtDmx packets 
        # will reach the receiver out of order. This field is incremented in the range 0x01 to 0xff to allow the receiving node 
        # to resequence packets. The Sequence field is set to 0x00 to disable this feature
        self.packet_header.append(self.sequence)
        # 13 - physical port (int 8)
        self.packet_header.append(self.PHYS_PORT)
        # 14 - universe, (2 x 8 low byte first)
        if self.is_simplified:
            # not quite correct but good enough for most cases:
            # the whole net subnet is simplified
            # by transforming a single uint16 into its 8 bit parts
            # you will most likely not see any differences in small networks
            msb, lsb = self.shift_this(self.universe)   # convert to MSB / LSB
            self.packet_header.append(lsb)
            self.packet_header.append(msb)
        # 14 - universe, subnet (2 x 4 bits each)
        # 15 - net (7 bit value)
        else:
            # as specified in Artnet 4 (remember to set the value manually after):
            # Bit 3  - 0 = Universe (1-16)
            # Bit 7  - 4 = Subnet (1-16)
            # Bit 14 - 8 = Net (1-128)
            # Bit 15     = 0
            # this means 16 * 16 * 128 = 32768 universes per port
            # a subnet is a group of 16 Universes
            # 16 subnets will make a net, there are 128 of them
            self.packet_header.append(self.subnet << 4 | self.universe)
            self.packet_header.append(self.net & 0xFF)
        # 16 - packet size (2 x 8 high byte first)
        msb, lsb = self.shift_this(self.packet_size)		# convert to MSB / LSB
        self.packet_header.append(msb)
        self.packet_header.append(lsb)

    # This packet is used to request the Table of RDM Devices (TOD). 
    # A Node receiving this packet must not interpret it as forcing full discovery. 
    # Full discovery is only initiated at power on or when an ArtTodControl.AtcFlush is received. 
    # The response is ArtTodData.
    def make_arttodreq_header(self,portadd,addr):
        """Make packet header."""
        # 0 - id (7 x bytes + Null)
        self.packet_header = bytearray()
        self.packet_header.extend(bytearray('Art-Net', 'utf8'))
        self.packet_header.append(0x0)
        # 8 - opcode (2 x 8 low byte first)
        b1, b2 = self.get_op_code(self.ARTNET_TODREQUEST)         # Arttod request data packet
        self.packet_header.append(b1)
        self.packet_header.append(b2)          
        #self.packet_header.append(0x00)
        #self.packet_header.append(0x50)  
        # 10 - prototocol version (2 x 8 high byte first)
        self.packet_header.append(0x0)
        self.packet_header.append(14)
        for z in range(0,9):
            self.packet_header.append(0x0) 
        first_byte, second_byte = self.shift_this(portadd)
        self.packet_header.append(first_byte)
        self.packet_header.append(0x0)                            # send all  
        self.packet_header.append(addr)        
        self.packet_header.append(second_byte)             

    # action=1 flush AtcNone=0 AtcFlush=1 The node flushes its TOD and instigates full discovery
    def make_arttodcontrol_header(self, portadd, action=1):
        """Make packet header."""
        # 0 - id (7 x bytes + Null)
        self.packet_header = bytearray()
        self.packet_header.extend(bytearray('Art-Net', 'utf8'))
        self.packet_header.append(0x0)
        # 8 - opcode (2 x 8 low byte first)
        b1, b2 = self.get_op_code(self.ARTNET_TODCONTROL)         # Arttod control data packet
        self.packet_header.append(b1)
        self.packet_header.append(b2)          
        #self.packet_header.append(0x00)
        #self.packet_header.append(0x50)  
        # 10 - prototocol version (2 x 8 high byte first)
        self.packet_header.append(0x0)
        self.packet_header.append(14)
        for z in range(0,9):
            self.packet_header.append(0x0) 
        first_byte, second_byte = self.shift_this(portadd)
        self.packet_header.append(first_byte)
        self.packet_header.append(action)                            # action as above     
        self.packet_header.append(second_byte)    

    # remote data management header
    def make_artrdm_header(self, portadd, rdm_ver=0x1, cmd=0x0):
        """Make packet header."""
        # 0 - id (7 x bytes + Null)
        self.packet_header = bytearray()
        self.packet_header.extend(bytearray('Art-Net', 'utf8'))
        self.packet_header.append(0x0)
        # 8 - opcode (2 x 8 low byte first)
        b1, b2 = self.get_op_code(self.ARTNET_RDM)         # Artrdm data packet
        self.packet_header.append(b1)
        self.packet_header.append(b2)          
        #self.packet_header.append(0x00)
        #self.packet_header.append(0x50)  
        # 10 - prototocol version (2 x 8 high byte first)
        self.packet_header.append(0x0)
        self.packet_header.append(14)
        # rdm version Art-Net Devices that only support RDM DRAFT V1.0 set field to 0x00. 
        # Devices that support RDM STANDARD V1.0 set field to 0x01.
        self.packet_header.append(rdm_ver)
        for z in range(0,9):
            self.packet_header.append(0x0) 
        first_byte, second_byte = self.shift_this(portadd)
        self.packet_header.append(first_byte)
        self.packet_header.append(cmd)                            # cmd ArProcess=0 Process RDM Packet.     
        self.packet_header.append(second_byte)  

    # remote data management message
    def make_artrdm_header_as_msg(self, portadd, rdm_ver=0x1, cmd=0x0, dataV):
        """Make packet header."""
        # 0 - id (7 x bytes + Null)
        self.packet_header = bytearray()
        self.packet_header.extend(bytearray('Art-Net', 'utf8'))
        self.packet_header.append(0x0)
        # 8 - opcode (2 x 8 low byte first)
        b1, b2 = self.get_op_code(self.ARTNET_RDM)         # Artrdm data packet
        self.packet_header.append(b1)
        self.packet_header.append(b2)          
        #self.packet_header.append(0x00)
        #self.packet_header.append(0x50)  
        # 10 - prototocol version (2 x 8 high byte first)
        self.packet_header.append(0x0)
        self.packet_header.append(14)
        # rdm version Art-Net Devices that only support RDM DRAFT V1.0 set field to 0x00. 
        # Devices that support RDM STANDARD V1.0 set field to 0x01.
        self.packet_header.append(rdm_ver)
        for z in range(0,9):
            self.packet_header.append(0x0) 
        first_byte, second_byte = self.shift_this(portadd)
        self.packet_header.append(first_byte)
        self.packet_header.append(cmd)                            # cmd ArProcess=0 Process RDM Packet.     
        self.packet_header.append(second_byte)  
        self.packet_header.extend(dataV)                          # append the rdm data packet to send
        
    # remote data management header
    def make_artrdmsub_header(self, portadd, rdm_ver=0x1, uid, comm_class, param, sd=0, sd_count=1, cmd=0x0):
        """Make packet header."""
        # 0 - id (7 x bytes + Null)
        self.packet_header = bytearray()
        self.packet_header.extend(bytearray('Art-Net', 'utf8'))
        self.packet_header.append(0x0)
        # 8 - opcode (2 x 8 low byte first)
        b1, b2 = self.get_op_code(self.ARTNET_OpRdmSub)         # Artrdmsub data packet
        self.packet_header.append(b1)
        self.packet_header.append(b2)          
        #self.packet_header.append(0x00)
        #self.packet_header.append(0x50)  
        # 10 - prototocol version (2 x 8 high byte first)
        self.packet_header.append(0x0)
        self.packet_header.append(14)
        # rdm version Art-Net Devices that only support RDM DRAFT V1.0 set field to 0x00. 
        # Devices that support RDM STANDARD V1.0 set field to 0x01.
        self.packet_header.append(rdm_ver)
        self.packet_header.append(0x0) 
        self.packet_header.extend(uid)
        self.packet_header.append(0x0) 
        self.packet_header.append(com_clasa)             # As per RDM specification. This field defines whether this is a Get, Set, GetResponse, SetResponse.
        self.packet_header.append(param)                 # As per RDM specification. This field defines the type of parameter contained in this packet. Bigendian.
        self.packet_header.append(sd)                    # Defines the first device information contained in packet. This follows the RDM convention that 0 = root device and 1 = first subdevice. Big-endian.
        self.packet_header.append(sd_count)              # The number of sub devices packed into packet. Zero is illegal. Big-endian.
        for zz in range(0,4):
            self.packet_header.append(0x0)         
        self.packet_header.append(cmd)                   # cmd ArProcess=0 Process RDM Packet.     

    # remote data management header
    def make_artrdmsub_as_msg(self, portadd, rdm_ver=0x1, uid, comm_class, param, sd=0, sd_count=1, cmd=0x0, dataV):
        """Make packet header."""
        # 0 - id (7 x bytes + Null)
        self.packet_header = bytearray()
        self.packet_header.extend(bytearray('Art-Net', 'utf8'))
        self.packet_header.append(0x0)
        # 8 - opcode (2 x 8 low byte first)
        b1, b2 = self.get_op_code(self.ARTNET_OpRdmSub)         # Artrdmsub data packet
        self.packet_header.append(b1)
        self.packet_header.append(b2)          
        #self.packet_header.append(0x00)
        #self.packet_header.append(0x50)  
        # 10 - prototocol version (2 x 8 high byte first)
        self.packet_header.append(0x0)
        self.packet_header.append(14)
        # rdm version Art-Net Devices that only support RDM DRAFT V1.0 set field to 0x00. 
        # Devices that support RDM STANDARD V1.0 set field to 0x01.
        self.packet_header.append(rdm_ver)
        self.packet_header.append(0x0) 
        self.packet_header.extend(uid)
        self.packet_header.append(0x0) 
        self.packet_header.append(com_clasa)             # As per RDM specification. This field defines whether this is a Get, Set, GetResponse, SetResponse.
        self.packet_header.append(param)                 # As per RDM specification. This field defines the type of parameter contained in this packet. Bigendian.
        self.packet_header.append(sd)                    # Defines the first device information contained in packet. This follows the RDM convention that 0 = root device and 1 = first subdevice. Big-endian.
        self.packet_header.append(sd_count)              # The number of sub devices packed into packet. Zero is illegal. Big-endian.
        for zz in range(0,4):
            self.packet_header.append(0x0)         
        self.packet_header.append(cmd)                   # cmd ArProcess=0 Process RDM Packet. 
        self.packet_header.extend(dataV)

    def rdm_data_packet(self, len, dest_id, src_id, TransNo, ResponseType, subDevNo=0, cmd_class, param_id, dl, dataV ): 
        self.rdm_packet = bytearray()
        self.rdm_packet.append(0xCC)                  # CC fro RDM
        self.rdm_packet.append(0x01)                  # start code 0x1 for RDM
        self.rdm_packet.sppend(len)        
        self.rdm_packet.extend(dest_id)  
        self.rdm_packet.extend(src_id) 
        self.rdm_packet.sppend(TransNo) 
        self.rdm_packet.sppend(ResponseType) 
        self.rdm_packet.append(0x00)
        self.rdm_packet.sppend(subDevNo)              # sub device number (root = 0)
        self.rdm_packet.sppend(cmd_class)
        self.rdm_packet.sppend(param_id)
        self.rdm_packet.sppend(dl)     
        self.rdm_packet.extend(dataV)
  
    ARTNET_FW_TYPE_FirmFirst = 0x00                                          # The first packet of a firmware upload.
    ARTNET_FW_TYPE_FirmCont = 0x01                                           # The continuation packet of a firmware upload.
    ARTNET_FW_TYPE_FirmLast = 0x02                                           # The last packet of a firmware upload.
    ARTNET_FW_TYPE_UbeaFirst = 0x03                                          # The first packet of a UBEA upload.
    ARTNET_FW_TYPE_UbeaCont = 0x04                                           # The continuation packet of a UBEA upload.
    ARTNET_FW_TYPE_UbeaLast = 0x05                                           # The last packet of a UBEA upload.
    # send a firmware file
    def make_artfirmware_header(self,type_id=self.ARTNET_FW_TYPE_FirmFirst,block_id,fware_sz,endian_swap=False):
        """Make packet header."""
        # 0 - id (7 x bytes + Null)
        self.packet_header = bytearray()
        self.packet_header.extend(bytearray('Art-Net', 'utf8'))
        self.packet_header.append(0x0)
        # 8 - opcode (2 x 8 low byte first)
        b1, b2 = self.get_op_code(self.ARTNET_FIRMWAREMASTER)         # Arttod request data packet
        self.packet_header.append(b1)
        self.packet_header.append(b2)          
        #self.packet_header.append(0x00)
        #self.packet_header.append(0x50)  
        # 10 - prototocol version (2 x 8 high byte first)
        self.packet_header.append(0x0)
        self.packet_header.append(14)
        for z in range(0,2):
            self.packet_header.append(0x0) 
        self.packet_header.append(type_id)
        self.packet_header.append(block_id)
        first_byte = hex(fware_sz & 0xFF)
        second_byte = hex(((fware_sz & 0xFF00)>>8))
        third_byte = hex(((fware_sz & 0xFF0000)>>16))
        fourth_byte = hex(((fware_sz & 0xFF000000)>>24))
        if not endian_swap:
            self.packet_header.append(first_byte)
            self.packet_header.append(second_byte) 
            self.packet_header.append(third_byte)
            self.packet_header.append(fourth_byte) 
        else:
            self.packet_header.append(second_byte) 
            self.packet_header.append(first_byte)
            self.packet_header.append(fourth_byte) 
            self.packet_header.append(third_byte)            
        for z in range(0,20):
            self.packet_header.append(0x0) 
  
    # send a firmware file
    def make_artfirmware_as_msg(self,type_id=self.ARTNET_FW_TYPE_FirmFirst,block_id,fware_sz,dataV,endian_swap=False):
        """Make packet header."""
        # 0 - id (7 x bytes + Null)
        self.packet_header = bytearray()
        self.packet_header.extend(bytearray('Art-Net', 'utf8'))
        self.packet_header.append(0x0)
        # 8 - opcode (2 x 8 low byte first)
        b1, b2 = self.get_op_code(self.ARTNET_FIRMWAREMASTER)         # Arttod request data packet
        self.packet_header.append(b1)
        self.packet_header.append(b2)          
        #self.packet_header.append(0x00)
        #self.packet_header.append(0x50)  
        # 10 - prototocol version (2 x 8 high byte first)
        self.packet_header.append(0x0)
        self.packet_header.append(14)
        for z in range(0,2):
            self.packet_header.append(0x0) 
        self.packet_header.append(type_id)
        self.packet_header.append(block_id)
        first_byte = hex(fware_sz & 0xFF)
        second_byte = hex(((fware_sz & 0xFF00)>>8))
        third_byte = hex(((fware_sz & 0xFF0000)>>16))
        fourth_byte = hex(((fware_sz & 0xFF000000)>>24))
        if not endian_swap:
            self.packet_header.append(first_byte)
            self.packet_header.append(second_byte) 
            self.packet_header.append(third_byte)
            self.packet_header.append(fourth_byte) 
        else:
            self.packet_header.append(second_byte) 
            self.packet_header.append(first_byte)
            self.packet_header.append(fourth_byte) 
            self.packet_header.append(third_byte)            
        for z in range(0,20):
            self.packet_header.append(0x0) 
        # This array contains the firmware or UBEA data block. The order is hi byte first. 
        # The interpretation of this data is manufacturer specific. Final packet should be 
        # null packed if less than 512 bytes needed.            
        self.packet_header.extend(dataV) 
        
    # ArtTimeCode allows time code to be transported over the network. 
    # The data format is compatible with both longitudinal time code and MIDI time code. 
    # The four key types of Film, EBU, Drop Frame and SMPTE are also encoded.

    # define the possible media types
    ARTNET_TC_TYPE_Film = 0x00                                               # Film 24fps
    ARTNET_TC_TYPE_EBU = 0x01                                                # EBU  25fps
    ARTNET_TC_TYPE_DF = 0x02                                                 # DF 29.7fps
    ARTNET_TC_TYPE_SMPTE = 0x03                                              # SMPYE 30fps
    def make_timecode_header(self,frame_time=29,s,m,h,type_media=self.ARTNET_TC_TYPE_SMPTE):
        """Make packet header."""
        # 0 - id (7 x bytes + Null)
        self.packet_header = bytearray()
        self.packet_header.extend(bytearray('Art-Net', 'utf8'))
        self.packet_header.append(0x0)
        # 8 - opcode (2 x 8 low byte first)
        b1, b2 = self.get_op_code(self.ARTNET_OpTimeCode)         # ArtDmx timecode packet
        self.packet_header.append(b1)
        self.packet_header.append(b2)            
        # 10 - filler
        self.packet_header.append(0x0)
        self.packet_header.append(0x0)
        # frames time
        frame_time = frame_time % 29                          # 0-29 range
        self.packet_header.append(frame_time)
        # seconds minutes hours
        self.packet_header.append(s)
        self.packet_header.append(m)
        self.packet_header.append(h)
        self.packet_header.append(type_media)

    # The ArtCommand packet is used to send property set style commands. 
    # The packet can be unicast or broadcast, the decision being application specific.
    ARTNET_ADDR_CMD_AcCancelMerge =0x01                              # If Node is currently in merge mode, cancel merge mode upon receipt of next ArtDmx packet. See discussion of merge mode operation
    ARTNET_ADDR_CMD_AcLedNormal =0x02                                # The front panel indicators of the Node operate normally.
    ARTNET_ADDR_CMD_AcLedMute =0x03                                  # The front panel indicators of the Node are disabled and switched off.
    ARTNET_ADDR_CMD_AcLedLocate =0x04                                # Rapid flashing of the Node’s front panel indicators. It is intended as an outlet identifier for large installations.
    ARTNET_ADDR_CMD_AcResetRxFlags =0x05                             # Resets the Node’s Sip, Text, Test and data error flags. If an output short is being flagged, forces the test to re-run.
    ARTNET_ADDR_CMD_AcMergeLtp0 =0x10                                # Set DMX Port 0 to Merge in LTP mode
    ARTNET_ADDR_CMD_AcMergeLtp1 =0x11                                # Set DMX Port 1 to Merge in LTP mode
    ARTNET_ADDR_CMD_AcMergeLtp2 =0x12                                # Set DMX Port 2 to Merge in LTP mode
    ARTNET_ADDR_CMD_AcMergeLtp3 =0x13                                # Set DMX Port 3 to Merge in LTP mode
    ARTNET_ADDR_CMD_AcMergeHtp0 =0x50                                # Set DMX Port 0 to Merge in HTP (default) mode
    ARTNET_ADDR_CMD_AcMergeHtp1 =0x51                                # Set DMX Port 1 to Merge in HTP (default) mode
    ARTNET_ADDR_CMD_AcMergeHtp2 =0x52                                # Set DMX Port 2 to Merge in HTP (default) mode
    ARTNET_ADDR_CMD_AcMergeHtp3 =0x53                                # Set DMX Port 3 to Merge in HTP (default) mode
    ARTNET_ADDR_CMD_AcArtNetSel0 =0x60                               # Set DMX Port 0 to output both DMX512 and RDM packets from the Art-Net protocol (default)
    ARTNET_ADDR_CMD_AcArtNetSel1 =0x61                               # Set DMX Port 1 to output both DMX512 and RDM packets from the Art-Net protocol (default)
    ARTNET_ADDR_CMD_AcArtNetSel2 =0x62                               # Set DMX Port 2 to output both DMX512 and RDM packets from the Art-Net protocol (default)
    ARTNET_ADDR_CMD_AcArtNetSel3 =0x63                               # Set DMX Port 3 to output both DMX512 and RDM packets from the Art-Net protocol (default)
    ARTNET_ADDR_CMD_AcAcnSel0 =0x70                                  # Set DMX Port 0 to output DMX512 data from the sACN protocol and RDM data from the Art-Net protocol.
    ARTNET_ADDR_CMD_AcAcnSel1 =0x71                                  # Set DMX Port 1 to output DMX512 data from the sACN protocol and RDM data from the Art-Net protocol.
    ARTNET_ADDR_CMD_AcAcnSel2 =0x72                                  # Set DMX Port 2 to output DMX512 data from the sACN protocol and RDM data from the Art-Net protocol.
    ARTNET_ADDR_CMD_AcAcnSel3 =0x73                                  # Set DMX Port 3 to output DMX512 data from the sACN protocol and RDM data from the Art-Net protocol.
    ARTNET_ADDR_CMD_AcClearOp0 =0x90                                 # Clear DMX Output buffer for Port 0
    ARTNET_ADDR_CMD_AcClearOp1 =0x91                                 # Clear DMX Output buffer for Port 1
    ARTNET_ADDR_CMD_AcClearOp2 =0x92                                 # Clear DMX Output buffer for Port 2
    ARTNET_ADDR_CMD_AcClearOp3 =0x93                                 # Clear DMX Output buffer for Port 3
    def make_artcmd_header(self, esta_code):
        """Make packet header."""
        # 0 - id (7 x bytes + Null)
        self.packet_header = bytearray()
        self.packet_header.extend(bytearray('Art-Net', 'utf8'))
        self.packet_header.append(0x0)
        # 8 - opcode (2 x 8 low byte first)
        b1, b2 = self.get_op_code(self.ARTNET_OpCommand)         # ArtCmd data packet
        self.packet_header.append(b1)
        self.packet_header.append(b2)          
        #self.packet_header.append(0x00)
        #self.packet_header.append(0x50)  
        # 10 - prototocol version (2 x 8 high byte first)
        self.packet_header.append(0x0)
        self.packet_header.append(14)
        # Esta manufacturer code // The ESTA manufacturer code. These codes are used to represent equipment manufacturer. 
        # They are assigned by ESTA. This field can be interpreted as two ASCII bytes representing the manufacturer initials.
        first_byte, second_byte = self.shift_this(esta_code)
        self.packet_header.append(first_byte)
        self.packet_header.append(second_byte)        
        # 16 - packet size (2 x 8 high byte first)
        msb, lsb = self.shift_this(self.packet_size)		# convert to MSB / LSB
        self.packet_header.append(msb)
        self.packet_header.append(lsb)

    # The ArtCommand packet is used to send property set style commands.
    # The packet can be unicast or broadcast, the decision being application specific.
    ARTNET_TRIGGER_KeyAscii = 0x00              # The SubKey field contains an ASCII character which the receiving device should process as if it were a keyboard press. (Payload not used)
    ARTNET_TRIGGER_KeyMacro = 0x01              # The SubKey field contains the number of a Macro which the receiving device should execute. (Payload not used).
    ARTNET_TRIGGER_KeySoft = 0x02               # The SubKey field contains a soft-key number which the receiving device should process as if it were a soft-key keyboard press. (Payload not used).
    ARTNET_TRIGGER_KeyShow = 0x03
    def make_arttrigger_header(self, oem_code, key, sub_key=self.ARTNET_TRIGGER_KeyMacro):
        """Make packet header."""
        # 0 - id (7 x bytes + Null)
        self.packet_header = bytearray()
        self.packet_header.extend(bytearray('Art-Net', 'utf8'))
        self.packet_header.append(0x0)
        # 8 - opcode (2 x 8 low byte first)
        b1, b2 = self.get_op_code(self.ARTNET_OpTrigger)         # ArtTrigger data packet
        self.packet_header.append(b1)
        self.packet_header.append(b2)          
        #self.packet_header.append(0x00)
        #self.packet_header.append(0x50)  
        # 10 - prototocol version (2 x 8 high byte first)
        self.packet_header.append(0x0)
        self.packet_header.append(14)
        # filler
        self.packet_header.append(0x00)
        self.packet_header.append(0x00)  
        # oem code
        first_byte, second_byte = self.shift_this(oem_code)
        self.packet_header.append(first_byte)
        self.packet_header.append(second_byte)          
        # key and sub key
        self.packet_header.append(key)
        self.packet_header.append(sub_key) 

    # The ArtCommand packet is used to send property set style commands.
    # The packet can be unicast or broadcast, the decision being application specific.
    def make_artnzs_header(self, start_code, sub_uni, netw):
        """Make packet header."""
        # 0 - id (7 x bytes + Null)
        self.packet_header = bytearray()
        self.packet_header.extend(bytearray('Art-Net', 'utf8'))
        self.packet_header.append(0x0)
        # 8 - opcode (2 x 8 low byte first)
        b1, b2 = self.get_op_code(self.ARTNET_OpNzs)         # ArtNzs data packet
        self.packet_header.append(b1)
        self.packet_header.append(b2)          
        #self.packet_header.append(0x00)
        #self.packet_header.append(0x50)  
        # 10 - prototocol version (2 x 8 high byte first)
        self.packet_header.append(0x0)
        self.packet_header.append(14)
        # 12 - sequence (int 8), NULL for not implemented
        # The sequence number is used to ensure that ArtDmx packets are used in the correct order. 
        # When Art-Net is carried over a medium such as the Internet, it is possible that ArtDmx packets 
        # will reach the receiver out of order. This field is incremented in the range 0x01 to 0xff to allow the receiving node 
        # to resequence packets. The Sequence field is set to 0x00 to disable this feature
        self.packet_header.append(self.sequence)
        # 13 - start code
        self.packet_header.append(start_code)
        # 14 - universe, (2 x 8 low byte first)
        self.packet_header.append(sub_uni)
        self.packet_header.append(netw)
        # 16 - packet size (2 x 8 high byte first)
        msb, lsb = self.shift_this(self.packet_size)		# convert to MSB / LSB
        self.packet_header.append(msb)
        self.packet_header.append(lsb) 
 
    def make_artvlc_header(self, start_code, sub_uni, netw):
        """Make packet header."""
        # 0 - id (7 x bytes + Null)
        self.packet_header = bytearray()
        self.packet_header.extend(bytearray('Art-Net', 'utf8'))
        self.packet_header.append(0x0)
        # 8 - opcode (2 x 8 low byte first)
        b1, b2 = self.get_op_code(self.ARTNET_OpNzs)         # ArtVlc data packet
        self.packet_header.append(b1)
        self.packet_header.append(b2)          
        #self.packet_header.append(0x00)
        #self.packet_header.append(0x50)  
        # 10 - prototocol version (2 x 8 high byte first)
        self.packet_header.append(0x0)
        self.packet_header.append(14)
        # 12 - sequence (int 8), NULL for not implemented
        # The sequence number is used to ensure that ArtDmx packets are used in the correct order. 
        # When Art-Net is carried over a medium such as the Internet, it is possible that ArtDmx packets 
        # will reach the receiver out of order. This field is incremented in the range 0x01 to 0xff to allow the receiving node 
        # to resequence packets. The Sequence field is set to 0x00 to disable this feature
        self.packet_header.append(self.sequence)
        # 13 - start code
        self.packet_header.append(start_code)
        # 14 - universe, (2 x 8 low byte first)
        self.packet_header.append(sub_uni)
        self.packet_header.append(netw)
        # 16 - packet size (2 x 8 high byte first)
        msb, lsb = self.shift_this(self.packet_size)		# convert to MSB / LSB
        self.packet_header.append(msb)
        self.packet_header.append(lsb) 

    # ArtVlc is a specific implementation of the ArtNzs packet which is used for the 
    # transfer of VLC (Visible Light Communication) data over Art-Net. 
    #( The packet’s payload can also be used to transfer VLC over a DMX512 physical layer).
    ARTNET_VLC_FLAG_BEACON = (1<<5)                                         # If set, the transmitter should continuously repeat transmission of this packet until another is received. If clear, the transmitter should transmit this packet once.
    ARTNET_VLC_FLAG_REPLY = (1<<6)                                          # If set this is a reply packet that is in response to the request sent with matching number in the transaction number: TransHi/Lo. If clear this is not a reply.
    ARTNET_VLC_FLAG_IEEE = (1<<7)                                           # If set, data in the payload area shall be interpreted as IEEE VLC data. If clear, PayLanguage defines the payload contents
    ARTNET_VLC_LANG_URL = 0x0                                               # BeaconURL – Payload contains a simple text string representing a URL.
    ARTNET_VLC_LANG_TXT = 0x1                                               # BeaconText – Payload contains a simple ASCII text message    
    
    # transaction id
    # The transaction number is a 16-bit value which allows VLC transactions to be synchronised. A value of 0 indicates the first packet in a transaction. 
    # A value of ffff16 indicates the final packet in the transaction. All other packets contain consecutive numbers which increment on each packet and roll over to 1 at fffe16
    #
    # slot
    # The slot number, range 1-512, of the device to which this packet is directed. A value 0f 0 indicates that all devices attached to this packet’s Port-Address should accept the packet
    #
    # paycount
    # The 16-bit payload size in the range 0 to 48010
    #
    # paychecksum
    # The 16-bit unsigned additive checksum of the data in the payload  
    #
    # VlcDepth
    # The 8-bit VLC modulation depth expressed as a percentage in the range 1 to 100. A value of 0 indicates that the transmitter should use its default value
    #
    # VlcFreq
    # The 16-bit modulation frequency of the VLC transmitter expressed in Hz. A value of 0 indicates that the transmitter should use its default value
    #
    # VlcMod
    # VlcModHi Int8 - The 16-bit modulation type number that the transmitter should use to transmit VLC. 000016 – Use transmitter default
    #
    # PayLang
    # Int8 - The 16-bit payload language code. Currently registered values:
    #
    # BeacRep
    # The 16-bit beacon mode repeat frequency. If Flags.Beacon is set, this 16-bit value indicates the frequency in 
    # Hertz at which the VLC packet should be repeated.  000016 – Use transmitter default.
    #
    # dataV
    # this is the data message you are attaching to this packet
    #
    def make_artvlc_packet(self,flags=self.ARTNET_VLC_LANG_TXT,trans_id,slot,paycount,paychecksum,VlcDepth,VlcFreq,VlcMod,PayLang,BeacRep,dataV):     
        self.vlc_packet = bytearray()
        self.vlc_packet.append(0x41)                        # magic number
        self.vlc_packet.append(0x4C)                        # magic number
        self.vlc_packet.append(0x45)                        # magic number
        self.vlc_packet.append(flags)                       # flags defining msg type   
        first_byte, second_byte = self.shift_this(trans_id)        
        self.vlc_packet.append(first_byte)
        self.vlc_packet.append(second_byte)  
        first_byte, second_byte = self.shift_this(slot)          
        self.vlc_packet.append(first_byte)
        self.vlc_packet.append(second_byte)       
        first_byte, second_byte = self.shift_this(paycount)          
        self.vlc_packet.append(first_byte)
        self.vlc_packet.append(second_byte)  
        first_byte, second_byte = self.shift_this(paychecksum)          
        self.vlc_packet.append(first_byte)
        self.vlc_packet.append(second_byte)  
        self.vlc_packet.append(0x00)
        self.vlc_packet.append(VlcDepth)        
        first_byte, second_byte = self.shift_this(VlcFreq)          
        self.vlc_packet.append(first_byte)
        self.vlc_packet.append(second_byte)   
        first_byte, second_byte = self.shift_this(VlcMod)        
        self.vlc_packet.append(first_byte)
        self.vlc_packet.append(second_byte) 
        first_byte, second_byte = self.shift_this(PayLang)          
        self.vlc_packet.append(first_byte)
        self.vlc_packet.append(second_byte) 
        first_byte, second_byte = self.shift_this(BeacRep)         
        self.vlc_packet.append(first_byte)
        self.vlc_packet.append(second_byte) 
        self.vlc_packet.extend(dataV)                        # append the data message 

    # A Controller or monitoring device on the network can enable or disable individual
    # DMX512 inputs on any of the network nodes. This allows the Controller to directly control 
    # network traffic and ensures that unused inputs are disabled and therefore not wasting bandwidth.
    # length of data message is ARTNET_MAX_PORTS = 4
    def make_artinput_header(self,bind,ports):
        """Make ArtSync header"""
        self.artsync_header = bytearray()  # Initialize as empty bytearray
        # ID: Array of 8 characters, the final character is a null termination.
        self.artsync_header.extend(bytearray('Art-Net', 'utf8'))
        self.artsync_header.append(0x0)
        # OpCode: Defines the class of data within this UDP packet. Transmitted low byte first.
        b1, b2 = self.get_op_code(self.ARTNET_INPUT)         # ArtDmx input disable/enable packet
        self.packet_header.append(b1)
        self.packet_header.append(b2)          
        # space filler
        self.artsync_header.append(0x0)
        # bind index
        # The BindIndex defines the bound node which originated this packet and is 
        # used to uniquely identify the bound node when identical IP addresses are in use. 
        # This number represents the order of bound devices. A lower number means closer to root device. 
        # A value of 1 means root device
        self.artsync_header.append(bind)
        # ports number of input or output ports
        first_byte, second_byte = self.shift_this(ports)   
        self.artsync_header.append(first_byte)
        self.artsync_header.append(second_byte)

    def make_artsync_header(self):
        """Make ArtSync header"""
        self.artsync_header = bytearray()  # Initialize as empty bytearray
        # ID: Array of 8 characters, the final character is a null termination.
        self.artsync_header.extend(bytearray('Art-Net', 'utf8'))
        self.artsync_header.append(0x0)
        # OpCode: Defines the class of data within this UDP packet. Transmitted low byte first.
        b1, b2 = self.get_op_code(self.ARTNET_OpSync)         # ArtDmx op_sync packet
        self.packet_header.append(b1)
        self.packet_header.append(b2)          
        #self.artsync_header.append(0x00)
        #self.artsync_header.append(0x52)
        # ProtVerHi and ProtVerLo: Art-Net protocol revision number. Current value =14.
        # Controllers should ignore communication with nodes using a protocol version lower than =14.
        self.artsync_header.append(0x0)
        self.artsync_header.append(14)
        # Aux1 and Aux2: Should be transmitted as zero.
        self.artsync_header.append(0x0)
        self.artsync_header.append(0x0)
        
    def send_artsync(self):
        """Send Artsync"""
        self.make_artsync_header()
        try:
            self.socket_client.sendto(self.artsync_header, (self.target_ip, self.UDP_PORT))
        except socket.error as error:
            print(f"ERROR: Socket error with exception: {error}")

    # send header and buffer
    def show(self):
        """Finally send data."""
        packet = bytearray()
        packet.extend(self.packet_header)
        packet.extend(self.buffer)
        try:
            self.socket_client.sendto(packet, (self.target_ip, self.UDP_PORT))
            if self.if_sync:  # if we want to send artsync
                self.send_artsync()
        except socket.error as error:
            print(f"ERROR: Socket error with exception: {error}")
        finally:
            self.sequence = (self.sequence + 1) % 256

    def send_rdm(self):
        """Finally send data."""
        packet = bytearray()
        packet.extend(self.packet_header)
        packet.extend(self.rdm_packet)
        try:
            self.socket_client.sendto(packet, (self.target_ip, self.UDP_PORT))
            if self.if_sync:  # if we want to send artsync
                self.send_artsync()
        except socket.error as error:
            print(f"ERROR: Socket error with exception: {error}")
        finally:
            self.sequence = (self.sequence + 1) % 256
            
    # send header
    def send_header_msg(self):
        """Finally send data."""
        packet = bytearray()
        packet.extend(self.packet_header)
        try:
            self.socket_client.sendto(packet, (self.target_ip, self.UDP_PORT))
            if self.if_sync:  # if we want to send artsync
                self.send_artsync()
        except socket.error as error:
            print(f"ERROR: Socket error with exception: {error}")
        finally:
            self.sequence = (self.sequence + 1) % 256

    # send header and data
    def send_data_msg(self,dataV):
        """Finally send data."""
        packet = bytearray()
        packet.extend(self.packet_header)
        packet.extend(dataV)
        try:
            self.socket_client.sendto(packet, (self.target_ip, self.UDP_PORT))
            if self.if_sync:  # if we want to send artsync
                self.send_artsync()
        except socket.error as error:
            print(f"ERROR: Socket error with exception: {error}")
        finally:
            self.sequence = (self.sequence + 1) % 256

    # send heasder and data and wait for reply and return its data
    def sendrcv_data_msg(self, dataV):
        """Finally send data."""
        packet = bytearray()
        packet.extend(self.packet_header)
        packet.extend(dataV)
        try:
            self.socket_client.settimeout(10)
            self.socket_client.sendto(packet, (self.target_ip, self.UDP_PORT))
            if self.if_sync:  # if we want to send artsync
                self.send_artsync()
            dataR = self.socket_client.recv(1024)
        except socket.error as error:
            print(f"ERROR: Socket error with exception: {error}")
        finally:
            self.sequence = (self.sequence + 1) % 256
        return dataR
        
    def send_vlc_packet(self):
        """Finally send data."""
        packet = bytearray()
        packet.extend(self.packet_header)
        packet.extend(self.vlc_packet)
        try:
            self.socket_client.sendto(packet, (self.target_ip, self.UDP_PORT))
            if self.if_sync:  # if we want to send artsync
                self.send_artsync()
        except socket.error as error:
            print(f"ERROR: Socket error with exception: {error}")
        finally:
            self.sequence = (self.sequence + 1) % 256
            
    def close(self):
        """Close UDP socket."""
        self.socket_client.close()

    # THREADING #

    def start(self):
        """Starts thread clock."""
        self.show()
        if not hasattr(self, "running"):
            self.running = True
        elif self.running:
            sleep((1000.0 / self.fps) / 1000.0)
            _thread.start_new_thread(self.start, ())

    def stop(self):
        """Set flag so thread will exit."""
        self.running = False

    # SETTERS - HEADER #

    def set_universe(self, universe):
        """Setter for universe (0 - 15 / 256).

        Mind if protocol has been simplified
        """
        # This is ugly, trying to keep interface easy
        # With simplified mode the universe will be split into two
        # values, (uni and sub) which is correct anyway. Net will always be 0
        if self.is_simplified:
            self.universe = self.put_in_range(universe, 0, 255, False)
        else:
            self.universe = self.put_in_range(universe, 0, 15, False)
        self.make_artdmx_header()

    def set_subnet(self, sub):
        """Setter for subnet address (0 - 15).

        Set simplify to false to use
        """
        self.subnet = self.put_in_range(sub, 0, 15, False)
        self.make_artdmx_header()

    def set_net(self, net):
        """Setter for net address (0 - 127).

        Set simplify to false to use
        """
        self.net = self.put_in_range(net, 0, 127, False)
        self.make_artdmx_header()

    def set_packet_size(self, packet_size):
        """Setter for packet size (2 - 512, even only)."""
        self.packet_size = self.put_in_range(packet_size, 2, 512, self.make_even)
        self.make_artdmx_header()

    def check_rdm_adcount_range(self, rdm_adcount):
        """check bounds of rd adcount"""
        ranged = self.put_in_range(rdm_adcount, 1, 32, False)
        return ranged

    def check_uid_range(self, uid):
        """check bounds of uid"""
        ranged = self.put_in_range(uid, 0, 200, False)
        return ranged
        
    # SETTERS - DATA #

    def clear(self):
        """Clear DMX buffer."""
        self.buffer = bytearray(self.packet_size)

    def set(self, value):
        """Set buffer."""
        if len(value) != self.packet_size:
            print("ERROR: packet does not match declared packet size")
            return
        self.buffer = value

    def set_16bit(self, address, value, high_first=False):
        """Set single 16bit value in DMX buffer."""
        if address > self.packet_size:
            print("ERROR: Address given greater than defined packet size")
            return
        if address < 1 or address > 512 - 1:
            print("ERROR: Address out of range")
            return
        value = self.put_in_range(value, 0, 65535, False)

        # Check for endianess
        if high_first:
            self.buffer[address - 1] = (value >> 8) & 0xFF  # high
            self.buffer[address] = (value) & 0xFF 			# low
        else:
            self.buffer[address - 1] = (value) & 0xFF				# low
            self.buffer[address] = (value >> 8) & 0xFF  # high

    def set_single_value(self, address, value):
        """Set single value in DMX buffer."""
        if address > self.packet_size:
            print("ERROR: Address given greater than defined packet size")
            return
        if address < 1 or address > 512:
            print("ERROR: Address out of range")
            return
        self.buffer[address - 1] = self.put_in_range(value, 0, 255, False)

    def set_single_rem(self, address, value):
        """Set single value while blacking out others."""
        if address > self.packet_size:
            print("ERROR: Address given greater than defined packet size")
            return
        if address < 1 or address > 512:
            print("ERROR: Address out of range")
            return
        self.clear()
        self.buffer[address - 1] = self.put_in_range(value, 0, 255, False)

    def set_rgb(self, address, red, green, blue):
        """Set RGB from start address."""
        if address > self.packet_size:
            print("ERROR: Address given greater than defined packet size")
            return
        if address < 1 or address > 510:
            print("ERROR: Address out of range")
            return

        self.buffer[address - 1] = self.put_in_range(red, 0, 255, False)
        self.buffer[address] = self.put_in_range(green, 0, 255, False)
        self.buffer[address + 1] = self.put_in_range(blue, 0, 255, False)

    # AUX Function #

    def send(self, packet):
        """Set buffer and send straightaway.

        Args:
        array - integer array to send
        """
        self.set(packet)
        self.show()

    def set_simplified(self, simplify):
        """Builds Header accordingly.

        True - Header sends 16 bit universe value (OK but incorrect)
        False - Headers sends Universe - Net and Subnet values as protocol
        It is recommended that you set these values with .set_net() and set_physical
        """
        # avoid remaking header if there are no changes
        if simplify == self.is_simplified:
            return
        self.is_simplified = simplify
        self.make_artdmx_header()

    def see_header(self):
        """Show header values."""
        print(self.packet_header)

    def see_buffer(self):
        """Show buffer values."""
        print(self.buffer)

    def blackout(self):
        """Sends 0's all across."""
        self.clear()
        self.show()

    def flash_all(self, delay=None):
        """Sends 255's all across."""
        self.set([255] * self.packet_size)
        self.show()
        # Blackout after delay
        if delay:
            sleep(delay)
            self.blackout()

if __name__ == '__main__':

    print("\033[34m ========= Starting ArtNet4 Communication ==========")
    print("Namespace run")
    TARGET_IP = '10.2.7.6'          # typically in 2.x or 10.x range
    UNIVERSE_TO_SEND = 15           # see docs
    PACKET_SIZE = 20                # it is not necessary to send whole universe

    a = Artnet_4(TARGET_IP, UNIVERSE_TO_SEND, PACKET_SIZE, artsync=True)
    a.init_packet_sizes(512)        # initialise the packet sizes for the packet buffers DMX=512
    a.set_simplified(False)         # set the header message for DMX over artNet4
    a.set_net(129)                  # set net
    a.set_subnet(16)                # set sub net

    # Look at the object state
    print(a)

    # set-up the DMX512 data segment(s)
    
    # DMX Channel-Attribute Allocations mapped direct from DMX data 1->1
    # example for ADB Combo Warp    
    ADB_COWARP_GelPos = 1                                                    # Gel position  
    ADB_COWARP_GelSpeed = 2                                                  # Gel Speed     
    ADB_COWARP_Fan = 3                                                       # Fan Speed   
    ADB_COWARP_Shut8 = 4                                                     # Shutter 8 bit  */
    ADB_COWARP_Shut16 = 5                                                    # Shutter 16 bit  */
    a.set_single_value(ADB_COWARP_GelPos, 255)                               # set the buffer with the dmx requests
    a.set_single_value(ADB_COWARP_GelSpeed, 100)
    a.set_single_value(ADB_COWARP_Fan, 200)
    a.set_single_value(ADB_COWARP_Shut16, 1000)

    # rgb light is mapped onto channel 6
    red_qty = 0
    green_qty = 140
    blue_qty = 160
    s.set_rgb(6, red_qty, green_qty, blue_qty)
    
    # now send that data to the ArtNet4 server 
    #    
    print("========= Sending values to ArtNet gw ============")
    a.show()
    a.see_buffer()
    a.flash_all()
    a.see_buffer()
    a.show()

    print("\033[35m --------- Values sent -----------------------------\033[0m")

    # Cleanup when you are done
    del a

