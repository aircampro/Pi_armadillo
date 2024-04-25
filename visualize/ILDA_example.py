# ====================================================================================================================================
# The ILDA image data transfer format (file extension .ild) was designed by the International Laser Display Association to facilitate
# the transfer of laser show content between sites and systems. It supports the storage of not just individual frames/objects but time 
# varying sequences (animations). It is a binary format employing the so-called "big endian" byte ordering
#
# A ILDA file can describe three different types of data: 2D coordinates, 3D coordinates, and index colour palettes. 
# Each data type is represented in a ILDA file by a header followed by the data. These types (sections) can appear in any order 
# except that colour palette data is supposed to precede the 2D or 3D data it applies to.
#

class ILDA():
    """implementation of ILDA laser file creation """

    # file types 
    ILDA_FORMAT_3D =0                                                       # type for the file or data entry section data is as per structs below
    ILDA_FORMAT_2D =1
    ILDA_COLOR_PALETTE =2
      
    def __init__(self):
        """ILDA

        class for creating ILDA files or packets for transfer to a laser 

        """
        # Instance variables
        self.packet_header = bytearray()
        self.buffer = bytearray()
        self.file_buffer = bytearray()
        
    def __exit__(self):
        """Graceful shutdown."""
        print("--- end ILDA class ---")
        
    def __del__(self):
        """Graceful shutdown."""
        print("--- deleted ILDA class ---")

    def __str__(self):
        """Printable object state."""
        print("--- started ILDA class ---")
        
    def clamp(self, number, min_val, max_val):
        """Utility method: sets number in defined range.

        Args:
        number - number to use
        min_val - lowest possible number
        max_val - highest possible number

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
        """Utility method: sets number in defined range.

        Args:
        endian_swap_byte - swaps endianness

        Returns:
        2 bytes in endian order

        """
        first_byte = hex(num_in & 0xFF)                    # LSB
        second_byte = hex(((num_in & 0xFF00)>>8))          # MSB
        if not endian_swap_byte:
            return first_byte, second_byte
        else:
            return second_byte, first_byte
 
    # makes the header sections
    #
    def make_ilda_header(self, format_typ=self.ILDA_COLOR_PALETTE, name, co_name, num_of_vals, cur_frame, total_frame, scanner_head=0):
        """Make ILDA file header."""
        self.packet_header = bytearray()
        self.packet_header.extend(bytearray('ILDA', 'utf8'))
        for rep in range(0,4):
            self.packet_header.append(0x0)
        self.packet_header.append(format_typ)
        self.packet_header.extend(name)
        self.packet_header.extend(co_name)
        # For coordinates this is the number of points in the following data section, 
        # for colour palettes it is the number of entries in the palette. 
        # Using this along with the known size for the data section entries allows a parsing program to skip 
        # over sections they weren't interested.
        first_byte, second_byte = self.shift_this(num_of_vals)
        self.packet_header.append(first_byte)
        self.packet_header.append(second_byte)
        # For files that contain a number of frames, eg: library of graphical shapes, collection of colour palettes, 
        # or an animation sequence, this is the current number. It ranges from 0 up to one the total number of frames minus 1
        first_byte, second_byte = self.shift_this(cur_frame)
        self.packet_header.append(first_byte)
        self.packet_header.append(second_byte)
        # The total number of frames and is not used for colour palette format types. 
        # This is set to 0 in a "null header" to indicate the end of the ILDA file
        first_byte, second_byte = self.shift_this(total_frame)
        self.packet_header.append(first_byte)
        self.packet_header.append(second_byte) 
        # scanner_head :: is used for systems with multiple scanners or heads, otherwise set to 0 for the default device 
        self.packet_header.append(scanner_head)
        self.packet_header.append(0x0) 

    # defines a color palette object        
    def make_colorIdx_object(self, r, g, b):
        """Make color mix object"""
        r_ranged = self.put_in_range(r, 0, 255, False)
        g_ranged = self.put_in_range(g, 0, 255, False)
        b_ranged = self.put_in_range(b, 0, 255, False)
        return [ r_ranged, g_ranged, b_ranged ]

    # makes color buffer
    def make_colorIdx_buffer(self, color_idx_obj):
        """Make color buffer"""
        self.buffer = bytearray()
        self.buffer.extend(color_idx_obj)

    # adds color to the buffer
    def add_colorIdx_buffer(self, color_idx_obj):
        """add color to buffer"""
        self.buffer.extend(color_idx_obj)

    # makes 3d objects
    def make_3D_data_buffer(self, x, y, z, laser_state, color_idx_obj):
        """Make 3d buffer"""
        self.buffer = bytearray()
        first_byte, second_byte = self.shift_this(x)
        self.buffer.append(first_byte)
        self.buffer.append(second_byte)
        first_byte, second_byte = self.shift_this(y)
        self.buffer.append(first_byte)
        self.buffer.append(second_byte)
        first_byte, second_byte = self.shift_this(z)
        self.buffer.append(first_byte)
        self.buffer.append(second_byte)
        self.buffer.append(laser_state)
        self.buffer.extend(color_idx_obj)

    # adds 3d objects
    def add_3D_data_buffer(self, x, y, z, laser_state, color_idx_obj):
        """Add to 3d buffer"""
        first_byte, second_byte = self.shift_this(x)
        self.buffer.append(first_byte)
        self.buffer.append(second_byte)
        first_byte, second_byte = self.shift_this(y)
        self.buffer.append(first_byte)
        self.buffer.append(second_byte)
        first_byte, second_byte = self.shift_this(z)
        self.buffer.append(first_byte)
        self.buffer.append(second_byte)
        self.buffer.append(laser_state)
        self.buffer.extend(color_idx_obj)

    def make_2D_data_buffer(self, x, y, laser_state, color_idx_obj):
        """Make 2d buffer"""
        self.buffer = bytearray()
        first_byte, second_byte = self.shift_this(x)
        self.buffer.append(first_byte)
        self.buffer.append(second_byte)
        first_byte, second_byte = self.shift_this(y)
        self.buffer.append(first_byte)
        self.buffer.append(second_byte)
        self.buffer.append(laser_state)
        self.buffer.extend(color_idx_obj)

    def add_2D_data_buffer(self, x, y, laser_state, color_idx_obj):
        """Add to 2d buffer"""
        first_byte, second_byte = self.shift_this(x)
        self.buffer.append(first_byte)
        self.buffer.append(second_byte)
        first_byte, second_byte = self.shift_this(y)
        self.buffer.append(first_byte)
        self.buffer.append(second_byte)
        self.buffer.append(laser_state)
        self.buffer.extend(color_idx_obj)        

    # sets the status to last point        
    def set_last_point_status(self, status):
        return (status | (1<<7))

    # sets the status with blanking 
    def set_blanking_status(self, status):
        return (status | (1<<6))        

    # joins header and buffer to new header
    def join_packets(self):
        self.packet_header.extend(self.buffer)    

    # joins array to header
    def add_packet_to_header(self, your_data_array):
        self.packet_header.extend(your_data_array) 

    # clear file buffer
    def init_filebuffer(self):
        self.file_buffer = bytearray()
   
    # add the packet to the file buffer   
    def add_packet_to_filebuffer(self):
        self.file_buffer.extend(self.packet_header)

    # add array data to the filebuffer
    def add_data_to_filebuffer(self, dataV):
        self.file_buffer.extend(dataV)
        
if __name__ == '__main__':

    ilda_file_name = "myfile.ild"    
    if argc >= 1:
        ilda_file_name = str(arg[1])
        
    print("\033[34m ========= Creating an ILDA file with the data programed below ========== \033[0m")
    ilda_obj = ILDA()                                                                # initiate the class
    ilda_obj.make_ilda_header(ilda_obj.ILDA_COLOR_PALETTE,"colorp", "me", 2, 1, 5)   # create a header for colorIdx data
    turq = ilda_obj.make_colorIdx_object(0,200,190)                                  # define the colors in the colorIdx
    mag = ilda_obj.make_colorIdx_object(240,0,220) 
    ilda_obj.make_colorIdx_buffer(turq)                                              # add the color sequence
    ilda_obj.add_colorIdx_buffer(mag)
    ilda_obj.join_packets()                                                          # add the color idx to the header 
    ilda_obj.init_filebuffer()                                                       # init the file buffer
    ilda_obj.add_packet_to_filebuffer()                                              # add the packet to the file buffer
    rd = ilda_obj.make_colorIdx_object(255,60,90)                                    # make some new colors red.green&blue
    gr = ilda_obj.make_colorIdx_object(2,255,20)
    bl = ilda_obj.make_colorIdx_object(50,55,255)
    ilda_obj.make_ilda_header(ilda_obj.ILDA_FORMAT_3D,"3dmap", "me", 3, 3, 5)        # create a 3d point header
    x = 10                                                                           # set co-ordinates and laser state
    y = 5
    z = 14
    laser_state = 1
    ilda_obj.make_3D_data_buffer(x, y, z, laser_state, rd)                           # add 1st point to buffer
    x = 20                                                                           # move
    y = 15
    z = 10  
    ilda_obj.add_3D_data_buffer(x, y, z, laser_state, bl)                            # add 2nd point to buffer 
    x = 30                                                                           # move
    y = 25
    z = 10
    laser_state = ilda_obj.set_last_point_status(laser_state)                        # set last point for laser state                       
    ilda_obj.make_3D_data_buffer(x, y, z, laser_state, gr)                           # add 3rd point to buffer    
    ilda_obj.join_packets()                                                          # add the 3d point map to the header 
    ilda_obj.add_packet_to_filebuffer()                                              # tag the 3d daya to the end of the file buffer
    with open(ilda_file_name, "w" as f:                                              # open a file for writing close once done
        f.write(ilda_obj.file_buffer)                                                # wrute binary content to file   

