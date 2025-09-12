# master sigfox 
#
# ref:- https://docs.pycom.io/tutorials/networks/sigfox/
#
from network import Sigfox
import socket

# send a message to an OMRON CJ PLC over Sigfox
data_omron_cj = [
    # header
    0x80, #(ICF) 
    0x00, #(RSV) 
    0x02, #(GCT) 
    0x00, #(DNA) 
    0x00, #(DA1) 
    0x00, #(DA2) 
    0x00, #(SNA) 
    0x9C, #(SA1)
    0x00, #(SA2) 
    0x22, #(SID) 

    # command
    0x01, #(MRC) 
    0x01, #(SRC)   (0x01,0x01 read、0x01,0x02 write)
    # data
    0x82,                                                               # Read device specification 0x82=D register
    0x00, 0x00, 0x96,                                                   # Specify read area (000096=150)
    0x00,0x07,                                                          # Read Bytes
]
omron_cj_rcv_buf_sz = 1024                                              # receive buffer size

if __name__ == "__main__":

    # set both sigfox rx/tx to same frequency
    sigfox = Sigfox(mode=Sigfox.FSK, frequency=868000000)
    # open socket
    s = socket.socket(socket.AF_SIGFOX, socket.SOCK_RAW)
    s.setblocking(True)
    sendmsg = bytearray(data_omron_cj)

    while True:
        s.send(sendmsg)
        time.sleep(2)
        response = s.recv(omron_cj_rcv_buf_sz)
        response = response.decode('utf-8')
        data1 = [format(i,'02X') for i in response]
        return_data = [0] * 6
        data1 = data1[14:]
        # Convert base 16 to base 10
        # Because the CJ series has a base 10 and a base 16 by register
        # Note conversion in int function
        return_data[0] = int(data1[4] + data1[5], 16)
        return_data[1] = int(data1[7])
        return_data[2] = int(data1[8] + data1[9])
        return_data[3] = int(data1[10] + data1[11])
        return_data[4] = int(data1[13])
        return_data[5] = int(data1[2] + data1[3] + data1[0] + data1[1], 16)
        print(return_data)
    s.close()