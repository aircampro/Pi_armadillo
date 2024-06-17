#
# client example to the GE FANUC Karel server controlling the fanuc robot
#
import socket
import time

ip = "127.0.0.1"
port = 65432
# the steps of action
step = 0 
step_delay = 0.3

def main():

  with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    print("Connecting to {}:{} ...".format(ip, port))
    sock.connect((ip, port))
    print('connected and communicating......')
    if (step == 0):
      try:
        while (step == 0):
          try:
            s = bytearray("s0", "utf8")                                  # start the seqwuence between this client and the robot
            sock.sendall(s)
            data = sock.recv(1024)          
            print('Response:', data.decode('ascii'))
            reply = data.decode('ascii')
            try:
              step_read = int(reply.split('s')[1])
              if (step_read == (step+1)):
                step = step_read
            except:
              pass
            time.sleep(step_delay)
          except socket.error:
            pass
      except KeyboardInterrupt:
        sock.close()
    elif (step >= 1):
      try:
        while (step >= 1):
          try:
            s = bytearray(reply, "utf8")
            sock.sendall(s)
            data = sock.recv(1024)          
            print('Response:', data.decode('ascii'))
            reply = data.decode('ascii')
            try:
              step_read = int(reply.split('s')[1])
              if (step_read == (step+1)):
                step = step_read
            except:
              pass
            time.sleep(step_delay)
          except socket.error:
            pass
      except KeyboardInterrupt:
        sock.close() 
        
if __name__ == '__main__':
  main()