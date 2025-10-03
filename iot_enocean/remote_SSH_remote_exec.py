#!/usr/bin/env python
# remote ssh to remote computer ro execute command
#
import paramiko

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='parser')
    parser.add_argument('ip_addr', type=str, help='ip address of websocket server')
    parser.add_argument('user', type=str, help='user name')
    parser.add_argument('passw', type=str, help='password')
    args = parser.parse_args()	
    ip_address = args.ip_addr
    u = args.user
    p = args.passw
    YOUR_CMD="/opt/FandP/bin/tools/GetVal TAG.IN"
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(ip_address, username="USER", password="PASSWORD", allow_agent = False)
    i, o, e = ssh.exec_command(YOUR_CMD)                                                        # example on F&P System 6 ro set block input
    s = e.read()
    if s: # an error occurred
        raise RuntimeError, s
    result = o.read()
    print(result)