# runs example of PPP server for various linux distributions
#
import distro
import subprocess
import time
from argparse import ArgumentParser

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-eth", type=str, default="eth0:1", help="ethernet port")
    parser.add_argument("-opt_file", type=str, default="/etc/ppp/options", help="options file")
    ep = str(args.eth)
    of = str(args.opt_file)
    # type_linux = distro.linux_distribution() returns all 3 but is depricated
    type_linux = distro.id()  
    ver_linux = distro.version()
    name_linux = distro.name()    
    if not type_linux.lower().find("ubuntu") == -1:
        #cmd = ["sudo", "pppoe-server", "-I", ep, "-L", "172.16.0.254", "-R", "172.16.0.101", "-O", "/etc/ppp/options"]
        cmd = ["sudo", "pppoe-server", "-I", ep, "-O", of]
        result = subprocess.run(cmd, capture_output=True, check=True, text=True)
    elif not type_linux.lower().find("debian") == -1:
        cmd = ["sudo", "pppoe", "-I", ep, "-O", of]
        result = subprocess.run(cmd, capture_output=True, check=True, text=True)
    elif not type_linux.lower().find("centos") == -1:
        cmd = ["sudo", "pppoe-start"]
        result = subprocess.run(cmd, capture_output=True, check=True, text=True)
        time.sleep(2)
        cmd = ["sudo", "pppoe-connect", ep]
        result = subprocess.run(cmd, capture_output=True, check=True, text=True)
    elif not type_linux.lower().find("fedora") == -1:
        cmd = ["sudo", "pppoe-server", "-I", ep, "-S"]
        result = subprocess.run(cmd, capture_output=True, check=True, text=True)
    else:
        print(f" {type_linux} {ver_linux} {name_linux} unsupported please add to this program")