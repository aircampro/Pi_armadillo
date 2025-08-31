#!/usr/bin/python
#
# example of doing a linux pipe in python
#
from subprocess import Popen, PIPE

if __name__ == "__main__":

    cmd1 = ['echo', 'Hello PiPe!']
    cmd2 = ['tr', '[a-z]', '[A-Z]']
    p1 = Popen(cmd1, stdout=PIPE)
    p2 = Popen(cmd2, stdin=p1.stdout, stdout=PIPE)
    stdout_data, stderr_data = p2.communicate()
    print(stdout_data.decode())