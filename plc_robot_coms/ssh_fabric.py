#!/usr/bin/python
#
# remote ssh using fabric
#
from fabric import Connection
result = Connection('web1.example.com').run('uname -s', hide=True)
msg = "Ran {0.command!r} on {0.connection.host}, got stdout:\n{0.stdout}"
print(msg.format(result))