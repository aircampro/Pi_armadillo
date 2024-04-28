#!/bin/bash
# example shows how to set/read a value in foxboro i/a series
#
_remote="root@machine"
 
echo "Local system name: $HOSTNAME"
echo "Local date and time: $(date)"

# cbp --> compound:block.param = TESTCOMPOUND:PID_BLOCK1.SPT 
cbp=$1
value_set=$2
cbp2=$3

#-l user	Specify the user name to use to log in
#-i file	Specify a public key file. The default setting is ~/.ssh/identity
#-p port	Set the port to connect to 
echo
echo "*** Running commands on remote host named $_remote ***"
echo
ssh -T $_remote <<'EOL'
	now="$(date)"
	name="$HOSTNAME"
	up="$(uptime)"
	echo "Server name is $name"
	echo "Server date and time is $now"
	echo "Server uptime: $up"
	echo "Now setting $cbp to $value_set"
	/opt/fox/bin/tools/setval $cbp $value_set
	echo "Now resding $cbp2"
	/opt/bin/tools/getval $cbp2
	echo "Bye"
EOL


