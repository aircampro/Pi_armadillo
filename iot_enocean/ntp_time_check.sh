# !/bin/bash
# ntp time checker raspi
#
SYSTEMCTL=/bin/systemctl
STTY=/bin/stty
IPCRM=/usr/bin/ipcrm
LOGGER=/usr/bin/logger
TR=/usr/bin/tr
AWK=/home/pi/opt/bin/awk

NTPQ=/home/pi/opt/bin/ntpq
NTPSHMMON=/home/pi/opt/gpsd-3.18.1/ntpshmmon
PYTHONPATH=/home/pi/opt/lib/python2.7/site-packages:/usr/local/lib/python2.7/site-packages:/usr/lib/pypy/lib-python/2.7/site-packages
export PYTHONPATH

GPSTTYS="/dev/ttyS0 /dev/ttyUSB0 /dev/ttyUSB1"

# debug
trap 'echo "[${BASH_SOURCE:-$0}:$LINENO] - "$BASH_COMMAND" abnomal status"' ERR

MKTEMP=/bin/mktemp
LOCKDIR=/tmp/`basename $0`
LOCKPID=$LOCKDIR/pid

if [ -d $LOCKDIR ]; then
	# LOCKDIRがある
	if [ -f $LOCKPID ]; then
		kill -0 `cat $LOCKPID`
		if [ $? -ne 0 ]; then
			rm -rf $LOCKDIR
		else
			$LOGGER -p local0.notice "$0: still running."
			exit
		fi
	else
		rmdir $LOCKDIR
	fi
fi

# make lock directory
trap 'rm -rf $LOCKDIR' 1 2 3 15
trap 'rm -rf $LOCKDIR' EXIT
mkdir $LOCKDIR
if [ $? -ne 0 ]; then
	$LOGGER -p local0.notice "$0: lockfile make error."
	exit
fi
echo $$ > $LOCKPID

stty_speed () {
	echo -n $GPSTTYS | xargs -d " " -n 1 -i'{}' $STTY -F '{}' 115200
}

# http://doc.ntp.org/4.2.8/drivers/driver28.html
# https://github.com/rnorris/gpsd/blob/master/SConstruct#LC1963
#
clear_shm (){
	echo shemapho clear
	for i in 0x4e545030 0x4e545031 0x4e545032 0x4e545033 0x4e545034 0x4e545035 0x4e545036 0x4e545037 0x47505344; do
		$IPCRM -M $i
	done
}

check_gpsd (){
	RESULT="`$NTPSHMMON -t 3 | $TR '\n' @`"
	if [ `echo $RESULT | $TR @ '\n' | $AWK '/NTP[0-9]/{print $2}' | sort -u | wc -l` -ne 6 ]; then
		echo "$NTPSHMMON error detected"
		echo $RESULT | $TR @ '\n'
		echo $RESULT | $TR @ '\n' | $AWK '/NTP[0-9]/{print $2}' | sort -u
		$SYSTEMCTL stop gpsd
		stty_speed
		/bin/sleep 1
		$SYSTEMCTL start gpsd
		$LOGGER -p local0.notice "$0: gpsd restart to me!"
		echo 'gpsd: Restart done!'
		exit 255
	fi
}

check_ntpd (){
	RESULT="`$NTPQ -p | $TR '\n' @`"
	echo "$RESULT" | $TR @ '\n' | $AWK '($0~/SHM/ && $7==0){exit 255}' || \
	( echo "ntpd error detected" ;\
	  echo "$RESULT" | $TR @ '\n' | $AWK '($0~/SHM/ && $7==0){print}' ;\
	  $SYSTEMCTL restart ntp ;\
	  $LOGGER -p local0.notice "$0: ntpd restart to me!";\
	  exit 255 )
}

check_gpsd && check_ntpd