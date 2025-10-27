# !/usr/bin/python
# -*- coding: utf-8 -*-
# 
# XMPP Protocol example attached to report data from i2c sensor or turn on/off lights via GPIO alarms locally from setpoints saved in eeprom
#
# pip install jabberbot
# pip install xmpppy
# pip install lazy-reload
# pip install requests
# pip install simplejson
# ref:- https://qiita.com/haminiku/items/14762e5fe20a5efa24b2
#
from __future__ import absolute_import, unicode_literals
# chatbot answering with i2c data
#
import sys
import time
import traceback
import logging
from jabberbot import botcmd, JabberBot, xmpp
import multiprocessing as mp
import random
import logging
import struct
import subprocess
import pigpio
# example of using pigpio rather than smbus for i2c        
# sudo apt install pigpio python-pigpio
# sudo systemctl start pigpiod
# sudo systemctl enable pigpiod
#

# AM2322 read temperature and humidity
#
AL_ON = True

# AM2322 Humidity and Temp sensor on i2c
def get_data(addr = 0x5c):
    pi = pigpio.pi()
    h = pi.i2c_open(1,addr) 
    try:
        pi.i2c_write_quick(h, 0) 
    except:
        pass
    pi.i2c_write_i2c_block_data(h,0x03,[0x00,0x04])
    time.sleep(0.003)
    (count, buf) = pi.i2c_read_device(h,0x08)
    HUM = (buf[2]*256 + buf[3])/10.0
    TEMP = ((buf[4]&0x7f)*256 + buf[5])/10.0 * ( -1 if buf[4]&0x80 > 0 else 1 ) 
    print("temp: {:.1f}, temp: {:.1f}".format(TEMP,HUM))
    pi.i2c_close(h)
    return HUM, TEMP

# As an example, the I2C access example of the EEPROM 24FC1025 is shown.
# default is read:offset 0x0000 3 byte data 3byte 0xAA,0xBB,0xCC
# f = True floating point number is used e = 1 little endian 
#
def eeprom(datav=b'xAA\xBB\xCC', base=b'\x00\x00', mode="w", adddre=0x50, byt=3, f=False, e=1):
    pi = pigpio.pi()
    h = pi.i2c_open(1, adddre)
    if mode == "w":                                        # write value first
        pi.i2c_write_device(h, base+datav)
        time.sleep(0.01)                                   # 10ms wait EEPROM。
    pi.i2c_write_device(h, base)
    val = pi.i2c_read_device(h, byt)
    if f == True:                                          # its a float
        if e == 1:                                         # little endian
            val = struct.unpack_from('<f', val)
        else:                                              # big endian
            val = struct.unpack_from('>f', val)        
    print(f'eeprom # {val}')
    pi.i2c_close(h)
    return val

# drive an output alarm if value exceeds limit set in eeprom
def drive_alarm(testv, opin=18, ba=b'\x00\x00'):
    pi = pigpio.pi()
    pi.set_mode(opin, pigpio.OUTPUT)
    get_sp = eeprom(mode="r",byt=4,f=True, base=ba)
    if testv > get_sp and get_sp < 200 and get_sp > 0:
        pi.write(opin,1)
        print(f'# {pi.read(ipin)=}')
    else:
        pi.write(opin,0)
        print(f'# {pi.read(ipin)=}')

# function to set GPIO output on/off with a feedback input default GPIO17OUT GPIO27IN
def drive_op_with_feedback(mode="on", opin=17, ipin=27):
    pi = pigpio.pi()
    pi.set_mode(opin, pigpio.OUTPUT)
    pi.set_mode(ipin, pigpio.INPUT)
    print(f'# {pi.get_mode(opin)=}/{pi.get_mode(ipin)=}')
    if mode == "on":
        pi.write(opin,1)                                     # GPIO17=High
        print(f'# {pi.read(ipin)=}')
    else:
        pi.write(opin,0)                                     # GPIO17=Low
        print(f'# {pi.read(ipin)=}')
    return pi.read(ipin) == pi.read(opin)

def command(cmd):
    return subprocess.Popen(cmd, stdout=subprocess.PIPE,shell=True).communicate()[0]

def removee(e):
    x = 0
    pvals = -1
    while x != pvals:                     # for some reason i had to iterate this multiple times ? when they are the same they are all gone
        pvals = x
        x = 0 
        while x <len(e):
            try:
                e.remove('--') 
                #print(x)
            except:
                None
            x += 1
        #print("end ",x)
    x = 0
    while x < len(e):
        try:
            e.remove('a')
            e.remove('b')
            e.remove('c')
            e.remove('d')
            e.remove('e')
            e.remove('f')
            e.remove('00:')
            e.remove('10:')
            e.remove('20:')
            e.remove('30:')
            e.remove('40:')
            e.remove('50:')
            e.remove('60:')
            e.remove('70:')
        except:
            None
        x += 1
    int_e = list(map(int,e)) 
    x = 0
    while x < len(int_e): 
        if int_e[x] <= 10:
            int_e.pop(x)
            x -= 1
        x += 1
    return int_e

def probe_i2c():
    try:
        cmd = 'i2cdetect -y 1'
        cmd_result = command(cmd)
        str = cmd_result.decode("utf-8") 
        str2 = str.split()                  # Separate by space.
        str = list(str2)                    # Storing in an Array                    
        return removee(str)
    except:
        print("i2c detect error")
        return -1

class ChatBot(JabberBot):
    """
    XMPP/Jabber bot
    """

    _content_commands = {}
    _global_commands = []
    _command_aliases = {}
    _all_msg_handlers = []
    _last_message = ''
    _last_send_time = time.time()
    _restart = False

    def __init__(self, config):
        self._config = config
        channel = config['connection']['channel']
        username = u"%s@%s" % (config['connection']['username'],
                               config['connection'].get('host',
                                                        config['connection']['host']))
        self._username = username
        super(ChatBot, self).__init__(
            username=username,
            password=config['connection']['password'])

        self.PING_FREQUENCY = 50                                             # timeout sec
        self.join_room(channel, config['connection']['nickname'])
        self.log.setLevel(logging.INFO)

    def join_room(self, room, username=None, password=None):
        """
        ChatRoomにjoin
        """
        NS_MUC = 'http://jabber.org/protocol/muc'
        if username is None:
            username = self._username.split('@')[0]
        my_room_JID = u'/'.join((room, username))
        pres = xmpp.Presence(to=my_room_JID)
        if password is not None:
            pres.setTag(
                'x', namespace=NS_MUC).setTagData('password', password)
        else:
            pres.setTag('x', namespace=NS_MUC)

        pres.getTag('x').addChild('history', {'maxchars': '0',
                                              'maxstanzas': '0'})
        self.connect().send(pres)

    def callback_message(self, conn, mess):
        """
        get the data and report it
        """
        _type = mess.getType()
        jid = mess.getFrom()
        props = mess.getProperties()
        text = mess.getBody()
        username = self.get_sender_username(mess)

        print "callback_message:{}".format(text)
        print _type
        # print jid
        print props
        print username
        super(ChatBot, self).callback_message(conn, mess)

        # get the data
        HUM, TEMP = get_data()

        # run chatbot with responses as below
        time.sleep(1)
        if 'temp' in text:
            ret = [f"Temperature Room1 is {TEMP}" ]
            self.send_simple_reply(mess, ret[0])
        if 'hum' in text:
            ret = [f"Humidity Room1 is {HUM}" ]
            self.send_simple_reply(mess, ret[0])

        # this example chooses a random message for you
        if 'lights on' in text:
            suc = drive_op_with_feedback()
            if suc == True:
                ret = ["turned lights on",
                       "room lit up",
                       "lights are on",
                       "lights have been activated"]
            else:
                ret = ["failed to get confirmation",
                       "try again lights didnt come on"]           
            self.send_simple_reply(mess, random.choice(ret))

        if 'lights off' in text:
            suc = drive_op_with_feedback(mode="off")
            if suc == True:
                ret = ["turned lights off",
                       "room dark",
                       "lights are off",
                       "lights have been deactivated"]
            else:
                ret = ["failed to get confirmation",
                       "try again lights didnt come off"]           
            self.send_simple_reply(mess, random.choice(ret))

        if 'setpoint temp' in text:
            val_sp = text.split("=")[1]
            datav = struct.pack("<f", val_sp)
            valu_sp = eeprom(datav, base=b'\x00\x00', mode="w", adddre=0x50, byt=4, f=True, e=1)
            ret = [f"set temp setpoint to {valu_sp}",
                   f"temp alarm set to {valu_sp}"]           
            self.send_simple_reply(mess, random.choice(ret))

        if 'setpoint hum' in text:
            val_sp = text.split("=")[1]
            datav = struct.pack("<f", val_sp)
            valu_sp = eeprom(datav, base=b'\x00\x08', mode="w", adddre=0x50, byt=4, f=True, e=1)
            ret = [f"set humidity setpoint to {valu_sp}",
                   f"humidity alarm set to {valu_sp}"]           
            self.send_simple_reply(mess, random.choice(ret))

    def send_message(self, mess):
        """Send an XMPP message
        Overridden from jabberbot to update _last_send_time
        """
        self._last_send_time = time.time()
        self.connect().send(mess)

def bot_start(conf):
    print "++++++++"
    print conf
    print "++++++++"
    bot = ChatBot(conf)
    bot.serve_forever()

def do_local_alarm():
    global AL_ON
    i2cdev_found = probe_i2c()
    print("i2c devices found were ....", i2cdev_found)
    while AL_ON == True:
        h, t = get_data()
        drive_alarm(t)
        drive_alarm(h, 19, b'\x00\x08')
        time.sleep(1)

class ChatDaemon(object):
    config = None

    def run(self):
        try:
            # Start Slack Bot
            process_slack = mp.Process(target=bot_start, args=(self.config_slack,))
            process_slack.start()

            # Start HipChat Bot
            process_hipchat = mp.Process(target=bot_start, args=(self.config_hipchat,))
            process_hipchat.start()

            # local alarm for temp and humidity from the set-points set in the eeprom from chatbot commands
            process_la = mp.Process(target=do_local_alarm, args=())
            process_la.start()
        except Exception, e:
            print >> sys.stderr, "ERROR: %s" % (e,)
            print >> sys.stderr, traceback.format_exc()
            return 1
        else:
            return 0

def main():
    logging.basicConfig()
    config_slack = {
        'connection': {
            'username': '{{name}}',
            'password': '{{password}}',
            'nickname': '{{name}}',
            'host': '{{TeamName}}.xmpp.slack.com',
            'channel': '{{RoomName}}@conference.{{TeamName}}.xmpp.slack.com',
        }
    }
    config_hipchat = {
        'connection': {
            'username': '{{name}}',
            'password': '{{password}}',
            'nickname': '{{nickname}}',
            'host': 'chat.hipchat.com',
            'channel': '{{RoomName}}@conf.hipchat.com',
        }
    }

    runner = ChatDaemon()
    runner.config_slack = config_slack
    runner.config_hipchat = config_hipchat
    runner.run()

if __name__ == '__main__':
    sys.exit(main())
