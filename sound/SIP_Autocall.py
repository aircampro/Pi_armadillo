#!/usr/bin/env python
#
# Example of making automated SIP voice call and hanging-up 
#
import pjsua as pj
import sys
import pyttsx3
import time

# index_number for test calls 0=loopback echo call 1=music 2=your id
CALL_LIST=['sip:echo@iptel.org', 'sip:music@iptel.org', 'sip:john.doe@iptel.org']

class MyAccCallback(pj.AccountCallback):
    def on_reg_state(self, account, param):
        print("reg state : ", param.code)

PORT=5060
def main(args):
    idx = -1
    useremail = ""
    what_to_say = "hello, this is a test"
    if len(args) > 1:
        try:
            idx = int(args[1])                                  # index number for testing.. 
            idx %= 4
        except:
            useremail = str(args[1])                            # actual email e.g. sip:john.doe@iptel.org
        if len(args) > 2:
            what_to_say = str(args[2])
    else:
        print(f"usage: {args[0]} [ index number 0-loopback 1-music or e.g. sip:m.s@iptel.org ]")
        sys.exit(-1)

    # text to speach engine
    engine = pyttsx3.init()
    # Get the available voices
    voices = engine.getProperty('voices')
    # Show the desired voice(s) (index 0 represents the first voice in the list)
    print(voices)
    engine.setProperty('voice', voices[0].id)
    # Set the speech rate (words per minute)
    engine.setProperty('rate', 100)
    # Set the volume (0.0 to 1.0)
    engine.setProperty('volume', 1.0)
    lib = pj.Lib()
    try:
        lib.init()
        transport = lib.create_transport(pj.TransportType.UDP, pj.TransportConfig(PORT))
        lib.start()
        acc_cfg = pj.AccountConfig(domain="iptel.org", username="user", password="passw")
        acc_cb = MyAccCallback()
        acc = lib.create_account(acc_cfg, cb=acc_cb) 
        if idx == -1:
            if useremail.find("@") == -1 or useremail.find("sip:") == -1 or useremail.find(".") == -1:
                print("send id must contain a sip: @ .")
                sys.exit(-1) 
            call = acc.make_call(useremail)
        else:
            call = acc.make_call(CALL_LIST[idx])
        engine.say(what_to_say)
        engine.runAndWait()
        time.sleep(5)
        call.hangup()
    finally:
        acc.delete()
        lib.destroy()

if __name__ == "__main__":
    args = sys.argv
    main(args)
