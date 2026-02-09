#!/usr/bin/env python
#
# Example of making SIP voice call 
#
import pjsua as pj
import sys

# index number for test calls 0=loopback echo call 1=music 2=your id
CALL_LIST=['sip:echo@iptel.org', 'sip:music@iptel.org', 'sip:john.doe@iptel.org']

class MyAccCallback(pj.AccountCallback):
    def on_reg_state(self, account, param):
        print("reg state : ", param.code)

def main(args):
    idx = -1
    useremail = ""
    if len(args) > 1:
        try:
            idx = int(args[1])                                  # index number for test
            idx %= 4
        except:
            useremail = str(args[1])                            # actual email e.g. sip:john.doe@iptel.org
    else:
        print(f"usage: {args[0]} [ index number 0-loopback 1-music or e.g. sip:m.s@iptel.org ]")
        sys.exit(-1)

    lib = pj.Lib()
    try:
        lib.init()
        transport = lib.create_transport(pj.TransportType.UDP, pj.TransportConfig(5060))
        lib.start()
        acc_cfg = pj.AccountConfig(domain="iptel.org", username="user", password="passw")
        acc_cb = MyAccCallback()
        acc = lib.create_account(acc_cfg, cb=acc_cb) 
        if idx == -1:
            if useremail.find("@") == -1 or useremail.find("sip:") == -1 or useremail.find(".") == -1:
                print("send id must contain all these sip: @ .")
                sys.exit(-1) 
            call = acc.make_call(useremail)
        else:
            call = acc.make_call(CALL_LIST[idx])
        lib.handle_events()
    finally:
        acc.delete()
        lib.destroy()

if __name__ == "__main__":
    args = sys.argv
    main(args)
