#!/usr/bin/env python3
# morse Q Code and abbreviation translator
#

# translate common known Q codes and abreviations
def Qcode_decode( senta ):
    translated = ""
    for word in senta.split(' '):
        if word == "QRT":
            translated += "Stop transmitting"
        elif word.upper() == "QRU?":
            translated += "Do you have anything for me?"
        elif word.upper() == "QRV?":
            translated += "Are you ready?"
        elif word.upper() == "QRY":
            translated += "Your turn"
        elif word.upper() == "QRZ?":
            translated += "Who's calling me?"
        elif word.upper() == "QSB":
            translated += "Your signals freeze"
        elif word.upper() == "QSD":
            translated += "Your manipulation is flawed"
        elif word.upper() == "QSL":
            translated += "I confirm receiving your messages"
        elif word.upper() == "QSM":
            translated += "Repeat the last message"
        elif word.upper() == "QSO":
            translated += "Communication with... directly"
        elif word.upper() == "QSP?":
            translated += "Can you pass...?"
        elif word.upper() == "QSQ?":
            translated += "Do you have a doctor?"
        elif word.upper() == "QSS":
            translated += "My operating frequency..."
        elif word.upper() == "QSU":
            translated += "Your operating frequency..."
        elif word.upper() == "QSV":
            translated += "Give (I give) the setting"
        elif word.upper() == "QSW":
            translated += "Transmit on this frequency"
        elif word.upper() == "QTC":
            translated += "I have a message for you"
        elif word.upper() == "QTH":
            translated += "Let us know your coordinates"
        elif word.upper() == "QTO?":
            translated += "What port did you leave from"
        elif word.upper() == "QTP?":
            translated += "What port do you call at ?"
        elif word.upper() == "QUF":
            translated += "I received a distress call from..."
        elif word.lower() == "tu":
            translated += "thanks"
        elif word.lower() == "ck":
            translated += "number of words"
        elif word.lower() == "cq":
            translated += "to everyone"
        elif word.lower() == "gd":
            translated += "good day"
        elif word.lower() == "sig":
            translated += "signature"
        elif word.lower() == "de":
            translated += "I am"
        elif word.lower() == "msg":
            translated += "message"
        elif word.lower() == "r":
            translated += "understood"
        elif word.lower() == "urg":
            translated += "urgent"
        elif word.lower() == "eta":
            translated += "estimated arrival time"
        elif word.lower() == "wx":
            translated += "weather forcast"
        elif word.lower() == "rpt":
            translated += "please repeat"
        elif word.lower() == "73":
            translated += "best wishes"
        elif word.lower() == "88":
            translated += "love and kisses"
        elif word.lower() == "k":
            translated += "invitation to work"
        elif word.lower() == "m/v":
            translated += "motor ship"
        else:
            translated += word+" "
    return translated
       

if __name__ == '__main__':
    argc = len(sys.argv)
    
    # translate the sentance passed to the shell
    if argc >= 2:
        tt = Qcode_decode(sys.argv[1])
        print(tt)
    else:
        print("please pass the sentance on the command line in quotes"	