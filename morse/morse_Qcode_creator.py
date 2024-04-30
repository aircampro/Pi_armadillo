#!/usr/bin/env python3
# morse Q Code and abbreviation encoder
#

# translate common known Q codes and abreviations
def Qcode_encode( msg ):
    # convert to lower case first
    senta = msg.lower()
    translated = ""
    if not senta.find("stop transmitting") == -1:
        senta.replace("stop transmitting","QRT")
    if not senta.find("do you have anything for me?") == -1:
        senta.replace("do you have anything for me?","QRU?")
    if not senta.find("are you ready?") == -1:
        senta.replace("are you ready?","QRV?")
    if not senta.find("your turn") == -1:
        senta.replace("your turn","QRY?")
    if not senta.find("who's calling me?") == -1:
        senta.replace("who's calling me?","QRZ?")
    if not senta.find("your signals freeze") == -1:
        senta.replace("your signals freeze","QSB")       
    if not senta.find("your manipulation is flawed") == -1:
        senta.replace("your manipulation is flawed","QSD") 
    if not senta.find("i confirm receiving your messages") == -1:
        senta.replace("i confirm receiving your messages","QSL")         
    if not senta.find("repeat the last message") == -1:
        senta.replace("repeat the last message","QSM") 
    if not senta.find("communication with directly") == -1:
        senta.replace("communication with directly","QSO") 
    if not senta.find("can you pass") == -1:
        senta.replace("can you pass","QSP?")        
    if not senta.find("do you have a doctor?") == -1:
        senta.replace("do you have a doctor?","QSQ?") 
    if not senta.find("my operating frequency") == -1:
        senta.replace("my operating frequency","QSS") 
    if not senta.find("your operating frequency") == -1:
        senta.replace("your operating frequency","QSU") 
    if not senta.find("give the message") == -1:
        senta.replace("give the message","QSV")         
    if not senta.find("transmit on this frequency") == -1:
        senta.replace("transmit on this frequency","QSW")
    if not senta.find("i have a message for you") == -1:
        senta.replace("i have a message for you","QTC")        
    if not senta.find("let us know your coordinates") == -1:
        senta.replace("let us know your coordinates","QTH") 
    if not senta.find("what port did you leave from") == -1:
        senta.replace("what port did you leave from","QTO?")
    if not senta.find("what port did you call at") == -1:
        senta.replace("what port did you call at","QTP?")
    if not senta.find("i received a distress call from") == -1:
        senta.replace("i received a distress call from","QUF?")
    if not senta.find("thanks") == -1:
        senta.replace("thanks","tu")        
    if not senta.find("number of words") == -1:
        senta.replace("number of words","ck") 
    if not senta.find("to everyone") == -1:
        senta.replace("to everyone","cq") 
    if not senta.find("good day") == -1:
        senta.replace("good day","gd") 
    if not senta.find("signature") == -1:
        senta.replace("signature","sig") 
    if not senta.find("i am") == -1:
        senta.replace("i am","de") 
    if not senta.find("message") == -1:
        senta.replace("message","msg")
    if not senta.find("understood") == -1:
        senta.replace("understood","r")
    if not senta.find("urgent") == -1:
        senta.replace("urgent","urg")        
    if not senta.find("estimated arrival time") == -1:
        senta.replace("estimated arrival time","eta")
    if not senta.find("weather forcast") == -1:
        senta.replace("weather forcast","wx")
    if not senta.find("repeat") == -1:
        senta.replace("repeat","rpt")
    if not senta.find("best wishes") == -1:
        senta.replace("best wishes","73")
    if not senta.find("regards") == -1:
        senta.replace("regards","73")
    if not senta.find("all the best") == -1:
        senta.replace("all the best","73")
    if not senta.find("love") == -1:
        senta.replace("love","88")        
    if not senta.find("invitation to work") == -1:
        senta.replace("invitation to work","k") 
    if not senta.find("motor ship") == -1:
        senta.replace("motor ship","m/v") 

    return senta
    
if __name__ == '__main__':
    argc = len(sys.argv)
    
    # pass the message and encode if needed
    if argc >= 2:
        enc_msg = Qcode_encode(sys.argv[1])
        print(enc_msg)
    else:
        print("please pass the sentance on the command line in quotes"	