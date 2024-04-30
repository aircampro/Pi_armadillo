#!/usr/bin/env python3
# --------------------------------------------------
#
# Generates morse code sound from string passed 
# morse_code_generator.py "this is the message to send"
#
# "Hiragana" to "Katakana" conversion for japanese
# pip install jaconv
# --------------------------------------------------
from jaconv import hira2kata
import re, unicodedata
from time import sleep
import platform

# standard morse dictionary with extended chars
#
MORSE_DICT ={
    "A" : ".-",
    "B" : "-...",
    "C" : "-.-.",
    "D" : "-..",
    "E" : ".",
    "F" : "..-.",
    "G" : "--.",
    "H" : "....",
    "I" : "..",
    "J" : ".---",
    "K" : "-.-",
    "L" : ".-..",
    "M" : "--",
    "N" : "-.",
    "O" : "---",
    "P" : ".--.",
    "Q" : "--.-",
    "R" : ".-.",
    "S" : "...",
    "T" : "-",
    "U" : "..-",
    "V" : "...-",
    "W" : ".--",
    "X" : "-..-",
    "Y" : "-.--",
    "Z" : "--..",
    " " : "/",
    "1" : ".----",
    "2" : "..---",
    "3" : "...--",
    "4" : "....-",
    "5" : ".....",
    "6" : "-....",
    "7" : "--...",
    "8" : "---..",
    "9" : "----.",
    "0" : "-----",
    "." : ".-.-.-",
    "," : "--..--",
    ":" : "---...",
    ";" : "-.-.-.",
    "?" : "..--..",
    "'" : ".----.",
    "-" : "-....-",
    "/" : "-..-.",
    "@" : ".--.-.",
    "=" : "-...-",
    "'" : ".----.",
    "(" : "-.--.",
    ")" : "-.--.-",
    "*" : "-.-.-",
    "+" : ".-.-.",
    "_" : "..__._.",
    "!" : "-.-.--",
    "Æ" : "._._",
    "Ä" : "._._",
    "Ø" : "___.",
    "Ö" : "___.",
    "Ó" : "___.",
    "Å" : ".__._",
    "À" : ".__._",
    "Ç" : "_._..",
    "Ü" : "..--",
    "É" : "..-..",
    "\"" : ".-..-.",
    "$" : "...-..-"
}

# japanese dictionary is in katakana
#
MORSE_DICT_JP = {
    'ア' : '--.--',
    'カ' : '.-..',
    'サ' : '-.-.-',
    'タ' : '-.',
    'ナ' : '.-.',
    'ハ' : '-...',
    'マ' : '-..-',
    'ヤ' : '.--',
    'ラ' : '...',
    'ワ' : '-.-',
    'イ' : '.-',
    'キ' : '-.-..',
    'シ' : '--.-.',
    'チ' : '..-.',
    'ニ' : '-.-.',
    'ヒ' : '--..-',
    'ミ' : '..-.-',
    'リ' : '--.',
    'ヰ' : '.-..-',
    'ウ' : '..-',
    'ク' : '...-',
    'ス' : '---.-',
    'ツ' : '.--.',
    'ヌ' : '....',
    'フ' : '--..',
    'ム' : '-',
    'ユ' : '-..--',
    'ル' : '-.--.',
    'ン' : '.-.-.',
    'エ' : '-.---',
    'ケ' : '-.--',
    'セ' : '.---.',
    'テ' : '.-.--',
    'ネ' : '--.-',
    'ヘ' : '.',
    'メ' : '-...-',
    'レ' : '---',
    'ヱ' : '.--..',
    'オ' : '.-...',
    'コ' : '----',
    'ソ' : '---.',
    'ト' : '..-..',
    'ノ' : '..--',
    'ホ' : '-..',
    'モ' : '-..-.',
    'ヨ' : '--',
    'ロ' : '.-.-',
    'ヲ' : '.---',
    '゛' : '..',
    '゜' : '..--.',
    '。' : '.-.-..',
    'ー' : '.--.-',
    '、' : '.-.-.-',
    '（' : '-.--.-',
    '）' : '.-..-.',
    '？' : '..--..',
    '！' : '-.-.--'
}

# russian morse dictionary 
#
MORSE_DICT_RU ={
    "A" : ".-",
    "Б" : "-...",
    "Ц" : "-.-.",
    "Д" : "-..",
    "E" : ".",
    "Ф" : "..-.",
    "Г" : "--.",
    "X" : "....",
    "И" : "..",
    "Й" : ".---",
    "K" : "-.-",
    "Л" : ".-..",
    "M" : "--",
    "H" : "-.",
    "O" : "---",
    "П" : ".--.",
    "Щ" : "--.-",
    "P" : ".-.",
    "С" : "...",
    "T" : "-",
    "У" : "..-",
    "Ж" : "...-",
    "В" : ".--",
    "Ь" : "-..-",
    "Ы" : "-.--",
    "З" : "--..",
    "Ч" : "---.",	
	"Ш" : "----",
	"Ъ" : "--.--",
	"Э" : "..-..",
	"Ю" : "..--",
	"Я" : ".-.-",	
    " " : "/",
    "1" : ".----",
    "2" : "..---",
    "3" : "...--",
    "4" : "....-",
    "5" : ".....",
    "6" : "-....",
    "7" : "--...",
    "8" : "---..",
    "9" : "----.",
    "0" : "-----",
    "." : ".-.-.-",
    "," : "--..--",
    ":" : "---...",
    ";" : "-.-.-.",
    "?" : "..--..",
    "'" : ".----.",
    "-" : "-....-",
    "/" : "-..-.",
    "@" : ".--.-.",
    "=" : "-...-",
    "'" : ".----.",
    "(" : "-.--.",
    ")" : "-.--.-",
    "*" : "-.-.-",
    "+" : ".-.-.",
    "_" : "..__._.",
    "!" : "-.-.--",
    "\"" : ".-..-.",
    "$" : "...-..-"
}

def morse_encode(msg, morse_dict) -> list:
    encoded_lst = []
    for i in msg.upper():
        if i in morse_dict:
            encoded_lst.append(morse_dict.get(i))
        else:
            encoded_lst.append('[ERROR]')
            print('ERROR: ', i, 'is not in the dictionary.')
    return ' '.join(encoded_lst)

def morse_decode(msg, morse_dict) -> str:
    decoded_msg = ''
    reverse_morse_dict = dict((v,k) for (k,v) in morse_dict.items())
    for i in msg.split(' '):
        if i in reverse_morse_dict:
            decoded_msg += reverse_morse_dict.get(i)
        else:
            decoded_msg += '[ERROR]'
            print('ERROR: ', i, 'is not in the dictionary.')
    return decoded_msg

def dakuten_separator(msg):
    bytes_text = unicodedata.normalize('NFD', msg).encode()
    bytes_text = re.sub(b'\xe3\x82\x99', b"\xe3\x82\x9b", bytes_text)
    bytes_text = re.sub(b"\xe3\x82\x9a", b'\xe3\x82\x9c', bytes_text)
    return bytes_text.decode()

def morse_beep(freq, dur=100, sys_has_play_installed=True, e3_dev=False):
    """
        Create a beep tone for sending the morse
        @param freq tone freq specified in Hz
        @param dur  duration of sound in （ms）
        @param sys_has_play_installed if you have the play command installed
        @param e3_dev you have installed the e3dev kit https://www.ev3dev.org/docs/getting-started/
    """
    pf = platform.system()
    if pf == "Windows":                                                                       # on windows with winsound library
        # use the winsound library
        import winsound
        winsound.Beep(freq, dur)
    elif (pf == 'Darwin' or pt == 'Linux') and (sys_has_play_installed):                      # apple mac or linux has play installed
        # one method on MAC or linux is to install play
        import os
        os.system('play -n synth %s sin %s' % (dur/1000, freq))
    elif pt == 'Linux' and not e3_dev:                                                        # only got linux  
        import fcntl
        import time
        import os
        len = dur / 1000                                                                      # convert millisecond to second
        KIOCSOUND = 0x4B2F
        CLOCK_TICK_RATE = 1193180
        fd = os.open("/dev/console", os.O_WRONLY)
        try:
            fcntl.ioctl(fd, KIOCSOUND, CLOCK_TICK_RATE / hz)
            try:
                time.sleep(len)
            finally:
                fcntl.ioctl(fd, KIOCSOUND, 0)
        finally:
            os.close(fd)
    elif pt == 'Linux' and e3_dev:                                                            # e3-dev installed on belena or using LEGO mindstorm ev3
        from ev3dev2.sound import Sound
        my_sound = Sound()
        sound_settings = (freq, dur, 0, Sound.PLAY_WAIT_FOR_COMPLETE) 
        my_sound.tone(sound_settings)        

# play the morse code with the correct frquency 300Hz and duration in ms
# 
def morse_sound(morse):
    for i in morse:
        if i == '-':
            morse_beep(300, 75*3)
        elif i == '.':
            morse_beep(300, 75)
        else:
            sleep(0.075)

def send_morse_sound(msg, morse_dict):
    msg = dakuten_separator(hira2kata(msg))                 # kata = jaconv.hira2hkata(hira) for japanese
    encoded_msg = morse_encode(msg, morse_dict)
    print(encoded_msg)
    decoded_msg = morse_decode(encoded_msg, morse_dict)
    print(decoded_msg)
    morse_sound(encoded_msg)

if __name__ == '__main__':
    argc = len(sys.argv)
    
    # test in english
    if argc >= 2:
        send_morse_sound(sys.argv[1], MORSE_DICT)
    else:
        send_morse_sound("This is the morse ocde test message!, 123890 : test_again ?", MORSE_DICT)
        
    # test in japanese
    if argc >= 3:
        send_morse_sound(sys.argv[2], MORSE_DICT_JP)
    else:
        send_morse_sound('よう！げんき？', MORSE_DICT_JP)
        
    # test in russian
    if argc >= 4:
        send_morse_sound(sys.argv[3], MORSE_DICT_RU)
    else:
        send_morse_sound('Используйте таблицу азбуки Морзе', MORSE_DICT_RU)
