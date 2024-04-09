#!/usr/bin/env python3
# -*- coding: utf-8 -+-
#
# ------------------------------------------------------------------------------
# Read QR Codes using webcam and open a door or alarm depending
# if time and station code are allowed
# barrier for entry to travel to train and exit after travel
# there is an option to use one single common exit for both directions of travel 
# in that case we use a station_from and station_to pair to check for valid exit
# permutation.
# -----------------------------------------------------------------------------
# choose the decoder to be pyzbar or openCV
DECODER="pyzbar"                                                                # choice "pyzbar" or "opencv"

import cv2
if DECODER == "pyzbar" :
    from pyzbar.pyzbar import decode, ZBarSymbol
import board
import RPi.GPIO as GPIO
import datetime

# GPIO pin scheme
GPIO.setmode(GPIO.BCM)                                                           # BCM channel, ex GPIO#

# I/O for the barrier / turnstile mechanism
coil_relay = 14; GPIO.setup(coil_relay, GPIO.OUT)                                               # for opening or closing it
alarm_relay = 6; GPIO.setup(alarm_relay, GPIO.OUT)                                              # for alert 
GPIO.output(coil_relay, 0)
GPIO.output(alarm_relay, 0)
exited_switch = 7; GPIO.setup(exited_switch, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)               # to denote object passed the exit

# you are permitted to exit at stations before the destination and up to it unless specific route and then code is -ve
#
STATION_ID=4
STATION_CODE=-STATION_ID
# override codes are for the manual operators cards
ALARM_OFF=-999                            # QR Code for operator release ALARM
FORCE_CLS=-998                            # QR Code for operator release FORCE turnstile to close
FORCE_OPN=-997                            # QR Code for operator release FORCE turnstile to open
FORCE_OFF=-996                            # QR Code for operator release FORCE turnstile to permanantly close
FORCE_RESET=-995                          # QR Code for operator release RESET turnstile to operation
ADD_STATIONS=-994                         # Add a series of destinations which are not in numerical sequence but are valid for this station
CLR_ADD_STATIONS=993                      # clear the above list of station exceptions
APPEND_STATIONS=-992                      # append to the list of extra stations
APPEND_ANOTHER=-991                       # use this card to append another list as above
SAVE_STATIONS=-990                        # write the station list to file
CHG_IV_CRYPTO=-989                        # read a new iv which is written in place of station number

FORCE_ALWAYS_OFF = 0                      # flag indicating the closure of the turnstile from normal operation i.e. ignore all cards

ACTIVE_QR=0                               # active QR code detected flag
EXIT_DELAY=2                              # time to wait before closing exit door
STAT_ALARM_OVR=0                          # station override is on
NO_ALM_CHK=0                              # ignore alarm conditions if manual override card is being used

# allow extra stations with numbers out of sequence to be allowed
EXTRA_STAT=[]

# this option is where the gate / barrier or turnstil is not just of the side of track but has a common exit 
# we therefore use pairs of nodes node_from=node_to
# eg. / 22-23, 22-26, 21-23
COMMON_EXIT_ROUTE=0

APPEND_DONE=0                                  # set when an append card has been used and cleared when append another presented or any other valid QR

# -----------------------------------------------------------
# Handle encryption 
# -----------------------------------------------------------
USE_CRYPT=0                                                   # use a encryption and decryption anti-fraud for the QR Codes 1=plain 2=hexlified etc as below
if argc >= 1 :                                                  # we specified a decryption to be used
    USE_CRYPT=int(argv[1])
    
DECRYPT_PW="my_qr_decoder"                                      # same password as the encoder uses
if USE_CRYPT == 1 :
    from simplecrypt import encrypt, decrypt
elif USE_CRYPT == 2 :  
    from simplecrypt import encrypt, decrypt
    from binascii import hexlify , unhexlify
elif USE_CRYPT >= 3 and USE_CRYPT <= 10 :                                          # blowfish modes
    import blowfish
    cipher = blowfish.Cipher(b"Key must be between 4 and 56 bytes long.")              # key used with blowfish
    # for the ciphers using an iv it should be read from the QR code and generated each time at the printing device
    from os import urandom
    # iv = urandom(8)                                                                  # initialization vector should be same as printing devices iv (this is to get from random number"
    iv = b'\x0f' * 8                                                                   # using a fixed number same as printer
elif USE_CRYPT == 11 :                                                               # use the chilkat library
    import chilkat2
    # This example assumes the Chilkat API to have been previously unlocked.
    # See Global Unlock Sample for sample code.
    crypt = chilkat2.Crypt2()
    # Attention: use "blowfish2" for the algorithm name:
    # "chacha20", "pki", "aes", "blowfish2", "des", "3des", "rc2", "arc4", "twofish", "pbes1" and "pbes2". 
    # The "pki" encryption algorithm isn't a specific algorithm, but instead tells the component to encrypt/decrypt 
    # using public-key encryption with digital certificates. The other choices are symmetric encryption algorithms that 
    # do not involve digital certificates and public/private keys.
    # If an application needs to decrypt something encrypted with the old 4321 byte-swapped blowfish, set the property to "blowfish_old"
    crypt.CryptAlgorithm = "blowfish2"
    # CipherMode may be "ecb", "cbc", or "cfb" GCM: Galois/Counter Mode
    # XTS: AES-XTS (starting in Chilkat v9.5.0.91)
    crypt.CipherMode = "cfb"
    # KeyLength (in bits) may be a number between 32 and 448.
    # 128-bits is usually sufficient.  The KeyLength must be a 
    # multiple of 8.
    crypt.KeyLength = 128
    # The padding scheme determines the contents of the bytes
    # that are added to pad the result to a multiple of the
    # encryption algorithm's block size.  Blowfish has a block
    # size of 8 bytes, so encrypted output is always
    # a multiple of 8.
    # 0 = RFC 1423 padding scheme: Each padding byte is set to the number of padding bytes. If the data is already a multiple of algorithm's block size bytes, an extra block is appended each having a value equal to the block size. (for example, if the algorithm's block size is 16, then 16 bytes having the value 0x10 are added.). (This is also known as PKCS5 padding: PKCS #5 padding string consists of a sequence of bytes, each of which is equal to the total number of padding bytes added. )
    #
    # 1 = FIPS81 (Federal Information Processing Standards 81) where the last byte contains the number of padding bytes, including itself, and the other padding bytes are set to random values.
    #
    # 2 = Each padding byte is set to a random value. The decryptor must know how many bytes are in the original unencrypted data.
    #
    # 3 = Pad with NULLs. (If already a multiple of the algorithm's block size, no padding is added).
    #
    # 4 = Pad with SPACE chars(0x20). (If already a multiple of algorithm's block size, no padding is added).
    #
    crypt.PaddingScheme = 1
    # EncodingMode specifies the encoding of the output for
    # encryption, and the input for decryption.
    # It may be "hex", "url", "base64", or "quoted-printable".
    # The valid modes are "Base64", "modBase64", "base64url", "Base32", "Base58", "UU", "QP" (for quoted-printable), 
    # "URL" (for url-encoding), "Hex", "Q", "B", "url_oauth", "url_rfc1738", "url_rfc2396", "url_rfc3986", "fingerprint", or "decimal".
    crypt.EncodingMode = "hex"
    # Selects the hash algorithm used by methods that create hashes. The valid choices are "sha1", "sha256", "sha384", "sha512", "sha3-224", 
    # "sha3-256", "sha3-384", "sha3-512", "md2", "md5", "haval", "ripemd128", "ripemd160","ripemd256", or "ripemd320"
    # Note: The HAVAL hash algorithm is affected by two other properties: HavalRounds and KeyLength.
    # The HavalRounds may have values of 3, 4, or 5.
    # The KeyLength may have values of 128, 160, 192, 224, or 256
    crypt.HashAlgorithm = "sha256"
    # An initialization vector is required if using CBC or CFB modes.
    # ECB mode does not use an IV.
    # The length of the IV is equal to the algorithm's block size.
    # It is NOT equal to the length of the key.
    ivHex = "0001020304050607"                                                          # as used by the printer device
    crypt.SetEncodedIV(ivHex,"hex")
    # The secret key must equal the size of the key.  For
    # 256-bit encryption, the binary secret key is 32 bytes.
    # For 128-bit encryption, the binary secret key is 16 bytes.
    keyHex = "000102030405060708090A0B0C0D0E0F"                                         # as used by the printer device
    crypt.SetEncodedKey(keyHex,"hex")
elif USE_CRYPT >= 12 and USE_CRYPT <= 18 :                                              # blowfish modes
    from Crypto.Cipher import Blowfish
    from struct import pack
    bs = Blowfish.block_size
    key = b'An arbitrarily long key'                                                    # again set to that of the printer machine
    iv = b'\x00' * Blowfish.block_size 
elif USE_CRYPT == 19 :                                                                  # AES-XTS
    from Crypto.Cipher import AES
    import base64 
    password = "12345abcd"
    iv = "L3f4mlTJtCIPV9af"                                  
    mode = AES.MODE_CBC 
    # pad this data
    def mkpad(s,size):
        s = s.encode("utf-8") 
        pad = b' ' * (size - len(s) % size) 
        return s + pad
    def unpad_string(s):
        padding_size = ord(s[-1])
        return s[:-padding_size]   
    # encryption to base64
    def aes_encrypt(password,data):
        password = mkpad(password,16) 
        data = mkpad(data,16) 
        password = password[:16] 
        aes = AES.new(password,mode,iv)
        data_cipher = aes.encrypt(data)
        return base64.b64encode(data_cipher).decode("utf-8")
    def aes_decrypt(password,encdata):
        password = mkpad(password,16) 
        password = password[:16] 
        aes = AES.new(password,mode,iv)
        encdata = base64.b64decode(encdata) 
        data = aes.decrypt(encdata) 
        return unpad_string(data) 
elif USE_CRYPT == 20 :                                                                  # Blowfish CBC with padding and send iv in string
        from Crypto.Cipher import Blowfish
        from Crypto import Random      
        key = b'secret_key'                                                             # same for printer and turnstile    
        def blo_encrypt(key, plaintext):
            iv = Random.new().read(Blowfish.block_size)
            cipher = Blowfish.new(key, Blowfish.MODE_CBC, iv)
            padded_plaintext = _pad_string(plaintext)
            ciphertext = cipher.encrypt(padded_plaintext)
            return iv + ciphertext
        def blo_decrypt(key, ciphertext):
            iv = ciphertext[:Blowfish.block_size]
            cipher = Blowfish.new(key, Blowfish.MODE_CBC, iv)
            padded_plaintext = cipher.decrypt(ciphertext[Blowfish.block_size:])
            plaintext = _unpad_string(padded_plaintext)
            return plaintext
        def _pad_string(s):
            padding_size = Blowfish.block_size - len(s) % Blowfish.block_size
            padding = chr(padding_size) * padding_size
            return s + padding
        def _unpad_string(s):
            padding_size = ord(s[-1])
            return s[:-padding_size]
    
if USE_CRYPT == 10 :                                                                   # Blowfish CTR mode is chosen
    from operator import xor
    # increment by one counters
    # nonce_seed = urandom(8)
    nonce_seed = b'\x01' * 8
    nonce = int.from_bytes(nonce_seed, "big")                                          # like the iv this should match both printing device and gate
    enc_counter = blowfish.ctr_counter(nonce, f = xor)
    dec_counter = blowfish.ctr_counter(nonce, f = xor) 

# decrypts the data string passed and returns the encryption
def decrypt_data( qr_read ) :

    if USE_CRYPT == 1 :
        plaintext = decrypt(DECRYPT_PW, qr_read.encode('utf-8'))  
        qr_read = plaintext.decode('utf-8')
    elif USE_CRYPT == 2 :
        hextext = decrypt(DECRYPT_PW, qr_read.encode('utf-8'))   
        qr_read = unhexlify(hextext).decode('utf-8')
    elif USE_CRYPT == 3 :
        # Electronic Codebook Mode (ECB) 
        data_decrypted = b"".join(cipher.decrypt_ecb(qr_read.encode('utf-8'))  
        qr_read = data_decrypted.decode('utf-8')   
    elif USE_CRYPT == 4 :
        # Electronic Codebook Mode with Cipher Text Stealing (ECB-CTS)   
        data_decrypted = b"".join(cipher.decrypt_ecb_cts(qr_read.encode('utf-8'))) 
        qr_read = data_decrypted.decode('utf-8') 
    elif USE_CRYPT == 5 :
        # Cipher-Block Chaining Mode (CBC)   
        data_decrypted = b"".join(cipher.decrypt_cbc(qr_read.encode('utf-8'), iv))     
        qr_read = data_decrypted.decode('utf-8') 
    elif USE_CRYPT == 6 :
        # Cipher-Block Chaining with Ciphertext Stealing (CBC-CTS)   
        data_decrypted = b"".join(cipher.decrypt_cbc_cts(qr_read.encode('utf-8'), iv))     
        qr_read = data_decrypted.decode('utf-8') 
    elif USE_CRYPT == 7 :
        # Propagating Cipher-Block Chaining Mode (PCBC)   
        data_decrypted = b"".join(cipher.decrypt_pcbc(qr_read.encode('utf-8'), iv))     
        qr_read = data_decrypted.decode('utf-8') 
    elif USE_CRYPT == 8 :
        # Cipher Feedback Mode (CFB)   
        data_decrypted = b"".join(cipher.decrypt_cfb(qr_read.encode('utf-8'), iv))     
        qr_read = data_decrypted.decode('utf-8')     
    elif USE_CRYPT == 9 :
        # Output Feedback Mode (OFB)   
        data_decrypted = b"".join(cipher.decrypt_ofb(qr_read.encode('utf-8'), iv))     
        qr_read = data_decrypted.decode('utf-8')   
    elif USE_CRYPT == 10 :
        # Counter Mode (CTR)   
        data_decrypted = b"".join(cipher.decrypt_ctr(qr_read.encode('utf-8'), enc_counter))     
        qr_read = data_decrypted.decode('utf-8') 
    elif USE_CRYPT == 11 :
        # Cipher Feedback Mode CFB mode
        decStr = crypt.DecryptStringENC(qr_read)
        qr_read = decStr
    elif USE_CRYPT == 12 :
        # Cipher-Block Chaining (CBC)
        ciphertext = qr_read.encode('utf-8')
        iv = ciphertext[:bs]                     # the first block sent in this example is the iv
        ciphertext = ciphertext[bs:]             # the rest is the message
        cipher = Blowfish.new(key, Blowfish.MODE_CBC, iv)
        msg = cipher.decrypt(ciphertext)
        last_byte = msg[-1]
        msg = msg[:- (last_byte if type(last_byte) is int else ord(last_byte))]
        qr_read = msg.decode('utf-8')
    elif USE_CRYPT == 13 :
        # Electronic Code Book (ECB)
        ciphertext = qr_read.encode('utf-8')
        iv = ciphertext[:bs]                     # the first block sent in this example is the iv
        ciphertext = ciphertext[bs:]             # the rest is the message
        cipher = Blowfish.new(key, Blowfish.MODE_ECB, iv)
        msg = cipher.decrypt(ciphertext)
        last_byte = msg[-1]
        msg = msg[:- (last_byte if type(last_byte) is int else ord(last_byte))]
        qr_read = msg.decode('utf-8')
    elif USE_CRYPT == 14 :
        # Cipher FeedBack (CFB)
        ciphertext = qr_read.encode('utf-8')
        iv = ciphertext[:bs]                     # the first block sent in this example is the iv
        ciphertext = ciphertext[bs:]             # the rest is the message
        cipher = Blowfish.new(key, Blowfish.MODE_CFB, iv)
        msg = cipher.decrypt(ciphertext)
        last_byte = msg[-1]
        msg = msg[:- (last_byte if type(last_byte) is int else ord(last_byte))]
        qr_read = msg.decode('utf-8')
    elif USE_CRYPT == 15 :
        # Output FeedBack (OFB)
        ciphertext = qr_read.encode('utf-8')
        iv = ciphertext[:bs]                     # the first block sent in this example is the iv
        ciphertext = ciphertext[bs:]             # the rest is the message
        cipher = Blowfish.new(key, Blowfish.MODE_OFB, iv)
        msg = cipher.decrypt(ciphertext)
        last_byte = msg[-1]
        msg = msg[:- (last_byte if type(last_byte) is int else ord(last_byte))]
        qr_read = msg.decode('utf-8')
    elif USE_CRYPT == 16 :
        # CounTer Mode (CTR)
        ciphertext = qr_read.encode('utf-8')
        iv = ciphertext[:bs]                     # the first block sent in this example is the iv
        ciphertext = ciphertext[bs:]             # the rest is the message
        cipher = Blowfish.new(key, Blowfish.MODE_CTR, iv)
        msg = cipher.decrypt(ciphertext)
        last_byte = msg[-1]
        msg = msg[:- (last_byte if type(last_byte) is int else ord(last_byte))]
        qr_read = msg.decode('utf-8')
    elif USE_CRYPT == 17 :
        # OpenPGP Mode
        ciphertext = qr_read.encode('utf-8')
        iv = ciphertext[:bs]                     # the first block sent in this example is the iv
        ciphertext = ciphertext[bs:]             # the rest is the message
        cipher = Blowfish.new(key, Blowfish.MODE_OPENPGP, iv)
        msg = cipher.decrypt(ciphertext)
        last_byte = msg[-1]
        msg = msg[:- (last_byte if type(last_byte) is int else ord(last_byte))]
        qr_read = msg.decode('utf-8')
    elif USE_CRYPT == 18 :
        # EAX Mode
        ciphertext = qr_read.encode('utf-8')
        iv = ciphertext[:bs]                     # the first block sent in this example is the iv
        ciphertext = ciphertext[bs:]             # the rest is the message
        cipher = Blowfish.new(key, Blowfish.MODE_EAX, iv)
        msg = cipher.decrypt(ciphertext)
        last_byte = msg[-1]
        msg = msg[:- (last_byte if type(last_byte) is int else ord(last_byte))]
        qr_read = msg.decode('utf-8')
    elif USE_CRYPT == 19 :
        dec = aes_decrypt(password, qr_read.encode('base64'))  
        qr_read = dec
    elif USE_CRYPT == 20 :
        dec = blo_decrypt(key, qr_read)  
        qr_read = dec
        
    return qr_read

# encrypts the data string passed and returns the encryption
def encrypt_data( message ) :

    if USE_CRYPT == 1 :
        ciphertext = encrypt(password, message.encode('utf8')) 
    elif USE_CRYPT == 2 :
        ciphertext = encrypt(password, hexlify(message.encode('utf8')))   
    elif USE_CRYPT == 3 :
        # Electronic Codebook Mode (ECB) 
        ciphertext = b"".join(cipher.encrypt_ecb(message.encode('utf-8'))    
    elif USE_CRYPT == 4 :
        # Electronic Codebook Mode with Cipher Text Stealing (ECB-CTS)   
        ciphertext = b"".join(cipher.encrypt_ecb_cts(message.encode('utf-8')))  
    elif USE_CRYPT == 5 :
        # Cipher-Block Chaining Mode (CBC)   
        ciphertext = b"".join(cipher.encrypt_cbc(message.encode('utf-8'), iv))     
    elif USE_CRYPT == 6 :
        # Cipher-Block Chaining with Ciphertext Stealing (CBC-CTS)   
        ciphertext = b"".join(cipher.encrypt_cbc_cts(message.encode('utf-8'), iv))     
    elif USE_CRYPT == 7 :
        # Propagating Cipher-Block Chaining Mode (PCBC)   
        ciphertext = b"".join(cipher.encrypt_pcbc(message.encode('utf-8'), iv))     
    elif USE_CRYPT == 8 :
        # Cipher Feedback Mode (CFB)   
        ciphertext = b"".join(cipher.encrypt_cfb(message.encode('utf-8'), iv))        
    elif USE_CRYPT == 9 :
        # Output Feedback Mode (OFB)   
        ciphertext = b"".join(cipher.encrypt_ofb(message.encode('utf-8'), iv))       
    elif USE_CRYPT == 10 :
        # Counter Mode (CTR)   
        ciphertext = b"".join(cipher.encrypt_ctr(message.encode('utf-8'), dec_counter))     
    elif USE_CRYPT == 11 :
        # Cipher Feedback Mode CFB mode
        ciphertext = crypt.EncryptStringENC(message)
    elif USE_CRYPT == 12 :
        # Cipher-Block Chaining (CBC)
        cipher = Blowfish.new(key, Blowfish.MODE_CBC)
        plaintext = message.encode('utf-8')
        plen = bs - len(plaintext) % bs
        padding = [plen]*plen
        padding = pack('b'*plen, *padding)
        ciphertext = cipher.iv + cipher.encrypt(plaintext + padding)        
    elif USE_CRYPT == 13 :
        # Electronic Code Book (ECB)
        cipher = Blowfish.new(key, Blowfish.MODE_ECB, iv)
        plaintext = message.encode('utf-8')
        plen = bs - len(plaintext) % bs
        padding = [plen]*plen
        padding = pack('b'*plen, *padding)
        ciphertext = cipher.iv + cipher.encrypt(plaintext + padding)
    elif USE_CRYPT == 14 :
        # Cipher FeedBack (CFB)
        cipher = Blowfish.new(key, Blowfish.MODE_CFB, iv)
        plaintext = message.encode('utf-8')
        plen = bs - len(plaintext) % bs
        padding = [plen]*plen
        padding = pack('b'*plen, *padding)
        ciphertext = cipher.iv + cipher.encrypt(plaintext + padding)
    elif USE_CRYPT == 15 :
        # Output FeedBack (OFB)
        cipher = Blowfish.new(key, Blowfish.MODE_OFB, iv)
        plaintext = message.encode('utf-8')
        plen = bs - len(plaintext) % bs
        padding = [plen]*plen
        padding = pack('b'*plen, *padding)
        ciphertext = cipher.iv + cipher.encrypt(plaintext + padding)
    elif USE_CRYPT == 16 :
        # CounTer Mode (CTR)
        cipher = Blowfish.new(key, Blowfish.MODE_CTR, iv)
        plaintext = message.encode('utf-8')
        plen = bs - len(plaintext) % bs
        padding = [plen]*plen
        padding = pack('b'*plen, *padding)
        ciphertext = cipher.iv + cipher.encrypt(plaintext + padding)
    elif USE_CRYPT == 17 :
        # OpenPGP Mode
        cipher = Blowfish.new(key, Blowfish.MODE_OPENPGP, iv)
        plaintext = message.encode('utf-8')
        plen = bs - len(plaintext) % bs
        padding = [plen]*plen
        padding = pack('b'*plen, *padding)
        ciphertext = cipher.iv + cipher.encrypt(plaintext + padding)
    elif USE_CRYPT == 18 :
        # EAX Mode
        cipher = Blowfish.new(key, Blowfish.MODE_EAX, iv)
        plaintext = message.encode('utf-8')
        plen = bs - len(plaintext) % bs
        padding = [plen]*plen
        padding = pack('b'*plen, *padding)
        ciphertext = cipher.iv + cipher.encrypt(plaintext + padding)
    elif USE_CRYPT == 19 :
        ciphertext = aes_encrypt(password, qr_read.encode('base64'))  
    elif USE_CRYPT == 20 :
        ciphertext = blo_encrypt(key, qr_read)  
    else :
        ciphertext = "unsupported Encryption number passed"
        
    return ciphertext
    
# boot up to use a list of stations last saved
SAVE_STAT_FL="stations.txt"
try:
    with open(SAVE_STAT_FL, "r") as f:
        while True:
            line = f.readline()
            if not line:
                break
            else:
                EXTRA_STAT.append(line.split("\n")[0])
except:
    print("warning : no station excpetions file saved to disk")
 
# -----------------------------------------------------------
# Init
# -----------------------------------------------------------
font = cv2.FONT_HERSHEY_SIMPLEX

# -----------------------------------------------------------
# Capture webcam frames
# -----------------------------------------------------------
# VideoCapture
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

if DECODER == "opencv":
    # QRCodeDetector using open CV
    qrd = cv2.QRCodeDetector()

while cap.isOpened():
    ret, frame = cap.read()

    if ret:
        if DECODER == "opencv" :
            retval, decoded_info, points, straight_qrcode = qrd.detectAndDecodeMulti(frame)

            if retval:
                points = points.astype(np.int32)

                for qr_read, point in zip(decoded_info, points):
                    if qr_read == '':
                        continue

                    x = point[0][0]
                    y = point[0][1]

                    print('dec:', qr_read)
                    frame = cv2.putText(frame, qr_read, (x, y - 6), font, .3, (0, 0, 255), 1, cv2.LINE_AA)

                    frame = cv2.polylines(frame, [point], True, (0, 255, 0), 1, cv2.LINE_AA)   

                    # decrypt the data if we encrypted it
                    if not (USE_CRYPT == 0) : 
                        qr_read = decrypt_data( qr_read ) 
                        
            cv2.imshow('cv2', frame)      
        
        elif DECODER == "pyzbar" :
            # decode QR code from frame saved
            value = decode(frame, symbols=[ZBarSymbol.QRCODE])
        
            if value:
                for qrcode in value:

                    # QR code rectangle co-ordinates
                    x, y, w, h = qrcode.rect

                    # QR decode  should be YY-MM-DD-TS : station number
                    qr_read = qrcode.data.decode('utf-8')
                    print('dec:', qr_read)
                    frame = cv2.putText(frame, qr_read, (x, y-6), font, .3, (255, 0, 0), 1, cv2.LINE_AA)

                    # draw rectangle
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 1)

                    # decrypt the data if we encrypted it
                    if not (USE_CRYPT == 0) : 
                        qr_read = decrypt_data( qr_read ) 
                        
            # show image for testing purpose
            cv2.imshow('pyzbar', frame)

        else:
            print("invalid decoder specified.....")
            # show image for testing purpose
            cv2.imshow('no decoder specified', frame)
            
        # get the date and check the right day of travel and month and year for travel
        today_date = datetime.datetime.now()
        date_str = str(today_date).split(" ")[0]  
        time_str_hrs = int(str(today_date).split(" ")[1].split(".")[0].split(":")[0]) * 3600
        time_str_mins = int(str(today_date).split(" ")[1].split(".")[0].split(":")[1]) * 60
        time_str_secs = int(str(today_date).split(" ")[1].split(".")[0].split(":")[2]) 
        time_this_day = time_str_hrs + time_str_mins + time_str_secs 
        time_str_ms = int(str(today_date).split(" ")[1].split(".")[1])                 
        date_arr = date_str.split("-")
        t_yr = int(date_arr[0])
        t_mn = int(date_arr[1])
        t_dy = int(date_arr[2])
        
        if value :
            try:        
                # parse the QR read 2024;04;08;TS : 957          TS is time in secs from start of day that the journey may start
                QR_date = qr_read.split(":")[0]
                QR_date_arr = QR_date.split(";")
                QR_t_yr = int(QR_date_arr[0])
                QR_t_mn = int(QR_date_arr[1])
                QR_t_dy = int(QR_date_arr[2])
                QR_t_ts = int(QR_date_arr[3])
                if COMMON_EXIT_ROUTE == 1:
                    QR_stat_no = qr_read.split(":")[1].split("=")[0]
                    QR_stat_to = qr_read.split(":")[1].split("=")[1]  
                else:                    
                    QR_stat_no = qr_read.split(":")[1]
                    QR_stat_to = 0  
                    
                # parse the QR read -994;-994;-994;-994 : -994 / 123, 502, 967 , 976       TS is time in secs from start of day that the journey may start
                if ( QR_t_yr == ADD_STATIONS ) :                                                     # AS-AS-AS-AS  separated by / then 1, 2,  23
                    EXTRA_STAT = qr_read.split("/")[1].split(",")    
                    NO_ALM_CHK=1 
                    APPEND_DONE=0  
                elif ( QR_t_yr == APPEND_STATIONS ) and (APPEND_DONE == 0) :                          # AS-AS-AS-AS  separated by / then 1, 2,  23
                    card_read = qr_read.split("/")[1].split(",")    
                    EXTRA_STAT += card_read
                    NO_ALM_CHK=1 
                    APPEND_DONE=1                                                                     # latch out further action
                elif ( QR_t_yr == APPEND_ANOTHER ) and (APPEND_DONE == 1) : 
                    APPEND_DONE=0                
                elif ( QR_t_yr == CLR_ADD_STATIONS ) :                                                # clear the add station list
                    EXTRA_STAT = []    
                    NO_ALM_CHK=1 
                    APPEND_DONE=0    
                elif ( QR_t_yr == SAVE_STATIONS ) :                                                   # save to file the station list
                    with open(SAVE_STAT_FL, "w") as f:
                        f.write("\n".join(EXTRA_STAT))   
                    NO_ALM_CHK=1 
                    APPEND_DONE=0                      
                # check the text read from the QR code and open the gate if okay to travel or exit
                elif ( QR_t_yr == FORCE_OFF ) and (QR_stat_no == FORCE_OFF) :                          # force the gate permently closed
                    GPIO.output(coil_relay, 0) 
                    FORCE_ALWAYS_OFF = 1
                    NO_ALM_CHK=1 
                    APPEND_DONE=0                      
                elif ( QR_t_yr == FORCE_RESET ) and (QR_stat_no == FORCE_RESET) :                    # re-open the gate
                    FORCE_ALWAYS_OFF = 0
                    NO_ALM_CHK=1  
                    APPEND_DONE=0                      
                elif ( QR_t_yr == FORCE_CLS ) and (QR_stat_no == FORCE_CLS) :                        # manual close
                    GPIO.output(coil_relay, 0) 
                    NO_ALM_CHK=1   
                    APPEND_DONE=0                      
                elif ( QR_t_yr == FORCE_OPN ) and (QR_stat_no == FORCE_OPN) :                        # manual open
                    ACTIVE_QR == 1
                    NO_ALM_CHK=1    
                    APPEND_DONE=0  
                elif ( QR_t_yr == CHG_IV_CRYPTO ) :                                                # read a new iv for crypto
                    iv = QR_stat_no                                                                # the new iv was printed as the station number 
                    NO_ALM_CHK=1 
                    APPEND_DONE=0 
                elif (QR_stat_no >= STATION_ID) or (QR_stat_no == STATION_CODE) and (FORCE_ALWAYS_OFF == 0) and (COMMON_EXIT_ROUTE == 0) :   # station allowed
                    no_alarm = GPIO.input(alarm_relay)
                    if (no_alarm == 0) :
                        ACTIVE_QR == 1
                    NO_ALM_CHK=0
                    APPEND_DONE=0  
                elif (QR_stat_to == STATION_ID) or (QR_stat_to == STATION_CODE) and (FORCE_ALWAYS_OFF == 0) and (COMMON_EXIT_ROUTE == 1) :   # station allowed
                    no_alarm = GPIO.input(alarm_relay)
                    if (no_alarm == 0) :
                        ACTIVE_QR == 1
                    NO_ALM_CHK=0
                    APPEND_DONE=0 
                elif (FORCE_ALWAYS_OFF == 0) :
                    if COMMON_EXIT_ROUTE == 1:
                        for s in EXTRA_STAT:                                                             # check the extra stations list pairs of stations s=s
                            try:
                                if ((QR_stat_no == int(s.split("=")[0])) and (QR_stat_to == int(s.split("=")[1]))) :
                                    no_alarm = GPIO.input(alarm_relay)
                                    if (no_alarm == 0) : 
                                        ACTIVE_QR == 1
                                        STAT_ALARM_OVR = QR_stat_no
                                    break
                            except:
                                print("invalid code entered with ADD_STATIONS CARD code=", s)
                    else:
                        for s in EXTRA_STAT:                                                             # check the extra stations list single id okay for barrier
                            try:
                                if (QR_stat_no == int(s)) :
                                    no_alarm = GPIO.input(alarm_relay)
                                    if (no_alarm == 0) : 
                                        ACTIVE_QR == 1
                                        STAT_ALARM_OVR = QR_stat_no
                                    break
                            except:
                                print("invalid code entered with ADD_STATIONS CARD code=", s)
                    NO_ALM_CHK=0  
                    APPEND_DONE=0                      

                # check the date and time if not okay sound an alarm for manual checking and alarm reset via operator card
                if NO_ALM_CHK == 0:
                    if ( QR_t_yr == ALARM_OFF ) and (QR_stat_no == ALARM_OFF):                           # manual card presented to turn alarm off
                        GPIO.output(alarm_relay, 0) 
                    elif ( QR_t_yr < t_yr ) or ( QR_t_ts > time_this_day ) :                             # check not old ticket and not too early if it was restricted type
                        GPIO.output(alarm_relay, 1)
                    elif (t_mn >= 3):                                                                    # march or more
                        if (QR_t_mn < (t_mn-2)):                                                         # more than 3 months
                            GPIO.output(alarm_relay, 1)    
                        elif ((t_mn-2) == QR_t_mn) and (t_dy > QR_t_dy):                                 # old day
                            GPIO.output(alarm_relay, 1)                        
                    elif (t_mn == 2):                                                                    # feb
                        if not ((QR_t_mn == 2) or (QR_t_mn == 1) or (QR_t_mn == 12)) :
                            GPIO.output(alarm_relay, 1)  
                        elif ((t_mn==12) == QR_t_mn) and (t_dy > QR_t_dy):
                            GPIO.output(alarm_relay, 1)                         
                    elif (t_mn == 1):                                                                    # jan
                        if not ((QR_t_mn == 11) or (QR_t_mn == 1) or (QR_t_mn == 12)) :
                            GPIO.output(alarm_relay, 1) 
                        elif ((t_mn==11) == QR_t_mn) and (t_dy > QR_t_dy):
                            GPIO.output(alarm_relay, 1)   
                    elif (t_mn == QR_t_mn) and (t_dy < QR_t_dy):                                         # too early
                        GPIO.output(alarm_relay, 1)
                    elif (QR_stat_no < STATION_ID) and (not (QR_stat_no == STAT_ALARM_OVR)) and (FORCE_ALWAYS_OFF == 0) and (COMMON_EXIT_ROUTE == 0) :  # invalid station then alarm                                     
                        GPIO.output(alarm_relay, 1)
                          
                # if sensor sees an exit after opening then automatically close again after 2 secs delay
                #
                if ACTIVE_QR == 1:
                    no_alarm = GPIO.input(alarm_relay)
                    if (no_alarm == 0) :
                        GPIO.output(coil_relay, 1)                                                    # open barrier physically
                    else:
                        ACTIVE_QR = 0
                        
                while (ACTIVE_QR == 1) :                                                              # when barrier opened wait for close proximity to be passed by delay secs
                    person_passed = GPIO.input(exited_switch) 
                    if (person_passed == 1) :
                        person_has_exited = 1                                                         # latch we saw the object pass  
                    
                    if (person_has_exited == 1) and not (person_passed): 
                        time.sleep(EXIT_DELAY)                                                        # wait for delay before closing
                        GPIO.output(coil_relay, 0)
                        person_has_exited = 0
                        person_passed = 0
                        ACTIVE_QR = 0

            except:
                print("invalid parse of QR code... no value aparent", s)  
                
    # quit === after testing you can remove this option ?
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# release the cv capture
cap.release()