#!/usr/bin/python3
# example of using eSSP class fpr communication with gaming machines
#
import eSSP
import time

k = eSSP.eSSP('/dev/ttyACM0')
k.sync_start(1000)                   # sync to the remote gaming machine checkout

k.sync()
k.serial_number()
k.poll() # if you want to clear the buffers before you start
k.enable()
k.bulb_on()
#print k.bulb_off()
print k.enable_higher_protocol()
#print k.poll()
#print k.set_inhibits('0xFF', '0xFF')
#print k.set_inhibits(k.easy_inhibit([1, 0, 1]), '0x00')
#print k.unit_data()
#print k.setup_request();
#k.disable();
#k.reset();
#print k.channel_security();
#print k.channel_values();
#print k.channel_reteach();

# to turn on encryption (try this might need adjustment) 
k.sync()

# this is for testing as i'm not fully sure how it derives the AES key for encrpypt/decrpypt
#
ECRYPT_MODE = 0
if ECRYPT_MODE == 1:
    # send the shared generator and base/modulus to the machine
    r,g = k.send_generator()                ## send the shared generator g --- A = g^a mod p
    r,p = k.send_modulus()                  ## send the shared modulus p --- A = g^a mod p
    #r = k.send_key_exchange()

    # use diffie-hellman for key exchange
    mk = eSSP.DHKE(g, p)
    mk = k.generate_dh_key(p, g)

    # we would now generate keys and pipe the encrption/decryption to the send and recieve operations 
    my_share_key = mk.share_key
    print("share key being used : ",my_share_key)
       
    # 10 5 euro, 4 10 euro, 10 20 euro, 4 50 euro, 10 100 euro bills will be dispensed in this example
    my_payouts=[(500,10),(1000,4),(2000,10),(5000,4),(10000,10)]
    # without encryption k.payout_by_denomination(my_payouts)

    # use AES for encryption/decryption example shows this static key but i think its supposed to 
    # use the 2 share keys that we sent in the DH exchange
    fixedKey = '0123456701234567'
    
    # think we actually use the shared generator and shared modulus as the fixed key so ill overwrite it here consider endianess
    # Key Length
    # The standard varies depending on the length of the key.
    # AES-128: 128bit=16byte
    # i think that means its made out of g and p in the DH key exchange which is 2 64bit random mnumbers which we sent to the slave
    #
    ENDIAN = 0
    if ENDIAN == 0:
        fixedKey = g | (p << 64)
    else:
        fixedKey = p | (g << 64)

    # heres another way using a hashlib its not entirely clear how the library does this 
    # with the function ITL CreateSSPHostEncryptionKey
    #
    # fixedKey = k.create_AES_key(p, g)

    # now do the 128bit AES encrpytion
    k.payout_by_denomination_with_aes(my_share_key, fixedKey, my_payouts)
else:
    # $$======= if not try this example here im making sure that g>p before we send them ==========$$
    p1, g1, p_arr, g_arr = k.create_gen_mod()     # function to create generator and modulus from random primes and modulus is smallest
    r, g2 = k.send_generator_array(g_arr)         ## send the shared generator g --- A = g^a mod p
    r, p2 = k.send_modulus_array(p_arr)           ## send the shared modulus p --- A = g^a mod 
    if (p1 != p2) or (g1 != g2):                  # check the 2 match
        print("discrepancy with generator and modulus values")
        
    # use diffie-hellman for key exchange
    mk = eSSP.DHKE(g, p)
    mk = k.generate_dh_key(p, g)                  # it now sends A = g^a mod p and receives back B

    # we would now generate keys and pipe the encrption/decryption to the send and recieve operations 
    my_share_key = mk.share_key                   # calculated from g p and B received back
    print("share key being used : ", my_share_key)
       
    # 10 5 euro, 4 10 euro, 10 20 euro, 4 50 euro, 10 100 euro bills will be dispensed in this example
    my_payouts=[(500,10),(1000,4),(2000,10),(5000,4),(10000,10)]

    # heres another way using a hashlib its not entirely clear how the library does this 
    # with the function ITL CreateSSPHostEncryptionKey but this method is a very secure one using hashlib SHA256
    # i'm thinking it uses p and g to create it and the other key is the DH calculation using B recieved back from the key exchange
    #
    fixedKey = k.create_AES_key(str(p), str(g))

    # now do the 128bit AES encrpytion
    k.payout_by_denomination_with_aes(my_share_key, fixedKey, my_payouts)

# now turn off encryption
k.disable()
k.reset()
# re sync it....
k.sync()
k.payout_by_denomination(my_payouts)                # not sure it will now work without encrption
k.enable_higher_protocol()
k.set_inhibits(k.easy_inhibit([1, 0, 1]), '0x00')
k.enable()
var = 1
i = 0
while var == 1:
	poll = k.poll()
	print("Poll")
	
	if len(poll) > 1:
		if len(poll[1]) == 2:
			if poll[1][0] == '0xef':
				if poll[1][1] == 1 or poll[1][1] == 3:
					while i < 10:
						k.hold()
						print("Hold " + str(i))
						time.sleep(0.5)
						i += 1
			if poll[1][0] == '0xee':
				print("Credit on Channel " + str(poll[1][1]))
				i = 0
				
	time.sleep(0.5)
