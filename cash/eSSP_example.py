#!/usr/bin/python3
# example of using eSSP class fpr communication with gaming machines
#
import eSSP
import time

k = eSSP.eSSP('/dev/ttyACM0')
print k.sync()
print k.serial_number()
print k.poll() # if you want to clear the buffers before you start
print k.enable()
print k.bulb_on()
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
print k.sync()

# send the shared generator and base/modulus to the machine
r,g = k.send_generator()                ## send the shared generator g --- A = g^a mod p
r,p = k.send_modulus()                  ## send the shared modulus p --- A = g^a mod p
#r = k.send_key_exchange()

# use diffie-hellman for key exchange
mk = eSSP.DHKE(g, p)
mk = k.generate_dh_key(g, p)

# we would now generate keys and pipe the encrption/decryption to the send and recieve operations 
my_share_key = mk.share_key
        
# 10 5 euro, 4 10 euro, 10 20 euro, 4 50 euro, 10 100 euro bills will be dispensed in this example
my_payouts=[(500,10),(1000,4),(2000,10),(5000,4),(10000,10)]
# without encryption k.payout_by_denomination(my_payouts)

# use AES for encryption/decryption
fixedKey = '0123456701234567'
k.payout_by_denomination_with_aes(my_share_key, fixedKey, my_payouts)

# now turn off encryption
print k.disable()
print k.reset()
print k.sync()
print k.enable_higher_protocol()
print k.set_inhibits(k.easy_inhibit([1, 0, 1]), '0x00')
print k.enable()
var = 1
i = 0
while var == 1:
	poll = k.poll()
	print "Poll"
	
	if len(poll) > 1:
		if len(poll[1]) == 2:
			if poll[1][0] == '0xef':
				if poll[1][1] == 1 or poll[1][1] == 3:
					while i < 10:
						k.hold()
						print "Hold " + str(i)
						time.sleep(0.5)
						i += 1
			if poll[1][0] == '0xee':
				print "Credit on Channel " + str(poll[1][1])
				i = 0
				
	time.sleep(0.5)
