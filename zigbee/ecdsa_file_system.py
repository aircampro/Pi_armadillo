# sets the ECDSA key for the remote zigbee file system
#
#  pip install ecdsa
import xbee

from ecdsa import SigningKey
from ecdsa import VerifyingKey
# SigningKeyVerifyingKey choice SECP256k1 SECP256k1 SECP256k1 NIST192p BRAINPOOLP160r1
from ecdsa import SECP256k1             

# ［FK (file system public key) <XBee3
# Sets the file system public key for the device.
# The 65-byte public key is required to ensure that the file system downloaded wirelessly is a valid XBee3 file system compatible with the Zigbee firmware.
# compatible with the Zigbee firmware.
# For more information, see Setting the Public Key on the XBee3 Device.
# Parameter range
# A valid 65-byte ECDSA public key
# Other accepted parameters:
#　0 Clear public key
#　1 Returns the upper 48 bytes of the public key
#　2 Returns the lower 17 bytes of the public key
# Default
#　0
# (Note: The default value of 0 means that no public key is set,
#　 Note: The default value of 0 indicates that all file system updates will be rejected because no public key has been set.

def set_ecdsa_key():
    secret_key = SigningKey.generate(curve=SECP256k1)                   # SigningKeyVerifyingKey choice SECP256k1 SECP256k1 SECP256k1 NIST192p BRAINPOOLP160r1
    print("generated secret key = "+secret_key)
    public_key = secret_key.verifying_key
    print("generated public key = "+public_key)
	pk=int(public_key.to_string().hex(), 16)                            # convert it to a hexidecimal string
    ecdsa_settings = {"aFK": 0, "aaFK": pk}                             # clear the key then set it to the one we generated
    ll=0
    for command, value in ecdsa_settings.items():
        ss=command.split('a')
        try:
            xbee.atcmd(ss[ll+1], value)
        except:
            print("failed to set file system public key")
        ll=ll+1
    ecdsa_get = {"aFK": 1, "aaFK": 2}                                    # read the key set in 2 parts then re-assemble it
    sso={}
    ll=0
    for command, value in ecdsa_get.items():
        ss=command.split('a')
        try:
            sso[int(ll)] = xbee.atcmd(ss[ll+1], value)
        except:
            print("failed to get file system public key")			
        ll=ll+1
    ll=0
    ecdsa_num=0
    for key, value in sso.items():
        if (key == 0):                                                 # Returned the upper 48 bytes of the public key (remove base 16 if not in hex)
            ecdsa_num = (int(value,16) << 17)                             
        elif (key == 1):                                               # Returned the lower 17 bytes of the public key (remove base 16 if not in hex)
            ecdsa_num = ecdsa_num | int(value,16)	                       
    if (ecdsa_num == pk):
        print("file system public key set to "+pk)
    else:
        print("file system public key faileds to set")	
		
# run to set a new ECDSA key
set_ecdsa_key()