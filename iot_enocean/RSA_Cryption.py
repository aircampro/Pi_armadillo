#
# RSA encryption / decryption
#
from math import gcd
import sys

def lcm(p, q):
  '''
  calculates lcm
  '''
  return (p * q) // gcd(p, q)

def generate_keys(p, q):
  '''
  generates the 2 keys from 2 integers
  '''
  N = p * q
  L = lcm(p - 1, q - 1)

  for i in range(2, L):
    if gcd(i, L) == 1:
      E = i
      break

  for i in range(2, L):
    if (E * i) % L == 1:
      D = i
      break

  return (E, N), (D, N)

def encrypt(plain_text, public_key):
  '''
   public_key  plain_text 
  '''
  E, N = public_key
  plain_integers = [ord(char) for char in plain_text]
  encrypted_integers = [pow(i, E, N) for i in plain_integers]
  encrypted_text = ''.join(chr(i) for i in encrypted_integers)

  return encrypted_text

def decrypt(encrypted_text, private_key):
  '''
  private_key  encrypted_text 
  '''
  D, N = private_key
  encrypted_integers = [ord(char) for char in encrypted_text]
  decrypted_intergers = [pow(i, D, N) for i in encrypted_integers]
  decrypted_text = ''.join(chr(i) for i in decrypted_intergers)

  return decrypted_text

def sanitize(encrypted_text):
  '''
  UnicodeEncodeError 
  '''
  return encrypted_text.encode('utf-8', 'replace').decode('utf-8')

if __name__ == '__main__':
  public_key, private_key = generate_keys(101, 3259)

  if len(sys.argv) == 1:                                  # no argument passed 
      plain_text = 'This is the message you are encrypting... ! '
  else:
      plain_text = str(sys.argv[1])
      
  encrypted_text = encrypt(plain_text, public_key)
  decrypted_text = decrypt(encrypted_text, private_key)

  print(f'''
public key: {public_key}
private key: {private_key}

plain text:
「{plain_text}」

encrypted:
「{sanitize(encrypted_text)}」

decrypted:
「{decrypted_text}」
'''[1:-1])