#!/usr/bin/env python3
# encrypt / decrypt messages before you send via morse
#
import sys
from sys import argv

def vigenere_encrypt(plaintext, key):
    """Vigenere Cipher Encrypt

    key are only UPPERCASE ALPHABET[A-Z]."""
    count = 0
    ciphertext = ''
    key = key.upper()
    for s in plaintext:
        if s.isupper():
            ciphertext += chr(((ord(s) + ord(key[count % len(key)])) % 26) + ord('A'))
        elif s.islower():
            ciphertext += chr(((ord(s) + ord(key[count % len(key)]) - (ord('a') - ord('A'))) % 26) + ord('a'))
        else:
            ciphertext += s
        count += 1
    return ciphertext

def vigenere_decrypt(ciphertext, key):
    """Vigenere Cipher Decrypt

    key are only UPPERCASE ALPHABET[A-Z]."""
    count = 0
    plaintext = ''
    key = key.upper()
    for s in ciphertext:
        if s.isupper():
            plaintext += chr(((ord(s) - ord(key[count % len(key)])) % 26) + ord('A'))
        elif s.islower():
            plaintext += chr(((ord(s) - ord(key[count % len(key)]) - (ord('a') - ord('A'))) % 26) + ord('a'))
        else:
            plaintext += s
        count += 1
    return plaintext


def main(textm):
    plaintext = textm
    key = 'MY_SECRET_KEY'
    ciphertext = vigenere_encrypt(plaintext, key)
    print(ciphertext)
    print(vigenere_decrypt(ciphertext, key))

if __name__ == '__main__':

    argc = len(sys.argv)
    if argc >= 2:
        main(argv[1])