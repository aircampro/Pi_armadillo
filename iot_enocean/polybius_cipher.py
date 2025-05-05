# Polybius' cipher table.
# A pair of numbers from 1 to 5 represents one alphabet
#

# convert from cipher to alphanumeric
TABLE = {
    '1': {'1': 'A', '2': 'B', '3': 'C', '4': 'D', '5': 'E'},
    '2': {'1': 'F', '2': 'G', '3': 'H', '4': '(I,J)', '5': 'K'},
    '3': {'1': 'L', '2': 'M', '3': 'N', '4': 'O', '5': 'P'},
    '4': {'1': 'Q', '2': 'R', '3': 'S', '4': 'T', '5': 'U'},
    '5': {'1': 'V', '2': 'W', '3': 'X', '4': 'Y', '5': 'Z'},
}

# convert from alphanumeric to cipher number
RTABLE = { 'A' : '11', 'B' : '12', 'C' : '13', 'D' : '14', 'E' : '15',
    'F' : '21', 'G' : '22', 'H' : '23', 'I' : '24', 'J' : '24', 'K' : '25', 
    'L' : '31', 'M' : '32', 'N' : '33', 'O' : '34', 'P' : '35',
    'Q' : '41', 'R' : '42', 'S' : '43', 'T' : '44', 'U' : '45',	
    'V' : '51', 'W' : '52', 'X' : '53', 'Y' : '54', 'Z' : '55',	
}

# convert the string to the cipher
def conv_polybius(text):
    ret = ''
    for i in range(0, len(text), 2):
       ret += TABLE[text[i]][text[i+1]]
    return ret

# make the cipher from the string
def make_polybius(text):
    ret = ''
    for i in text:
       ret += RTABLE[i]
    return ret
	
if __name__ == '__main__':
    text = "11421315"                       # for example ARCE send as in table above 11=A 42=R etc
    ans = conv_polybius(text)
    print(ans)
    cip = make_polybius(ans)
    print(cip)
    cip = make_polybius("MARKNICHOLAS")
    print(cip)
    ans = conv_polybius(cip)
    print(ans)