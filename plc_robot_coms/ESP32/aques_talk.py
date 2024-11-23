import re
#import pykakasi

class AquesTalkWriter():
    def __init__(self):
        self.response = ""
        #self.kks = pykakasi.kakasi()

    def convert(self):
        #converted_data = self.kks.convert(self.response)
        converted_data = self.response
        message = ''
        for i, res in enumerate(converted_data):
            message += converted_data[i]['passport']
        return message

    def remove(self, message):
        symbols = ["(", ")", "!", ":", "・", "？", "?", "&", " "]
        for symbol in symbols:
            message = message.replace(symbol, "")
        return message

    def mathexp(self, message):
        symbols = ["+", "-", "x", "="]
        converts = ["tasu", "hiku", "kakeru", "wa"]
        for symbol, convert in zip(symbols, converts):
            message = message.replace(symbol, convert)
        return message

    def lower(self, message):
        return message.lower()

    def alphabet_reading(self, message):
        alphabets = list(set(re.findall(r"[a-zA-Z]+", self.response)))
        alphabets = [alpha.lower() for alpha in alphabets]
        for alpha in alphabets:
            message = message.replace(alpha, "<ALPHA VAL=" + alpha + ">")
        return message

    def number_reading(self, message):
        numbers = list(set(re.findall(r"\d+", self.response)))
        for num in numbers:
            message = message.replace(num, "<NUMK VAL=" + num + ">")
        return message

    def write(self):
        message = self.convert()
        message = self.remove(message)
        message = self.mathexp(message)
        message = self.lower(message)
        message = self.alphabet_reading(message)
        message = self.number_reading(message)
        return message