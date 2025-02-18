#!/usr/bin/env python
#
# Example of raw http communication on a socket
#
import socket

class httpResponse:
    def __init__(self):
        self.responce_code = None
        self.body = None
        self.content_length = -1

class ResponseBuffer:
    def __init__(self):
        self.data = b''

    def append(self, data):
        self.data += data

    def read_line(self):
        if self.data == b'':
            return None

        end_index = self.data.find(b'\r\n')
        if end_index == -1:
            ret = self.data
            self.data = b''
        else:
            ret = self.data[:end_index]
            self.data = self.data[end_index + len(b'\r\n'):]
        return ret.decode('utf-8')

    def get_body(self):
        body_index = self.data.find(b'\r\n\r\n')
        if body_index == -1:
            return None
        else:
            return self.data[body_index + len(b'\r\n\r\n'):]
            
class httpClient:

    def request(host, port=80):
        response = httpResponse()

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((request.host, request.port))
        request_str = 'GET / HTTP/1.1\nHost: %s\r\n\r\n' % (host)
        s.send(request_str.encode('utf-8'))
        s.close()
        return response
		
    def request2(host, port=80):
        response = httpResponse()

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((request.host, request.port))
        request_str = 'GET / HTTP/1.1\nHost: %s\r\n\r\n' % (host)
        s.send(request_str.encode('utf-8'))

        s.send(request_str.encode('utf-8'))

        headerbuffer = ResponseBuffer()
        allbuffer = ResponseBuffer()
        while True:
            chunk = s.recv(4096)
            allbuffer.append(chunk)

            if response.content_length == -1:
                headerbuffer.append(chunk)
                response.content_length = httpClient.parse_contentlength(headerbuffer)

            else:
                if len(allbuffer.get_body()) >= response.content_length:
                    break

        response.body = allbuffer.get_body()
        response.responce_code = 200

        s.close()
        return response

    def parse_contentlength(buffer):
        while True:
            line = buffer.read_line()
            if line.startswith('Content-Length'):
                return int(line.replace('Content-Length: ', ''))
            if line == None:
                return -1


		
		
		
		