#!/usr/bin/python3
import http.server
import socketserver
import client_helvar_dali

helvarNet = client_helvar_dali.HelvarNet('10.254.1.2', 50000)        
leds = [client_helvar_dali.LedUnit(helvarNet, '1.2.1.1'),
        client_helvar_dali.LedUnit(helvarNet, '1.2.1.2'),
        client_helvar_dali.LedUnit(helvarNet, '1.2.1.3'),
        client_helvar_dali.LedUnit(helvarNet, '1.2.1.4'),
        client_helvar_dali.LedUnit(helvarNet, '1.2.1.5')]
grps = [ client_helvar_dali.LedGroup(helvarNet, '1'), client_helvar_dali.LedGroup(helvarNet, '4') ]

class Handler(http.server.BaseHTTPRequestHandler):
    def __parse_url(self):
        parts = self.path.split('/')
        print(self.path)
        return {'base' : parts[1],
                'id' : int(parts[2]),
                'level' : int(parts[3]),
                'fade_time' : int(parts[4])}

    def do_HEAD(self):
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()

    def do_GET(self):
        req = self.__parse_url()
        if (req['base'] == 'lamp'):
            if (req['id'] <= len(leds)-1) and (req['level'] <= 255) :
                leds[req['id']].set_device_lvl(req['level'], req['fade_time'])
        elif (req['base'] == 'group'):
            if (req['id'] <= len(grps)-1) and (req['level'] <= 255) :
                grps[req['id']].set_group_lvl(req['level'], req['fade_time'])
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        #self.wfile.close()

if __name__ == "__main__":
    PORT = 8002
    socketserver.TCPServer.allow_reuse_address = True
    httpd = socketserver.TCPServer(("", PORT), Handler)

    print("serving at port", PORT)
    httpd.serve_forever()
