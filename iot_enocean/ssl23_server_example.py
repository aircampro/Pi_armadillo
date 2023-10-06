# SSL 23 example
import socketserver,ssl
 
# SSL to create context and cert file
context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
context.load_cert_chain(certfile="cert.pem", keyfile="key.pem")
 
class MyTCPHandler(socketserver.BaseRequestHandler):
    """
    handle TCP requests
    """
    def handle(self):
        # 
        srequest = context.wrap_socket(self.request, server_side=True)
 
        self.data = srequest.recv(1024).strip()
        print("{} wrote:".format(self.client_address[0]))
        print(self.data)
        # 
        srequest.sendall(self.data.upper())
 
if __name__ == "__main__":
    HOST, PORT = "127.0.0.1", 12345
 
    # localhost specified above
    with socketserver.TCPServer((HOST, PORT), MyTCPHandler) as server:
        # Ctrl-C
        server.serve_forever()