import socket
import asyncore


'''''''''''''''''''''''''''''
    Telemetry handler
'''''''''''''''''''''''''''
telemetry_client_connected = False

class telemetry_handler(asyncore.dispatcher_with_send):
  
    def handle_read(self):
        pass

    def handle_close(self):
        global telemetry_client_connected
        print('Telemetry> connection closed.')
        telemetry_client_connected = False
        self.close()

'''''''''''''''''''''''
    Telemetry server
'''''''''''''''''''''
class telemetry_server(asyncore.dispatcher):
    
    def __init__(self, host, port):
        global telemetry_client_connected
        telemetry_client_connected = False
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.set_reuse_addr()
        self.bind((host, port))
        self.listen(5)
    
    def handle_accept(self):
        global telemetry_client_connected
        pair = self.accept()
        if pair is not None:
            sock, addr = pair
            print('Telemetry> incoming connection from ', repr(addr))
            self.handler = telemetry_handler(sock)
            telemetry_client_connected = True

    def sendTelemetry(self, data):
        global telemetry_client_connected
        if telemetry_client_connected:
            self.handler.send(data.encode("utf-8"))
            #self.flush()
