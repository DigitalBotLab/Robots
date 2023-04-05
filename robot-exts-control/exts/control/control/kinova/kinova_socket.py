# send message to Kinova Server to control the real robot

import socket

class KinovaClient():
    def __init__(self, HOST = "localhost",  PORT = 9999) -> None:
        # SOCK_DGRAM is the socket type to use for UDP sockets
        self.host = HOST
        self.port = PORT
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # send a test message to the server
        message = "Hello, server!"
        self.sock.sendto(message.encode(), (self.host, self.port))
        self.sock.settimeout(10)
        
        # wait for a response from the server
        data, addr = self.sock.recvfrom(1024)

        print("Socket Server and Client initialized")
        # # check if the response is correct
        # if data.decode() == "Hello, client!":
        #     print("Connected to UDPServer")
        # else:
        #     print("Failed to connect to UDPServer")

    def send_message(self, message: str):
        print("Sent:     {}".format(message))
        self.sock.sendto(bytes(message + "\n", "utf-8"), (self.host, self.port))
        received = str(self.sock.recv(1024), "utf-8")
        print("received: {}".format(received))

        return received