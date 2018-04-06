import socket
import sys
import time

ip = '10.0.18.111'
# Bind the socket to the port
server_address = (ip, 6791)

def connect():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    while(True):
        try:
            sock.connect(server_address)
            break
        except socket.error:
            pass
    return sock

while(True):
    try:
        print("Connecting to RFFE@"+ip)
        s = connect()
        s.send("Test msg!\0")
        s.close()
        print("Message sent successfuly to RFFE@"+ip)
        time.sleep(2)
    except socket.error:
        pass

