from socket import socket, gethostbyname, AF_INET, SOCK_DGRAM, SOL_SOCKET, SO_REUSEADDR

PORT_NUMBER = 22
SIZE = 1024

# hostName = gethostbyname( '0.0.0.0' )

mySocket = socket( AF_INET, SOCK_DGRAM)
mySocket.bind( ('', PORT_NUMBER) )

while True:
        try:
                (data,addr) = mySocket.recvfrom(SIZE)
                print(data.decode())
        except:
                mySocket.close()