import socket               

sock = socket.socket()

host = "192.168.1.33" #ESP32 IP in local network
port = 80             #ESP32 Server Port    

sock.connect((host, port))

message = b'Hello World'  #for steven on python 2 this was a string "hello world" 
sock.send(message)

data = []       

while len(data) < len(message):
    data += sock.recv(1)

print(data) #this prints out an array of bytes we would have to decode back. could also just all use python 2 since that seemed to allow strings

sock.close()
