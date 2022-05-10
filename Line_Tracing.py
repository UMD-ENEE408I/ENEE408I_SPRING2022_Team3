import math
import matplotlib.pyplot as plt
import socket

xVal = []
yVal = []
phi = 0.0
cX = 0
cY = 0
cX_prev = 0
cY_prev = 0
m1 = 0
m2 = 0
m1Prev = 0
m2Prev = 0
n1 = 0
theta = 0
n2 = 0

s = socket.socket()

host =   "192.168.2.104" #  #put IP address "192.168.1.201"
port = 80

s.connect((host, port))
i = 0
while i < 2000:
    i = i + 1

    n1 = n1*(11/360)
    n2 = n2*(11/360)
    if n1 == n2:
        cX = cX + n1 * math.cos(phi*math.pi/180)
        cY = cY + n1 * math.sin(phi*math.pi/180)
    else:
        if n1 < n2: #left turn
            a = -1
            smlTurn = n1
            lrgTurn = n2
        else:
            a = 1
            smlTurn = n2
            lrgTurn = n1

        r = 4*a*(n1 + n2)/(n1 - n2)
        rX = cX + r * math.cos((phi-a*90) * (math.pi/180))
        rY = cY + r * math.sin((phi-a*90) * (math.pi/180))
        if r == 4:
            r = r + .001
        theta = 180*smlTurn/(2*math.pi*(r-4))
        if theta == 0:
            phi = phi - (a*lrgTurn/8) * (180/math.pi)
        else:
            phi = phi - a*theta

        cX_prev = cX
        cY_prev = cY
        cX = math.cos(theta*math.pi/180)*(cX - rX) + math.sin(theta*math.pi/180)*(cY - rY) + rX
        cY = -math.sin(theta*math.pi/180)*(cX - rX) + math.cos(theta*math.pi/180)*(cY - rY) + rY


    data = []
    while len(data) < 3:
        data.append(s.recv(1))

    n1 = int.from_bytes(data[0], "big", signed="True")
    n2 = int.from_bytes(data[1], "big", signed="True")
    print(data[2].decode("utf-8"))
    if data[2] == "R":
        phi += -90
    elif data[2] == "L":
        phi += 90
    elif data[2] == "U": #uturn if implemented
        phi += 180
    xVal.append(cX)
    yVal.append(cY)


plt.xlim(-50,50)
plt.ylim(-50,50)

plt.plot(xVal, yVal, color = 'red', marker = "o")
plt.show()


s.close()
