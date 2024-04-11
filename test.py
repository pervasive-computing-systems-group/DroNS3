#for testing
import commands
import os
import subprocess

byte_amounts = [250000, 500000, 1000000, 2000000]

def connect(d):
    for byte_amount in byte_amounts:
        c = commands.Connect(passed_vehicle=None, first=False, bytes_sent=byte_amount,distance = d)
        c.begin()
        c.connect()

os.chdir("Server_Client")
subprocess.run(["make"], shell = True)
while (True):
    distance = int(input())
    if distance == -1:
        break
    connect(distance)