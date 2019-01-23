import Orion5
import time
from General import ComQuery


'''
Author: Jenna Riseley November 2017

A test that instantiates the orion5 class. It must be the version that I modified
in 2017 to include the functions in Orion5.m.

Using IDLE, further commands can then
be issued to the orion5.

For example:

$ sudo idle3 testScript2.py

Then in idle, after the orion5 has connected:

>>> orion.setAllJointsPosition(pos1)
>>> orion.setAllJointsPosition(pos2)

See Orion5.py (the version modified/ extended by myself in 2017) for further commands. 
Any command found in Orion5.m can be issued to the modified version of Orion5.

'''

# Find orion5

NUM_JOINTS=5

comport = None
print('\nSearching for Orion5...')
try:
    while True:
        comport = ComQuery()
        if comport is not None:
            print('Found Orion5, serial port name:', comport.device)
            break
        time.sleep(2)
except KeyboardInterrupt:
    print('\nExiting...\n')
    quit()

orion = Orion5.Orion5(comport.device)

time.sleep(5)

for i in range(0,NUM_JOINTS-1):
        orion.setJointControlMode(i,orion.POS_SPEED)
        orion.setJointSpeed(i,80)

pos1 = [180, 230, 300, 80, 30]
pos2 = [270, 150, 300, 80, 50]



time.sleep(2)

print(orion.getAllJointsPosition())
