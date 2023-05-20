#!/usr/bin/env python

'''
test mavlink messages
'''
from __future__ import print_function

from pymavlink import mavutil

baudRate = 9600

def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    msg = m.recv_match(type='HEARTBEAT', blocking=True)
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_component))

# create a mavlink serial instance
def connect(baudRate):
    serial_connections = mavutil.auto_detect_serial()
    if len(serial_connections) == 0:
        print('No serial devices connected')
        return 0
    else:
        print(serial_connections)
        device = serial_connections[0]
        master = mavutil.mavlink_connection(device, baud=baudRate)
        wait_heartbeat(master)
        print("Sending all message types")
        return 1

connect(baudRate)

