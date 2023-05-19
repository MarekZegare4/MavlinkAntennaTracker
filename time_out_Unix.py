from pymavlink import mavutil
import signal
import time
from tqdm import tqdm

timeOut = 10

def handler(signum, frame):
    raise Exception("end of time")

def loop_forever():
    #signal.alarm(0) jak coś zadziała
    for i in tqdm(range(timeOut), desc="Time-out"):
        time.sleep(1)

signal.signal(signal.SIGALRM, handler)
signal.alarm(timeOut)
try:
    loop_forever()
except Exception: 
    print("Connection time-out") 

