import serial.tools.list_ports
from pick import pick
ports = serial.tools.list_ports.comports()

# 'com_list' contains list of all com ports
com_list = []
name_list = []
options = []
for p in ports:
    print(p)
    com_list.append(p.device)
    name_list.append(p.description)

title = 'Select port'
option, index = pick(name_list, title)
print(option)
print(index)

