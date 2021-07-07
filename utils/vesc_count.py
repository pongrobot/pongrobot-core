import random
import os
import glob
import serial
import serial.tools.list_ports
import sys

portId = sys.argv[1]

# Output an ID for a new vesc
# Use pyserial to enumerate the ports and assign an ID
serialPorts = serial.tools.list_ports.comports()
vescs = []
for port in serialPorts:
	if port.description == "ChibiOS/RT Virtual COM Port":
		vescs.append(port)

vescs.sort(key=lambda x: x.name, reverse=False)
for i, vesc in enumerate(vescs):
	if vesc.name == portId:
		print("vesc" + str(i))
