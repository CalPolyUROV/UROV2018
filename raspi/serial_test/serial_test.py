#python3

import serial

outbound = serial.Serial(
    port= '/dev/ttyS0',
    baudrate=9600,
    parity=serial.PARITY_NONE,   # parity is error checking, odd means the message should have an odd number of 1 bits
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,   # eight bits of information per pulse/packet
    timeout=0.1
)

while(True):
	received = (outbound.read()).decode("utf-8")
	if(received != ""):		
		print(received)
