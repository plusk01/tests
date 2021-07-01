#!/usr/bin/env python3
import serial

ser = serial.Serial('/dev/ttyACM0')

a = 8.33
b = -0.3324

#
# Example 1 - more robust because we wait for the '\n' sentinel
#

out = str(a) + ',' + str(b) + '\n'
ser.write(out.encode())

#
# Example 2 - more compact, but less robust because no sentinel byte
#

# import struct
# out = struct.pack('ff',a,b)
# ser.write(out)