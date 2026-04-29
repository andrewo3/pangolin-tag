import struct
from sys import argv

filename = argv[1]
print(filename)

M_IMU = 5

with open(filename,"rb") as f:
    data = f.read()
    i = 0
    if data[0] != ord('i'):
        print("Invalid data!")
        f.close()
        quit()
    i+=1
    while i < len(data):
        print(data[i])
        if len(data)-i >= 5:
            if data[i:i+5] == b"FLUSH":
                print("MSG: FLUSH")
                i += 5
                continue
        if data[i] == 5:
            i += 1
            length = struct.unpack("H",data[i:i+2])[0]
            i+=2
            payload = data[i:i+length]
            print("MSG: IMU")
            print(f"length: {length}")
            print("Payload:",list(payload))
            i+=length
        elif data[i] == 6:
            i += 1
            timestamp = struct.unpack("i",data[i:i+4])[0]
            print("MSG: TIME")
            print("Timestamp: ",timestamp)
            i += 4
        else:
            quit()
