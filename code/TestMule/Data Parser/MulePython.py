import struct
import os

# C Struct Definition:
#   uint8_t dataVersion = 1;
#   uint32_t time;
#   uint16_t throttle;
#   int16_t left;
#   int16_t right;
#   float steer;
#   uint16_t speed;

keys = ("Version","Millis","Throttle","Left","Right","Steer","leftRPM","rightRPM")
structFormat = "<BIHhhfHH"
dataSize = 512//struct.calcsize(structFormat)*struct.calcsize(structFormat)

for filename in os.listdir():
    if ".mule" in filename:
        dataEntries = []
        fileSize = os.stat(filename).st_size
        with open(filename, "rb") as inFile:
            for x in range(fileSize//512):
                for entry in struct.iter_unpack(structFormat, inFile.read(dataSize)):
                    if entry[0] == 1:
                        dataEntries.append(entry)
                inFile.read(512-dataSize)

        import csv
        with open(filename.replace(".mule",".csv"),'w', newline='') as csvfile:
            csvWriter = csv.writer(csvfile, dialect='excel')
            csvWriter.writerow(keys)
            for entry in dataEntries:
                csvWriter.writerow(entry)