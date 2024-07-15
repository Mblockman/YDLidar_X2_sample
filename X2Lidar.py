import serial
import struct
import time
import math


class X2Lidar:
    SERIAL_BAUDRATE = 115200
    DEFAULT_TIMEOUT = 500

    def __init__(self):
        self._serial = None
        self.distanceList = [0] * 360
        self.lightList = [0] * 360
        self.temp = [0] * 360

    def connect(self, port):
        self._serial = serial.Serial(port, self.SERIAL_BAUDRATE, timeout=1)
        return self._serial.is_open

    def isOpen(self):
        return self._serial.is_open if self._serial else False
    
    def scanData(self, timeout=DEFAULT_TIMEOUT):
        start_time = time.time()
        read_data = bytearray()

        while (time.time() - start_time) < timeout / 1000.0:
            if self._serial.in_waiting > 0:
                temp = self._serial.read(1)
                if temp == b"\xaa" and len(read_data) == 0:
                    read_data += temp
                    continue
                if temp == b"\x55" and len(read_data) == 1:
                    read_data += temp
                    continue
                if len(read_data) == 2:
                    read_data += temp
                    continue
                if len(read_data) == 3:
                    if temp == 0:
                        read_data = bytearray()
                        continue
                    read_data += temp
                    read_data += self._serial.read(int(temp[0]) * 2 + 6)
                    break
                
        return read_data    
    
    def process_lidar_sumCheck(self, data):
        #print(f"data: {' '.join(f'{b:02x}' for b in data)}")
        if(len(data) < 10):
            #print(f"data length is insufficient: {len(data)}")
            return False
        dataSize = data[3]
        #if len(data) < dataSize * 3 + 10:
        if len(data) != dataSize * 2 + 10:
            #print(f"data length is incorrect: {len(data)} {dataSize * 3 + 10}")
            return False
        
        cs = (data[9] << 8) | data[8]
        checkSum = 0x55AA
        for i in range(0, dataSize * 2, 2):
            checkSum ^= ((data[10 + i + 1] << 8) | data[10 + i])
        checkSum ^= (data[3] << 8 | data[2])
        checkSum ^= (data[5] << 8 | data[4])
        checkSum ^= (data[7] << 8 | data[6])
        if checkSum == cs:
            return True
        else:
            return False    

    def process_lidar_data(self, data, flag=False):
        CT = data[2]

        LSN = data[3]
        FSA = (data[5] << 8 | data[4])
        LSA = (data[7] << 8 | data[6])
        row = 0
        Distance = [0] * LSN
        Intensity = [0] * LSN

        for i in range(0, 2 * LSN, 2):
            distance_data = (data[10 + i + 1] << 8 | data[10 + i])
            Distance[i // 2] = distance_data >> 2

        Angle = [0] * LSN
        Angle_FSA = (FSA >> 1) / 64
        Angle_LSA = (LSA >> 1) / 64
        Angle_Diff = (Angle_LSA - Angle_FSA)

        X_value = [0] * LSN
        Y_value = [0] * LSN
        if Angle_Diff < 0:
            Angle_Diff += 360
        for i in range(LSN):
            try:
                Angle[i] = i * Angle_Diff / (LSN - 1) + Angle_FSA
                if Distance[i] > 0:
                    AngCorrect = math.atan(21.8 * (155.3 - Distance[i]) / (155.3 * Distance[i]))
                    if flag:
                        Angle[i] += AngCorrect * 180 / math.pi
                        if Angle[i] >= 360:
                            Angle[i] -= 360
                    else:
                        X_value[i] = Distance[i] * math.cos(math.radians(Angle[i]))
                        Y_value[i] = Distance[i] * math.sin(math.radians(Angle[i]))
                        
            except Exception as e:
                #print(f"Error: {e} {i}")
                continue
            '''
            if Distance[i] > 0:
                if flag:
                    print(f"Angle: {Angle[i]}, Distance: {Distance[i]} {flag}")
                else:
                    print(f"x: {X_value[i]}, y: {Y_value[i]}")
            '''
        if flag:
            #angle = [math.radians(a) for a in Angle]
            angle_distance_pairs = [(a, d) for a, d in zip(Angle, Distance) if d > 0]
            angle, Distance = zip(*angle_distance_pairs) if angle_distance_pairs else ([], [])
            return angle, Distance        
        else:
            return X_value, Y_value        
        

if __name__ == "__main__":
    lidar = X2Lidar()
    if lidar.connect('COM4'):
        while True:
            scan_data = lidar.scanData()
            if scan_data is not None:
                if lidar.process_lidar_sumCheck(scan_data):
                    angle, distance = lidar.process_lidar_data(scan_data,True)
                    if angle is not None and distance is not None:
                        print(f"angle: {angle}, distance: {distance}")   
                else:
                    print("checksum error")     
