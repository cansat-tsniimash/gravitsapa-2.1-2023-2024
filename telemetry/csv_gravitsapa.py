import struct


FILEPATH = "gravitsapa_gcs-.bin"
CSV_ORIENT_FILEPATH = FILEPATH + '-orient' + '.csv'
CSV_STATEPHOTOREZ_FILEPATH = FILEPATH + '-state&find' + '.csv'
CSV_GPS_FILEPATH = FILEPATH + '-gps' + '.csv'


# Формат лога:
# 4 байта, флоат - время
# 1 байт, беззнаковое целое - размер пакета
# пакет длинной с указанным размером


stream = open(FILEPATH, mode="rb")

def crc16(data : bytearray, offset=0, length=-1):
    if length < 0:
        length = len(data)
    
    if data is None or offset < 0 or offset > len(data)- 1 and offset+length > len(data):
        return 0

    crc = 0xFFFF
    for i in range(0, length):
        crc ^= data[offset + i] << 8
        for j in range(0, 8):
            if (crc & 0x8000) > 0:
                crc =(crc << 1) ^ 0x1021
            else:
                crc = crc << 1
        crc = crc & 0xFFFF

    return crc & 0xFFFF


def read_packet(stream):
	packet_size_raw = stream.read(1)
	if not packet_size_raw:
		return None

	packet_size, = struct.unpack(">B", packet_size_raw)

	packet = stream.read(packet_size)

	time_raw = stream.read(8)
	time, = struct.unpack("<d", time_raw)

	return time, packet





class OrientParser:
	pack_len = 26
	def __init__(self):
		self.csv_orient = open(CSV_ORIENT_FILEPATH, "w")
		line_name = '%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s' % ("flag", "num" , "time_s" ,"acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z", "mag_x", "mag_y", "mag_z", "crc")
		print(line_name, file = self.csv_orient)

	def parse(self, data: bytes):
		unpacked = struct.unpack("<BHI9hb", data[:self.pack_len])

		flag = unpacked[0]
		num = unpacked[1]
		time_s = unpacked[2]
		acc_x = unpacked[3] 
		acc_y = unpacked[4] 
		acc_z = unpacked[5] 
		gyro_x = unpacked[6] 
		gyro_y = unpacked[7]
		gyro_z = unpacked[8] 
		mag_x = unpacked[9] 
		mag_y = unpacked[10]
		mag_z = unpacked[11]
		
		crc = unpacked[12]

		if (crc != crc16(data, length=(self.pack_len-2))):
			return -1

		line_orient = "%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s" % (flag, num, time_s ,acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, crc)
		print(line_orient, file = self.csv_orient)
		print(line_orient)
		return 0
		
class StateNFind:
	pack_len = 20
	def __init__(self):
		self.csv_bme = open(CSV_STATEPHOTOREZ_FILEPATH, "w")
		line_name = "%s;%s;%s;%s;%s;%s;%s;%s;%s" % ("flag", "num" , "time_s" , "temp_bmp", "bmp_pres", "lux", "ds_temp", "state", "crc")
		print(line_name, file = self.csv_bme)
	def parse(self, data: bytes):
		unpacked = struct.unpack("<BHIHIHhHb", data[:self.pack_len])

		flag = unpacked[0]		
		num = unpacked[1]
		time_s = unpacked[2]
		temp_bmp = unpacked[3]
		bmp_pres = unpacked[4]
		lux = unpacked[5]
		ds_temp = unpacked[6]
		state = unpacked[7]
		crc = unpacked[8]

		if (crc != crc16(data, length=(self.pack_len-2))):
			return -1


		line_bme280 = "%s;%s;%s;%s;%s;%s;%s;%s;%s" % (flag, num , time_s , temp_bmp, bmp_pres, lux, ds_temp, state, crc)
		print(line_bme280, file = self.csv_bme)
		print(line_bme280)
		return 0

class GpsParser:
	pack_len = 28
	def __init__(self):
		self.csv_dosim = open(CSV_GPS_FILEPATH, "w")
		line_name = '%s;%s;%s;%s;%s;%s;%s;%s' % ("flag", "num", "time_s", "time_us", "lat", "lon",  "fix" , "crc")
		print(line_name, file = self.csv_dosim)
	def parse(self, data: bytes):
		unpacked = struct.unpack("<BH3I3fb", data[:self.pack_len])	

		flag = unpacked[0]
		num = unpacked[1]
		time_s = unpacked[2]
		time_us = unpacked[3]
		
		lat = unpacked[4]
		lon = unpacked[5]
		fix = unpacked[6]
		crc = unpacked[7]

		if (crc != crc16(data, length=(self.pack_len-2))):
			return -1


		line_dosim = "%s;%s;%s;%s;%s;%s;%s;%s" % (flag, num, time_s, time_us, lat, lon,  fix , crc)
		print(line_dosim, file = self.csv_dosim)
		print(line_dosim)
		return 0




orient = OrientParser()
stateNfind = StateNFind()
gps = GpsParser()


buf = []

while True:
	#data = read_packet(stream)
	buf.extend(list(stream.read(1)))
	if len(buf) < 1:
		break

	#time, packet = data
	flag = buf[0]
	#print(f"flag={flag}, data={packet}, len(data)={len(packet)}")

	try:
		if flag == 0x30:
			buf.extend(list(stream.read(orient.pack_len - 1)))
			if orient.parse(bytes(buf)) != 0:
				buf = buf[1:]
			else:
				buf = buf[orient.pack_len:]


		elif flag == 0x01:
			buf.extend(list(stream.read(stateNfind.pack_len - 1)))
			if stateNfind.parse(bytes(buf)) != 0:
				buf = buf[1:]
			else:
				buf = buf[stateNfind.pack_len:]


		elif flag == 0x20:
			buf.extend(list(stream.read(gps.pack_len - 1)))
			if gps.parse(bytes(buf)) != 0:
				buf = buf[1:]
			else:
				buf = buf[gps.pack_len:]


		else:
			print("НЕИЗВЕСТНЫЙ ФЛАГ %d" % int(flag))
			buf = buf[1:]
			print(buf)
			
			
	except Exception as e:
		raise e
		print("НЕ МОГУ РАЗОБРАТЬ ПАКЕТ С ФЛАГОМ %x: %s" % (int(flag), e))


	# unpacked_bme = struct.unpack("<B", packet[:117])
	# unpacked_dosim = struct.unpack("<B", packet[:99])
	# unpacked_gps = struct.unpack("<B", packet[:66])
	# unpacked_state = struct.unpack("<B", packet[:71])

	#print(unpacked)
	#print("readed bytes %s of data %s at time %s" % (len(packet), packet, time))

#print("flag;BMP_temperature;LSM_acc_x;LSM_acc_y;LSM_acc_z;LSM_gyro_x;LSM_gyro_y;LSM_gyro_z;num;time_from_start;BMP_pressure;crc;time", file=csv_stream)