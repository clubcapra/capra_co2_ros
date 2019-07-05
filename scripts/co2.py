#!/usr/bin/python
import  array, time, io, fcntl
import rospy
from std_msgs.msg import String

bus = 1
addressT6713 = 0x15
I2C_SLAVE=0x0703

class i2c(object):
	def __init__(self, device, bus):

		self.fr = io.open("/dev/i2c-"+str(bus), "rb", buffering=0)
		self.fw = io.open("/dev/i2c-"+str(bus), "wb", buffering=0)

		# set device address

		fcntl.ioctl(self.fr, I2C_SLAVE, device)
		fcntl.ioctl(self.fw, I2C_SLAVE, device)

	def write(self, bytes):
		self.fw.write(bytes)

	def read(self, bytes):
		return self.fr.read(bytes)

	def close(self):
		self.fw.close()
		self.fr.close()

class T6713(object):
	def __init__(self):
		self.dev = i2c(addressT6713, bus)

	def status(self):
		buffer = array.array('B', [0x04, 0x13, 0x8a, 0x00, 0x01])
		self.dev.write(buffer)
		time.sleep(0.1)
		data = self.dev.read()
		buffer = array.array('B', data)
		return buffer[2]*256+buffer[3]

	def gasPPM(self):
		buffer = array.array('B', [0x04, 0x13, 0x8b, 0x00, 0x01])
		self.dev.write(buffer)
		time.sleep(0.1)
		data = self.dev.read(4)
		buffer = array.array('B', data)
		print(buffer)
		#print("buffer value : %X,%X,%X,%X,%X,%X,%X,",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7])
		return buffer[2]*256+buffer[3]

	def checkABC(self):
		buffer = array.array('B', [0x04, 0x03, 0xee, 0x00, 0x01])
		self.dev.write(buffer)
		time.sleep(0.1)
		data = self.dev.read(4)
		buffer = array.array('B', data)
		return buffer[2]*256+buffer[3]

	def calibrate(self):
		buffer = array.array('B', [0x05, 0x03, 0xec, 0xff, 0x00])
		self.dev.write(buffer)
		time.sleep(0.1)
		data = self.dev.read(5)
		buffer = array.array('B', data)
		return buffer[3]*256+buffer[3]
              

def measureGasConcentration():
	pub = rospy.Publisher('ppm', String, queue_size=1)
	rospy.init_node('co2', anonymous=True)
	rate = rospy.Rate(10) #10Hz
    while not rospy.is_shutdown():
        obj = T6713()
		pub.publish(obj.gasPPM())
		rate.sleep()
#	print "Status: ", bin(obj.status())
	# print "PPM: ", obj.gasPPM()
#	print "ABC State: ", obj.checkABC()
#        time.sleep(1)
    
if __name__ == "__main__":
	try:
		measureGasConcentration()
	except rospy.ROSInterruptException:
		pass
