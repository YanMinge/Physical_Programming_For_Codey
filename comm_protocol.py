# -- coding: utf-8 --
#!/usr/bin/env python
import RPi.GPIO
import threading
import time
import sys
import serial
from threading import Timer
import random
import os
import codecs, binascii
import cv2
import numpy as np
from xyimage import XYImage
import glob
import stat
import serial.tools.list_ports
import pygame

READ_FILE_NAME = "main.py"
FTP_HEADER_BYTE = "F3005E01"
FTP_HEADER = binascii.a2b_hex(FTP_HEADER_BYTE)
FTP_END_BYTE = "F3005E01"
FTP_END = binascii.a2b_hex(FTP_END_BYTE)

STORE_FILE_NAME = "/flash/main.py"
FILE_BLOCK_SIZE = 160

frame_header_str = "F3"
frame_end_str = "F4"
protocol_id_str = "01"
dev_id_str = "00"
srv_id_str = "5E"
file_header_cmd_id_str = "01"
file_block_cmd_id_str = "02"
file_state_cmd_id_str = "F0"
file_type_str = "00"

FRAME_HEAD = 0xF3
FRAME_END = 0xF4
DEV_ID = 0x00
SRV_ID = 0x5E
CMD_STATE_ID = 0xF0

FTP_FSM_HEAD_S       = 0
FTP_FSM_HEAD_CHECK_S = 1
FTP_FSM_LEN1_S       = 2
FTP_FSM_LEN2_S       = 3
FTP_FSM_DATA_S       = 4
FTP_FSM_CHECK_S      = 5
FTP_FSM_END_S        = 6

BUTTON_START                    = 1
IMAGE_PROCESS_CALIBRATE         = 2
IMAGE_PROCESS_WHITEBALANCE      = 3
IMAGE_PROCESS_HSV               = 4
IMAGE_PROCESS_GRAY              = 5
IMAGE_PROCESS_THRESHOLD         = 6
IMAGE_PROCESS_CROP_GROUPS       = 7
IMAGE_PROCESS_CROP_BLOCKS       = 8
IMAGE_PROCESS_CREATE_PYTHON     = 9
IMAGE_PROCESS_DONE              = 10
SEND_FILE                       = 11
TEST_STEP                       = 12
PROCESS_DONE                    = 13

timeCount = 0
isResponse = False
isButtonPressed = False
isWindowsInit = False
ProcessState = PROCESS_DONE
#ProcessState = SEND_FILE
stateCount = 0

clicked = False

def onMouse(event,x,y,flags,param):
    global clicked
    if event == cv2.EVENT_LBUTTONUP:
        clicked = True

# ------------------------ start ----------------------------------------------------------
port_list = list(serial.tools.list_ports.comports())

if len(port_list) <= 0:
	print "None port find!!"
else:
	port_list_0 = list(port_list[0])
	port_serial = port_list_0[0]
	print "port_serial:" + port_serial
	ser = serial.Serial( port_serial, 115200 )

# These function input a int variable and return if it can be display
def is_display( c ):
	if ( 0x20 <= c and c <= 0x7E ):
		return True
	else:
		return False

def print_bytes_hex( bytes_data ):
	#print( ":".join( "{:02x}".format(c) for c in bytes_data ) )
	hex_data = str(binascii.b2a_hex(bytes_data))
	a = ''
	for num in range(0,len(hex_data)/2):
		a = a + hex_data[num*2]+hex_data[num*2+1]+' '
	print(a)

def bytes_to_hex_str( bytes_data ):
	#return " ".join( "{:02x}".format(c) for c in bytes_data )
	hex_data = str(binascii.b2a_hex(bytes_data))
	a = ''
	for num in range(0,len(hex_data)/2):
		a = a + hex_data[num*2]+hex_data[num*2+1]+' '
	return a

# data is bytesarray
# return is int
def calc_xor( data ):
	ret = 0
	for c in data:
		ret = ret ^ c
	return ret

def calc_add_checksum( data ):
	ret = 0
	for c in data:
		ret = ret + ord(c)
	return ret & 0xFF

def calc_32bit_xor( data ):
	bytes_len = len( data )
	#data_bytes = bytes( data, encoding = 'utf-8' )
	data_bytes = bytes( data )
	# print_bytes_hex( data_bytes )
	# print( bytes_len/4 )
	# print( int(bytes_len/4) )
	checksum = bytearray.fromhex( "00 00 00 00" )
	for i in range(int(bytes_len/4)):
		checksum[0] = checksum[0] ^ ord(data_bytes[i*4 + 0])
		checksum[1] = checksum[1] ^ ord(data_bytes[i*4 + 1])
		checksum[2] = checksum[2] ^ ord(data_bytes[i*4 + 2])
		checksum[3] = checksum[3] ^ ord(data_bytes[i*4 + 3])

	if ( bytes_len%4 ):
		for i in range( bytes_len%4 ):
			checksum[0+i] = checksum[0+i] ^ ord(data_bytes[4*int(bytes_len/4) + i])

	#print_bytes_hex( checksum )
	return checksum

def get_file_len( file_name ):
	# read file 
	# f = open( file_name, 'r' )
	# f_d = f.read()
	# f_len = len(f_d)	
	# f.close()
	return os.path.getsize( "./file_name" )
	
def hex2byte(h):
	h = h[2:]
	a = ''
	for num in range(0,len(h)/2):
		a = a + h[num*2]+h[num*2+1]+' '
	return a

def int2hex(input,length,byteorder):
 	a = ''

	h = hex(input)[2:]
	if len(h) % 2:
		h = '0' + h
	if byteorder == 'little':
		h = h.decode('hex')[::-1].encode('hex')
	elif byteorder == 'large':
		h = h
	if (len(h) + 1)/2 >= length:
		return '0x' + h

	if length > (len(h) + 1)/2:
		for num in range(0,(length - (len(h)+1)/2)):
			a += '00'
	if byteorder == 'little':
		h = h + a
	elif byteorder == 'large':
		h = a + h
	return '0x' + h

def int2byte(input,length,byteorder):
	h = int2hex(input,length,byteorder)
	return hex2byte(h)

def send_file( ser, file_name, file_type ):
	global ProcessState
	global timeCount
	timeCount = 0
	global isResponse
	resend_count = 0
	# read file 
	f = open( file_name, 'rb' )
	f_d = f.read()
	f_len = len(f_d)
	# send file header
	while( True ):
		#cmd_len_str = bytes_to_hex_str( (0x09 + len(STORE_FILE_NAME)).to_bytes( 2, byteorder='little' ) )
		cmd_len_str = int2byte(0x09 + len(STORE_FILE_NAME), 2, 'little')
		#file_size_str = bytes_to_hex_str( f_len.to_bytes(4, byteorder='little') )
		file_size_str = int2byte(f_len, 4, 'little')
		file_checksum_str = bytes_to_hex_str( calc_32bit_xor( f_d ) )
		file_name_str = bytes_to_hex_str( bytes( STORE_FILE_NAME))
		frame_data_str = protocol_id_str 
		frame_data_str += " " + dev_id_str + " " + srv_id_str 
		frame_data_str += " " + file_header_cmd_id_str + " " + cmd_len_str + " " 
		frame_data_str += file_type_str + " " + file_size_str + " " + file_checksum_str 
		frame_data_str += " " + file_name_str;
		frame_data_str = frame_data_str.replace(' ','')
		frame_data_len = len(binascii.a2b_hex(frame_data_str))
		frame_data_len_str = int2byte(frame_data_len,2,'little' )
		frame_data_len_str = frame_data_len_str.replace(' ','')
		frame_head_checkusum_str = int2byte( calc_add_checksum(binascii.a2b_hex(frame_header_str+frame_data_len_str)),1,'little' )
		frame_checksum_str = int2byte( calc_add_checksum(binascii.a2b_hex(frame_data_str)),1,'little' )
		
		send_head_str = frame_header_str + " " + frame_head_checkusum_str + " " + frame_data_len_str + " " + frame_data_str + " " + frame_checksum_str + " " + frame_end_str
		send_head_str = send_head_str.replace(' ','')
		#print( send_head_str )
		
		ser.write(binascii.a2b_hex( send_head_str) )
		# wait for respond
		timeCount = 0
		isResponse = False
		while ( isResponse == False ):
			if(ProcessState != PROCESS_DONE):
				if(timeCount > 10):
					resend_count = resend_count+1
					timeCount = 0
					ser.write(binascii.a2b_hex( send_head_str) )
					print( "Resend file header:",resend_count)

				if(resend_count > 2):
					ProcessState = PROCESS_DONE
					print( "Send file header Time Out" )
					f.close()
					return
		print( "send file header OK" )
		break

	# send file block
	start_time = time.time()
	file_offset = 0
	while ( file_offset < f_len ):
		#print( "==== %% %f"%(100*file_offset/f_len) )
		if ( (file_offset + FILE_BLOCK_SIZE) <  f_len ):
			send_file_size = FILE_BLOCK_SIZE
		else:
			send_file_size = f_len - file_offset

		file_offset_str = int2byte( file_offset,4,'little' )
		cmd_len_str = int2byte( (0x04 + send_file_size),2,'little' )
		# file_block_str = bytes_to_hex_str( bytes( f_d[file_offset:file_offset+send_file_size], encoding='utf-8' ) )
		file_block_str = bytes_to_hex_str( bytes( f_d[file_offset:file_offset+send_file_size] ) )
		frame_data_str = protocol_id_str + " " + dev_id_str + " " + srv_id_str + " " + file_block_cmd_id_str + " " + cmd_len_str + " " + file_offset_str + " " + file_block_str;
		frame_data_str = frame_data_str.replace(' ','')
		frame_data_len = len(binascii.a2b_hex(frame_data_str) )
		frame_data_len_str = int2byte(frame_data_len,2,'little' )
		frame_data_len_str = frame_data_len_str.replace(' ','')
		frame_head_checkusum_str = int2byte(calc_add_checksum(binascii.a2b_hex( frame_header_str+frame_data_len_str ) ),1,'little' )
		frame_checksum_str = int2byte( calc_add_checksum(binascii.a2b_hex( frame_data_str )),1,'little' )

		send_block_str = frame_header_str + " " + frame_head_checkusum_str + " " + frame_data_len_str + " " + frame_data_str + " " + frame_checksum_str + " " + frame_end_str
		send_block_str = send_block_str.replace(' ','')
		#print( send_block_str )
		# random product err
		send_block_bytes = binascii.a2b_hex( send_block_str);
		
		
		#if ( 5 == random.randint( 1, 10 ) ):
		#	print( "---->ERR send" )
		#	send_block_bytes[ random.randint(0, bytes.fromhex(cmd_len_str)[0] + 6) ] += 1

		ser.write( send_block_bytes )
		isResponse = False
		timeCount = 0
		while ( isResponse == False ):
			if(ProcessState != PROCESS_DONE):
				if(timeCount > 10):
					resend_count = resend_count+1
					timeCount = 0
					ser.write( send_block_bytes )
					print( "Resend file block:",file_offset,",",resend_count)

				if(resend_count > 2):
					ProcessState = PROCESS_DONE
					print( "Send file block Time Out" )
					f.close()
					return

		file_offset = file_offset + send_file_size
		print( "send file block",file_offset,"OK" )

	print( "===============================================================================================" )
	print( ">>>>>> Spend time : %d second"%( time.time() - start_time ), "avg tx speed: %d"% (file_offset/(time.time() - start_time)) )
	print( ">>>>>> Total cnt %d"%f_len )
	print( "===============================================================================================" )

	ProcessState = PROCESS_DONE
	f.close()
	pygame.mixer.music.load("/home/pi/project/codey_test/images/CodeDownloadSuc.wav")
	pygame.mixer.music.play()

# end fo send_file

def calibrate():
	global ProcessState
	print "calibrate"
	raw = cv2.imread("/home/pi/project/codey_test/outputs/raw.jpg")
	sp = raw.shape
	x = sp[1]
	y = sp[0]
	crop = raw.copy()[10:y-10, x/3:x]
	cv2.imwrite("/home/pi/project/codey_test/outputs"+'/crop.jpg', crop)
	ProcessState = IMAGE_PROCESS_WHITEBALANCE

def whiteBalance():
	global ProcessState
	print "whiteBalance"
	crop = cv2.imread("/home/pi/project/codey_test/outputs/crop.jpg")
	b,g,r = cv2.split(crop)
	B = cv2.mean(b)[0]
	G = cv2.mean(g)[0]
	R = cv2.mean(r)[0]
	KB = (R + G + B) / (3 * B)  
	KG = (R + G + B) / (3 * G)
	KR = (R + G + B) / (3 * R)

	b = np.uint8(b * KB)
	g = np.uint8(g * KG)
	r = np.uint8(r * KR)
	image = cv2.merge((b,g,r))

	cv2.imwrite("/home/pi/project/codey_test/outputs"+'/whiteBalance.jpg', image)

	rows,cols,chnl = image.shape
	M = cv2.getRotationMatrix2D((cols/2, rows/2), -90, 1)
	img = cv2.warpAffine(crop, M, (cols, rows),borderMode=cv2.BORDER_CONSTANT, borderValue=(255,255,255))
	cv2.imwrite("/home/pi/project/codey_test/outputs"+'/Rota.jpg', img)

	ProcessState = IMAGE_PROCESS_HSV

def hsvProcess():
	global ProcessState
	print "hsvProcess"
	ProcessState = IMAGE_PROCESS_GRAY

def grayProcess():
	global ProcessState
	print "grayProcess"
	#image = cv2.imread("/home/pi/project/codey_test/outputs/Rota.jpg")
	#gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	#cv2.imwrite("/home/pi/project/codey_test/outputs"+'/gray.jpg', image)
	ProcessState = IMAGE_PROCESS_THRESHOLD

def threshold():
	global ProcessState
	print "threshold"
	#image = cv2.imread("/home/pi/project/codey_test/outputs/gray.jpg")
	#res, thresh = cv2.threshold(image, 60, 255, cv2.THRESH_BINARY)
	#cv2.imwrite("/home/pi/project/codey_test/outputs"+'/threshold.jpg', thresh)
	ProcessState = IMAGE_PROCESS_CROP_GROUPS

def cropGroups():
	global ProcessState
	print "cropGroups"
	ProcessState = IMAGE_PROCESS_CROP_BLOCKS

def cropBlocks():
	kernel = np.ones((4,1),np.uint8) 
	raw_images = []
	contour_image = []
	
	global ProcessState
	global stateCount
	print "cropBlocks"

	img = cv2.imread('/home/pi/project/codey_test/outputs/Rota.jpg')  
	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	res, thresh = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY)
	cv2.imwrite("/home/pi/project/codey_test/outputs"+'/threshold.jpg', thresh)
	closing = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
	_, contours, _ = cv2.findContours(closing.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	mask = np.zeros(thresh.shape, np.uint8)
	cv2.drawContours(mask, contours, -1, (255), 3)
	contours_num = 0
	path = '/home/pi/project/codey_test/outputs/' + 'Group'
	cv2.imwrite('/home/pi/project/codey_test/outputs/closing.jpg', closing)
	for cnt in contours:
		x,y,w,h = cv2.boundingRect(cnt)
		if h<20 or w<15:
			continue
		if h>300 or w>300:
			continue
		print("--Yanminge-- [CropGroupsCommand]contours_num[%d]: x = %d,y = %d, w = %d,h = %d\r\n" %(contours_num,x,y,w,h))
		crop = thresh.copy()[y:(y+h), x:(x+w)]

		contour_image.append(XYImage(crop, x, y, (x/2, y/2), 0))
		contours_num = contours_num + 1

 	#对分组出来的照片进行冒泡排序
	for contour_i in range(0,contours_num):
		for contour_j in range(contour_i,contours_num):
			contour_i_xy = contour_image[contour_i].x + 6*(contour_image[contour_i].y)
			contour_j_xy = contour_image[contour_j].x + 6*(contour_image[contour_j].y)
			if(contour_i_xy > contour_j_xy):
				temp_contour_image = contour_image[contour_i]
				contour_image[contour_i] = contour_image[contour_j]
				contour_image[contour_j] = temp_contour_image

	for contour_i in range(0,contours_num):
		cv2.imwrite(path+'/' + str(contour_i) +'.jpg', contour_image[contour_i].image)
	stateCount = contours_num
	ProcessState = IMAGE_PROCESS_CREATE_PYTHON

def createPython():
	global ProcessState
	global stateCount
	commandbuf = []
	print "createPython"+str(stateCount)
	img = cv2.imread("/home/pi/project/codey_test/images/all.jpg",0)
	img_match = img.copy()
	for infile in range(0,stateCount):
		template = cv2.imread('/home/pi/project/codey_test/outputs/Group/'+str(infile)+'.jpg',0)
		w,h = template.shape[::-1]
		print '/home/pi/project/codey_test/outputs/Group/'+str(infile)+'.jpg'
		res = cv2.matchTemplate(img,template,cv2.TM_CCOEFF_NORMED)
		minVal,maxVal,minloc,maxloc = cv2.minMaxLoc(res)
		print "match Value:" + str(maxVal) + "--x:" + str(maxloc[0]) + ",y:" + str(maxloc[1])
		if maxVal > 0.80:
			top_left = maxloc
			buttom_right =(top_left[0]+w,top_left[1]+h)
			cv2.rectangle(img_match, top_left, buttom_right, (7,249,151), 2)
			cv2.imwrite('/home/pi/project/codey_test/outputs/match.jpg', img_match)
			commandbuf.append(top_left[0]/68)
	if os.path.exists('/home/pi/project/codey_test/main.py'):
		os.remove('/home/pi/project/codey_test/main.py')
	mode = 0777 
	os.mknod('/home/pi/project/codey_test/main.py',mode)
	file_obj = open('/home/pi/project/codey_test/main.py','a')
	file_obj.write("import codey\r\n")
	file_obj.write("import rocky\r\n")
	file_obj.write("codey.face('00001020402012020212204020100000',0.5)\r\n")
	file_obj.write("\r\n")
	file_obj.write("def on_button_callback():\r\n")
	for command in commandbuf:
		print "command ",str(command)
		if(command == 0):
			file_obj.write("	rocky.forward(50,0.5)\r\n")
		elif(command == 1):
			file_obj.write("	rocky.backward(50,0.5)\r\n")
		elif(command == 2):
			file_obj.write("	rocky.turn_left(50,0.5)\r\n")
		elif(command == 3):
			file_obj.write("	rocky.turn_right(50,0.5)\r\n")
		elif(command == 4):
			file_obj.write("	rocky.turn_right_angle(90)\r\n")
		elif(command == 5):
			file_obj.write("	rocky.turn_left_angle(90)\r\n")
		elif(command == 6):
			file_obj.write("	codey.say('cat.wav')\r\n")
		elif(command == 7):
			file_obj.write("	codey.say('dog.wav')\r\n")
		elif(command == 8):
			file_obj.write("	codey.face('000010387c3810000010387c38100000',0.5)\r\n")
			file_obj.write("	codey.say('sad.wav')\r\n")
		elif(command == 9):
			file_obj.write("	codey.face('0000363e1c3e360000363e1c3e360000',0.5)\r\n")
			file_obj.write("	codey.say('hurt.wav')\r\n")
		elif(command == 10):
			file_obj.write("	codey.face('00001020402012020212204020100000',0.5)\r\n")
			file_obj.write("	codey.say('cat.wav')\r\n")
	file_obj.write("\r\n")	
	file_obj.write("codey.on_button('A', on_button_callback)\r\n")
	file_obj.close()
	ProcessState = SEND_FILE

def send_task():
	global ProcessState
	while( True ):
		if(ProcessState == SEND_FILE):
			send_file( ser, READ_FILE_NAME, 0 )
	
		elif(ProcessState == IMAGE_PROCESS_CALIBRATE):
			calibrate()

		elif(ProcessState == IMAGE_PROCESS_WHITEBALANCE):
			whiteBalance()

		elif(ProcessState == IMAGE_PROCESS_HSV):
			hsvProcess()

		elif(ProcessState == IMAGE_PROCESS_GRAY):
			grayProcess()
	
		elif(ProcessState == IMAGE_PROCESS_THRESHOLD):
			threshold()
	
		elif(ProcessState == IMAGE_PROCESS_CROP_GROUPS):
			cropGroups()

		elif(ProcessState == IMAGE_PROCESS_CROP_BLOCKS):
			cropBlocks()

		elif(ProcessState == IMAGE_PROCESS_CREATE_PYTHON):
			createPython()

		time.sleep(0.5)

def timeCount_task():
	global timeCount
	while( True ):
		timeCount = timeCount + 1
		time.sleep(0.2)
		#print(timeCount)

def imageProcess_task():
	global ProcessState
	global timeCount
	global clicked
	global isWindowsInit
	while( True ):
		if(ProcessState == BUTTON_START):
			print 'Showing camera feed. click window or press any key to stop.'
			if(isWindowsInit == False):
				cameraCapture = cv2.VideoCapture(0)
				cv2.namedWindow('MyWindow')
				cv2.setMouseCallback('MyWindow',onMouse)
				print 'init windows.'
				isWindowsInit = True
			ProcessState = TEST_STEP

		if(ProcessState == TEST_STEP):
			success = True
			timeCount = 0
			while success:
				if(clicked and isWindowsInit == True):
					print 'destroy windows.'
					cameraCapture.release()
					cv2.destroyWindow('MyWindow')
					isWindowsInit = False
					clicked = False
					ProcessState = IMAGE_PROCESS_CALIBRATE
					break
				if ProcessState != TEST_STEP:
					break
				if(isWindowsInit == True):
					success,frame = cameraCapture.read()
					if(success):
						print 'show windows.'
						cv2.imshow('MyWindow',frame)
						if timeCount > 20:
							cv2.imwrite("/home/pi/project/codey_test/outputs"+'/raw.jpg', frame)	
							clicked = True	
				cv2.waitKey(1)
		time.sleep(0.5)

def buttonCheck_task():
	global isButtonPressed
	global ProcessState
	btnStart = 21
	RPi.GPIO.setmode(RPi.GPIO.BCM)
	RPi.GPIO.setup(btnStart,RPi.GPIO.IN,pull_up_down = RPi.GPIO.PUD_UP)
	try:
		while( True ):
			if(RPi.GPIO.input(btnStart) == 0):
				if isButtonPressed == False:
					isButtonPressed = True
					print("button press!",ProcessState)
					if ProcessState == PROCESS_DONE or ProcessState == TEST_STEP:
						ProcessState = BUTTON_START
						pygame.mixer.music.load("/home/pi/project/codey_test/images/startWork.wav")
						pygame.mixer.music.play()
						#ProcessState = SEND_FILE
			else:
				if(isButtonPressed == True):
					isButtonPressed = False
			time.sleep(0.1)
	except KeyboardInterrupt:
		pass

class FtpFsm( object ):
	global isResponse
	def __init__( self ):
		self.__state = FTP_FSM_HEAD_S
		self.__buf = []
		self.__data_len = 0
		self.__cur_data_len = 0
		self.__checksum = 0x00
		self.__headchecksum = 0x00
		self.__recv_head_checksum = 0x00

	def get_state( self ):
		return self.__state

	def set_state( self, s ):
		self.__state = s

	def push_char( self, c ):
		c = ord(c)
		if ( FTP_FSM_HEAD_S == self.__state ):
			if ( FRAME_HEAD == c ):
				self.__state = FTP_FSM_HEAD_CHECK_S
				del self.__buf
				self.__buf = []
				self.__checksum = 0
				self.__headchecksum = c

		elif ( FTP_FSM_HEAD_CHECK_S == self.__state ):
			self.__recv_head_checksum = c
			self.__state = FTP_FSM_LEN1_S

		elif( FTP_FSM_LEN1_S == self.__state ):
			self.__headchecksum += c
			self.__data_len = c
			self.__state = FTP_FSM_LEN2_S

		elif( FTP_FSM_LEN2_S == self.__state ):
			self.__headchecksum += c
			if ( self.__headchecksum == self.__recv_head_checksum ):
				self.__data_len += c*0xff
				self.__state = FTP_FSM_DATA_S
			else:
				self.__state = FTP_FSM_HEAD_S

		elif( FTP_FSM_DATA_S == self.__state ):
			self.__checksum += c
			self.__buf.append( c )
			if ( len(self.__buf) == self.__data_len ):
				self.__state = FTP_FSM_CHECK_S
				# print( "expect checksum %02x"%(self.__checksum & 0xFF) )

		elif( FTP_FSM_CHECK_S == self.__state ):
			if ( (self.__checksum & 0xFF) == c ):
				self.__state = FTP_FSM_END_S
			else:
				self.__state = FTP_FSM_HEAD_S
				
		elif( FTP_FSM_END_S == self.__state ):
			if ( FRAME_END == c ):
				self.__state = FTP_FSM_HEAD_S
				return self.__buf
			else:
				self.__state = FTP_FSM_HEAD_S 

	def clear_buf( self ):
		del self.__buf
		self.__buf = []

	def get_buf( self ):
		return self.__buf

ftp_fsm = FtpFsm()

# clear all the data in serial buffer
ser.read( ser.inWaiting() )
time.sleep( 0.5)
pygame.mixer.init()
pygame.mixer.music.load("/home/pi/project/codey_test/images/readyWork.wav")
pygame.mixer.music.play()
sendTask = threading.Thread(target = send_task)
imageProcessTask = threading.Thread(target = imageProcess_task)
timeCountTask = threading.Thread(target = timeCount_task)
buttonCheckTask = threading.Thread(target = buttonCheck_task)
sendTask.start()
timeCountTask.start()
imageProcessTask.start()
buttonCheckTask.start()

while( True ):
	if ( ser.inWaiting() ):
		r_b = ser.read( ser.inWaiting() )
		for c in r_b:
			#print( "%c"%(c))
			buf_list = ftp_fsm.push_char( c )
			if ( type(buf_list) == list ):
				#print( " #################################### " )
				#print( buf_list )
				# protocol id is 0x01 and command status code is 0x00
				if ( 0x01 == buf_list[0] and 0x00 == buf_list[6] ):
					isResponse = True
					pass
				ftp_fsm.clear_buf()
			#if ( is_display(c) ):
			#	print( "%c"%(c), end='' )
			#else:
			#	print( "%02x"%(c), end=' ' )

#print( "***********************************************************************" )

# t.stop()
