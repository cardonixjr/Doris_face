import cv2
import numpy as np
import sys
from PyDynamixel import DxlComm, Joint

# Funcao que calcula a distancia da pessoa mais proxima para a camera
f = lambda l: -1.43*x + 258

# Inicia comunicacao da porta serial
port = DxlComm("/dev/ttyUSB0")

neck_h = Joint(62)
neck_v = Joint(61)

face_cascade = cv2.CascadeClassifier("haar_cascade_face_default.xml")

cap = cv2.VideoCapture(0)

while True:
	ret, img = cap.read()
	height, width, layers = img.shape
	img_center = [width/2, height/2]

	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	faces = face_cascade.detectMultiScale(gray, 1.3, 5)

	# A larger rectangle area means that the person face is closer to the robot face
	larger_area = 0
	larger_area_center = []
	smaller_distance = 0
	for(x,y,w,h) in faces:
		center = [(x+x+w)/2, (y+y+h)/2]
		area = w*h

		if area > larger_area:
			larger_area = area
			larger_area_center = center
			smaller_distance = f(w)

		cv2.rectangle(img, (x,y),(x+w, y+h), (255,0,0), 2)
		roi_gray = gray[y:y+h, x:x+w]
		roi_color = img[y:y+h, x:x+w]

	width_limits = [0.3*width, 0.7*width, 0.3*height, 0.7*height]

	# Center the camera
	if larger_area_center != []:

		# Receive the neck motors angles
		theta_x = neck_h.receiveCurrAngle()
		theta_y = neck_v.receiveCurrAngle()

		if larger_area_center[0] > width_limits[1]:
			# Significa que a pessoa esta muito para a esquerda na imagem
			print(theta_x - 1)
			# co = img_center[0] - larger_area_center[0]
			# theta = theta_x + np.arctan(co/smaller_distance)
			# print(theta)
			# print("vai pra direita")

		if larger_area_center[0] < width_limits[0]:
			# Significa que a pessoa esta muito para a direita na imagem
			print(theta_x + 1)
		# 	co = img_center[0] - larger_area_center[0]
		# 	theta = theta_x - np.arctan(co/smaller_distance)
		# 	print(theta)
		# 	print("vai pra esquerda")

		if larger_area_center[1] > width_limits[3]:
		 	# Significa que a pessoa esta muito para baixo na imagem
			print(theta_y - 1)
		# 	co = img_center[1] - larger_area_center[1]
		# 	theta = theta_y - np.arctan(co/smaller_distance)
		# 	print(theta)
		# 	print("vai pra cima")
			
		if larger_area_center[1] < width_limits[2]:
			# Significa que a pessoa esta muito para cima na image
			print(theta_y + 1)
		# 	co = larger_area_center[1] - img_center[1]
		# 	theta = theta_y + np.arctan(co/smaller_distance)		
		# 	print(theta)
		# 	print("vai pra baixo")

	cv2.imshow("screen", img)

	k = cv2.waitKey(30) & 0xff

	if k == 26:
		break

cap.release()
cv2.destroyAllWindows()